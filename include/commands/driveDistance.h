#pragma once

#include "command/command.h"
#include "config.h"
#include "controllers/pidController.h"
#include "subsystems/drivetrain.h"
#include "subsystems/localization.h"

#include "pros/rtos.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

// Drive a signed distance (in) while holding a field heading (rad).
class DriveDistance : public Command {
public:
    DriveDistance(DrivetrainSubsystem* drivetrain, LocalizationSubsystem* localization, double distanceIn,
                  double headingRad = INFINITY, bool finish = true, int timeoutMs = 0)
        : drivetrain_(drivetrain),
          localization_(localization),
          distanceIn_(distanceIn),
          headingRad_(headingRad),
          finish_(finish),
          timeoutMs_(timeoutMs) {}

    void initialize() override {
        const auto pose = localization_->getPose();
        startX_ = pose.x();
        startY_ = pose.y();
        if (std::isinf(headingRad_)) headingRad_ = pose.z();

        drivePid_ = PID(CONFIG::DRIVE_PID);
        drivePid_.setOutputLimit(-CONFIG::DRIVE_OUTPUT_LIMIT, CONFIG::DRIVE_OUTPUT_LIMIT);
        drivePid_.setSetpoint(distanceIn_);
        drivePid_.reset();

        headingPid_ = PID(CONFIG::DRIVE_HEADING_PID);
        headingPid_.enableContinuousInput(-CONFIG::PI, CONFIG::PI);
        headingPid_.setOutputLimit(-CONFIG::TURN_OUTPUT_LIMIT, CONFIG::TURN_OUTPUT_LIMIT);
        headingPid_.setSetpoint(headingRad_);
        headingPid_.reset();

        lastMs_ = pros::millis();
        startMs_ = lastMs_;
        settleStartMs_ = 0;
        overshot_ = false;
        approachSign_ = 0.0;
    }

    void execute() override {
        const std::uint32_t now = pros::millis();
        const double dt = std::max((now - lastMs_) * 0.001, 0.001);
        lastMs_ = now;

        const double traveled = traveledIn();
        const double err = distanceIn_ - traveled;
        noteOvershoot(err, drivePid_);

        double linear = drivePid_.calculate(traveled, dt);
        linear = correctedOutput(linear, err, exitTolerance(), CONFIG::DRIVE_CORRECTION_MIN, overshot_);
        const double angular = headingPid_.calculate(localization_->getAngle(), dt);
        drivetrain_->setPct(linear + angular, linear - angular);
    }

    void end(bool /*interrupted*/) override { drivetrain_->stop(); }

    bool isFinished() override {
        if (timedOut()) return true;
        if (!finish_) return false;
        const double err = std::fabs(distanceIn_ - traveledIn());
        if (err > exitTolerance()) {
            settleStartMs_ = 0;
            return false;
        }
        if (settleStartMs_ == 0) settleStartMs_ = pros::millis();
        return (pros::millis() - settleStartMs_) >= static_cast<std::uint32_t>(CONFIG::SETTLE_MS);
    }

    std::vector<Subsystem*> getRequirements() override { return {drivetrain_}; }

private:
    double traveledIn() const {
        const auto pose = localization_->getPose();
        const double s = std::sin(headingRad_);
        const double c = std::cos(headingRad_);
        return (pose.x() - startX_) * s + (pose.y() - startY_) * c;  // in
    }

    double exitTolerance() const {
        return overshot_ ? CONFIG::DISTANCE_CORRECTION_IN : CONFIG::DISTANCE_FINISH_IN;
    }

    bool timedOut() const {
        return timeoutMs_ > 0 && (pros::millis() - startMs_) >= static_cast<std::uint32_t>(timeoutMs_);
    }

    // Latch the first time the remaining distance changes sign, then drive back.
    void noteOvershoot(double err, PID& pid) {
        if (overshot_) return;
        if (approachSign_ == 0.0) {
            if (std::fabs(err) > 1e-4) approachSign_ = std::copysign(1.0, err);
            return;
        }
        if (err * approachSign_ < 0.0) {
            overshot_ = true;
            settleStartMs_ = 0;
            pid.reset();
        }
    }

    static double correctedOutput(double output, double err, double tolerance, double minOutput, bool correcting) {
        if (!correcting || std::fabs(err) <= tolerance) return output;
        if (output * err > 0.0 && std::fabs(output) >= minOutput) return output;
        return std::copysign(minOutput, err);
    }

    DrivetrainSubsystem* drivetrain_;
    LocalizationSubsystem* localization_;
    double distanceIn_;  // in
    double headingRad_;  // rad
    bool finish_;
    int timeoutMs_;  // ms; <= 0 disables
    double startX_ = 0.0;  // in
    double startY_ = 0.0;  // in
    PID drivePid_{CONFIG::DRIVE_PID};
    PID headingPid_{CONFIG::DRIVE_HEADING_PID};
    std::uint32_t lastMs_ = 0;
    std::uint32_t startMs_ = 0;
    std::uint32_t settleStartMs_ = 0;
    bool overshot_ = false;
    double approachSign_ = 0.0;
};
