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
                  double headingRad = INFINITY, bool finish = true)
        : drivetrain_(drivetrain),
          localization_(localization),
          distanceIn_(distanceIn),
          headingRad_(headingRad),
          finish_(finish) {}

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
        settleStartMs_ = 0;
    }

    void execute() override {
        const std::uint32_t now = pros::millis();
        const double dt = std::max((now - lastMs_) * 0.001, 0.001);
        lastMs_ = now;

        const double linear = drivePid_.calculate(traveledIn(), dt);
        const double angular = headingPid_.calculate(localization_->getAngle(), dt);
        drivetrain_->setPct(linear + angular, linear - angular);
    }

    void end(bool /*interrupted*/) override { drivetrain_->stop(); }

    bool isFinished() override {
        if (!finish_) return false;
        const double err = std::fabs(distanceIn_ - traveledIn());
        if (err > CONFIG::DISTANCE_FINISH_IN) {
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

    DrivetrainSubsystem* drivetrain_;
    LocalizationSubsystem* localization_;
    double distanceIn_;  // in
    double headingRad_;  // rad
    bool finish_;
    double startX_ = 0.0;  // in
    double startY_ = 0.0;  // in
    PID drivePid_{CONFIG::DRIVE_PID};
    PID headingPid_{CONFIG::DRIVE_HEADING_PID};
    std::uint32_t lastMs_ = 0;
    std::uint32_t settleStartMs_ = 0;
};
