#pragma once

#include "command/command.h"
#include "config.h"
#include "controllers/pidController.h"
#include "subsystems/drivetrain.h"

#include "pros/rtos.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

// Turn to face (x, y) then drive there. Target is computed in initialize() from the current pose.
class DriveToPoint : public Command {
public:
    DriveToPoint(DrivetrainSubsystem* drivetrain, double xIn, double yIn, bool finish = true)
        : drivetrain_(drivetrain), targetX_(xIn), targetY_(yIn), finish_(finish) {}

    void initialize() override {
        const auto pose = drivetrain_->getPose();
        startX_ = pose.x();
        startY_ = pose.y();
        const double dx = targetX_ - startX_;
        const double dy = targetY_ - startY_;
        distanceIn_ = std::hypot(dx, dy);
        headingRad_ = std::atan2(dx, dy);  // CW from +Y

        turnPid_ = PID(CONFIG::TURN_PID);
        turnPid_.enableContinuousInput(-CONFIG::PI, CONFIG::PI);
        turnPid_.setOutputLimit(-CONFIG::TURN_OUTPUT_LIMIT, CONFIG::TURN_OUTPUT_LIMIT);
        turnPid_.setSetpoint(headingRad_);
        turnPid_.reset();

        drivePid_ = PID(CONFIG::DRIVE_PID);
        drivePid_.setOutputLimit(-CONFIG::DRIVE_OUTPUT_LIMIT, CONFIG::DRIVE_OUTPUT_LIMIT);
        drivePid_.setSetpoint(distanceIn_);
        drivePid_.reset();

        headingPid_ = PID(CONFIG::DRIVE_HEADING_PID);
        headingPid_.enableContinuousInput(-CONFIG::PI, CONFIG::PI);
        headingPid_.setOutputLimit(-CONFIG::TURN_OUTPUT_LIMIT, CONFIG::TURN_OUTPUT_LIMIT);
        headingPid_.setSetpoint(headingRad_);
        headingPid_.reset();

        const double headingErr = std::fabs(CONFIG::wrapPi(headingRad_ - pose.z()));
        phase_ = (distanceIn_ < CONFIG::DISTANCE_FINISH_IN)
                     ? Phase::kDone
                     : (headingErr < CONFIG::ANGLE_FINISH_RAD ? Phase::kDrive : Phase::kTurn);

        lastMs_ = pros::millis();
        settleStartMs_ = 0;
    }

    void execute() override {
        if (phase_ == Phase::kDone) {
            drivetrain_->stop();
            return;
        }

        const std::uint32_t now = pros::millis();
        const double dt = std::max((now - lastMs_) * 0.001, 0.001);
        lastMs_ = now;

        if (phase_ == Phase::kTurn) {
            const double out = turnPid_.calculate(drivetrain_->getAngle(), dt);
            drivetrain_->setPct(out, -out);
            const double headingErr = std::fabs(CONFIG::wrapPi(headingRad_ - drivetrain_->getAngle()));
            if (headingErr < CONFIG::ANGLE_FINISH_RAD) {
                phase_ = Phase::kDrive;
                drivePid_.reset();
                headingPid_.reset();
                lastMs_ = now;
            }
            return;
        }

        const double linear = drivePid_.calculate(traveledIn(), dt);
        const double angular = headingPid_.calculate(drivetrain_->getAngle(), dt);
        drivetrain_->setPct(linear + angular, linear - angular);
    }

    void end(bool /*interrupted*/) override { drivetrain_->stop(); }

    bool isFinished() override {
        if (!finish_) return false;
        if (phase_ == Phase::kDone) return true;
        if (phase_ != Phase::kDrive) return false;
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
    enum class Phase { kTurn, kDrive, kDone };

    double traveledIn() const {
        const auto pose = drivetrain_->getPose();
        const double s = std::sin(headingRad_);
        const double c = std::cos(headingRad_);
        return (pose.x() - startX_) * s + (pose.y() - startY_) * c;  // in
    }

    DrivetrainSubsystem* drivetrain_;
    double targetX_;  // in
    double targetY_;  // in
    bool finish_;
    double startX_ = 0.0;      // in
    double startY_ = 0.0;      // in
    double distanceIn_ = 0.0;  // in
    double headingRad_ = 0.0;  // rad
    Phase phase_ = Phase::kTurn;
    PID turnPid_{CONFIG::TURN_PID};
    PID drivePid_{CONFIG::DRIVE_PID};
    PID headingPid_{CONFIG::DRIVE_HEADING_PID};
    std::uint32_t lastMs_ = 0;
    std::uint32_t settleStartMs_ = 0;
};
