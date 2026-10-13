#pragma once

#include "command/command.h"
#include "commands/motionUtil.h"
#include "commands/rotate.h"
#include "config.h"
#include "controllers/pidController.h"
#include "subsystems/drivetrain.h"
#include "subsystems/localization.h"

#include "pros/rtos.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>

// Turn to face (x, y), pause, then drive there steering toward the point. Heading is locked once close.
// Timeouts are per stage, in ms; <= 0 disables that limit. Speeds are pct of 12 V. A nonzero minSpeed
// enables motion chaining: the drive hands off at speed once within earlyExitIn of the target.
class DriveToPoint : public Command {
public:
    DriveToPoint(DrivetrainSubsystem* drivetrain, LocalizationSubsystem* localization, double xIn, double yIn,
                 int turnTimeoutMs = 0, int driveTimeoutMs = 0, bool forwards = true,
                 double maxSpeed = CONFIG::DRIVE_MAX_SPEED, double minSpeed = 0.0, double earlyExitIn = 0.0,
                 int pauseMs = 75)
        : drivetrain_(drivetrain),
          localization_(localization),
          targetX_(xIn),
          targetY_(yIn),
          forwards_(forwards),
          maxSpeed_(maxSpeed),
          minSpeed_(minSpeed),
          earlyExitIn_(earlyExitIn),
          pauseMs_(pauseMs),
          turnTimeoutMs_(turnTimeoutMs),
          driveTimeout_(driveTimeoutMs) {}

    void initialize() override {
        turn_.reset();

        const auto pose = localization_->getPose();
        if (std::hypot(targetX_ - pose.x(), targetY_ - pose.y()) < CONFIG::LATERAL_CONTROLLER.smallError) {
            phase_ = Phase::kDone;
            return;
        }

        const double facing = CONFIG::wrapPi(angleTo(pose) + (forwards_ ? 0.0 : CONFIG::PI));
        if (std::fabs(CONFIG::wrapPi(facing - pose.z())) < CONFIG::ANGULAR_CONTROLLER.smallError) {
            beginPause();
            return;
        }

        phase_ = Phase::kTurn;
        turn_.emplace(drivetrain_, localization_, facing, turnTimeoutMs_);
        turn_->initialize();
    }

    void execute() override {
        switch (phase_) {
            case Phase::kTurn:
                turn_->execute();
                if (turn_->isFinished()) {
                    turn_->end(false);
                    beginPause();
                }
                break;
            case Phase::kPause:
                if (pros::millis() - pauseStartMs_ >= static_cast<std::uint32_t>(pauseMs_)) beginDrive();
                break;
            case Phase::kDrive:
                driveStep();
                break;
            case Phase::kDone:
                break;
        }
    }

    void end(bool /*interrupted*/) override { drivetrain_->stop(); }

    bool isFinished() override {
        if (phase_ == Phase::kDone) return true;
        if (phase_ != Phase::kDrive) return false;
        return driveTimeout_.isDone() || (close_ && (smallExit_.getExit() || largeExit_.getExit()));
    }

    std::vector<Subsystem*> getRequirements() override { return {drivetrain_}; }

private:
    enum class Phase { kTurn, kPause, kDrive, kDone };

    double angleTo(const Eigen::Vector3f& pose) const {
        return std::atan2(targetX_ - pose.x(), targetY_ - pose.y());  // rad, CW from +Y
    }

    void beginPause() {
        if (pauseMs_ <= 0) {
            beginDrive();
            return;
        }
        phase_ = Phase::kPause;
        pauseStartMs_ = pros::millis();
    }

    void beginDrive() {
        phase_ = Phase::kDrive;
        driveTimeout_.start();
        pathHeading_ = angleTo(localization_->getPose());
        lateralPid_.reset();
        headingPid_.reset();
        smallExit_.reset();
        largeExit_.reset();
        prevLateral_ = 0.0;
        close_ = false;
    }

    void driveStep() {
        const auto pose = localization_->getPose();
        const double dx = targetX_ - pose.x();
        const double dy = targetY_ - pose.y();
        if (std::hypot(dx, dy) < CONFIG::DRIVE_CLOSE_IN) close_ = true;

        // Motion chaining: hand off at speed once within earlyExitRange of the line through the target,
        // perpendicular to the path.
        const double alongPath = dx * std::sin(pathHeading_) + dy * std::cos(pathHeading_);  // in
        if (minSpeed_ != 0.0 && alongPath <= earlyExitIn_) {
            phase_ = Phase::kDone;
            return;
        }

        const double toTarget = angleTo(pose);
        const double direction = forwards_ ? 1.0 : -1.0;
        const double facing = pose.z() + (forwards_ ? 0.0 : CONFIG::PI);
        const double lateralError = std::hypot(dx, dy) * std::cos(CONFIG::wrapPi(toTarget - pose.z()));  // in

        smallExit_.update(lateralError);
        largeExit_.update(lateralError);

        double angular = 0.0;  // heading is locked once close, where the angle to the target is unstable
        if (!close_) {
            angular = std::clamp(headingPid_.update(CONFIG::wrapPi(toTarget - facing)), -maxSpeed_,
                                 maxSpeed_);
        }
        double lateral = motion::shapeLateral(lateralPid_.update(lateralError), prevLateral_, direction, close_,
                                              maxSpeed_, minSpeed_, CONFIG::LATERAL_CONTROLLER.slew);
        lateral = motion::prioritizeAngular(lateral, angular, maxSpeed_);
        prevLateral_ = lateral;

        drivetrain_->setPct(lateral + angular, lateral - angular);
    }

    DrivetrainSubsystem* drivetrain_;
    LocalizationSubsystem* localization_;
    double targetX_;  // in
    double targetY_;  // in
    bool forwards_;
    double maxSpeed_;     // pct of 12 V
    double minSpeed_;     // pct of 12 V
    double earlyExitIn_;  // in
    int pauseMs_;         // ms between the turn and the drive
    int turnTimeoutMs_;  // ms; <= 0 disables
    motion::Timeout driveTimeout_;

    std::optional<Rotate> turn_;
    PID lateralPid_{CONFIG::LATERAL_CONTROLLER};
    PID headingPid_{CONFIG::HEADING_CONTROLLER};
    ExitCondition smallExit_{CONFIG::LATERAL_CONTROLLER.smallError, CONFIG::LATERAL_CONTROLLER.smallErrorTimeoutMs};
    ExitCondition largeExit_{CONFIG::LATERAL_CONTROLLER.largeError, CONFIG::LATERAL_CONTROLLER.largeErrorTimeoutMs};

    Phase phase_ = Phase::kTurn;
    std::uint32_t pauseStartMs_ = 0;
    double pathHeading_ = 0.0;  // rad; direction from the drive start to the target
    double prevLateral_ = 0.0;
    bool close_ = false;
};
