#pragma once

#include "command/command.h"
#include "commands/motionUtil.h"
#include "config.h"
#include "controllers/pidController.h"
#include "subsystems/drivetrain.h"
#include "subsystems/localization.h"

#include <algorithm>
#include <cmath>

// Drive a signed distance (in) while holding a field heading (rad). Defaults to the heading at start.
// Speeds are pct of 12 V. A nonzero minSpeed enables motion chaining: the drive hands off at speed
// once within earlyExitIn of the target.
class DriveDistance : public Command {
public:
    DriveDistance(DrivetrainSubsystem* drivetrain, LocalizationSubsystem* localization, double distanceIn,
                  double headingRad = INFINITY, int timeoutMs = 0, double maxSpeed = CONFIG::DRIVE_MAX_SPEED,
                  double minSpeed = 0.0, double earlyExitIn = 0.0)
        : drivetrain_(drivetrain),
          localization_(localization),
          distanceIn_(distanceIn),
          headingRad_(headingRad),
          maxSpeed_(maxSpeed),
          minSpeed_(minSpeed),
          earlyExitIn_(earlyExitIn),
          timeout_(timeoutMs) {}

    void initialize() override {
        const auto pose = localization_->getPose();
        heading_ = std::isinf(headingRad_) ? pose.z() : headingRad_;
        targetX_ = pose.x() + distanceIn_ * std::sin(heading_);
        targetY_ = pose.y() + distanceIn_ * std::cos(heading_);
        direction_ = distanceIn_ < 0.0 ? -1.0 : 1.0;

        lateralPid_.reset();
        headingPid_.reset();
        smallExit_.reset();
        largeExit_.reset();
        timeout_.start();
        prevLateral_ = 0.0;
        close_ = false;
        chained_ = false;
    }

    void execute() override {
        const auto pose = localization_->getPose();
        const double remaining = (targetX_ - pose.x()) * std::sin(heading_) +
                                 (targetY_ - pose.y()) * std::cos(heading_);  // in, + is ahead along heading
        if (std::fabs(remaining) < CONFIG::DRIVE_CLOSE_IN) close_ = true;

        // Motion chaining: hand off at speed once within earlyExitRange of the target line.
        if (minSpeed_ != 0.0 && remaining * direction_ <= earlyExitIn_) {
            chained_ = true;
            return;
        }

        smallExit_.update(remaining);
        largeExit_.update(remaining);

        const double angular = std::clamp(headingPid_.update(CONFIG::wrapPi(heading_ - pose.z())), -maxSpeed_,
                                          maxSpeed_);
        double lateral = motion::shapeLateral(lateralPid_.update(remaining), prevLateral_, direction_, close_,
                                              maxSpeed_, minSpeed_, CONFIG::LATERAL_CONTROLLER.slew);
        lateral = motion::prioritizeAngular(lateral, angular, maxSpeed_);
        prevLateral_ = lateral;

        drivetrain_->setPct(lateral + angular, lateral - angular);
    }

    void end(bool /*interrupted*/) override { drivetrain_->stop(); }

    bool isFinished() override {
        return chained_ || timeout_.isDone() || smallExit_.getExit() || largeExit_.getExit();
    }

    std::vector<Subsystem*> getRequirements() override { return {drivetrain_}; }

private:
    DrivetrainSubsystem* drivetrain_;
    LocalizationSubsystem* localization_;
    double distanceIn_;  // in
    double headingRad_;  // rad; INFINITY holds the heading at start
    double maxSpeed_;     // pct of 12 V
    double minSpeed_;     // pct of 12 V
    double earlyExitIn_;  // in
    motion::Timeout timeout_;

    PID lateralPid_{CONFIG::LATERAL_CONTROLLER};
    PID headingPid_{CONFIG::HEADING_CONTROLLER};
    ExitCondition smallExit_{CONFIG::LATERAL_CONTROLLER.smallError, CONFIG::LATERAL_CONTROLLER.smallErrorTimeoutMs};
    ExitCondition largeExit_{CONFIG::LATERAL_CONTROLLER.largeError, CONFIG::LATERAL_CONTROLLER.largeErrorTimeoutMs};

    double heading_ = 0.0;    // rad
    double targetX_ = 0.0;    // in
    double targetY_ = 0.0;    // in
    double direction_ = 1.0;  // +1 forwards, -1 backwards
    double prevLateral_ = 0.0;
    bool close_ = false;
    bool chained_ = false;
};
