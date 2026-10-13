#pragma once

#include "command/command.h"
#include "commands/motionUtil.h"
#include "config.h"
#include "controllers/pidController.h"
#include "subsystems/drivetrain.h"
#include "subsystems/localization.h"

#include <algorithm>
#include <cmath>

// Point-turn to a field heading. Factories pass degrees; this command stores radians.
// Speeds are pct of 12 V. A nonzero minSpeed enables motion chaining: the turn hands off at speed
// once within earlyExitRad of the target, or once it passes it.
class Rotate : public Command {
public:
    Rotate(DrivetrainSubsystem* drivetrain, LocalizationSubsystem* localization, double headingRad, int timeoutMs = 0,
           double maxSpeed = CONFIG::TURN_MAX_SPEED, double minSpeed = 0.0, double earlyExitRad = 0.0)
        : drivetrain_(drivetrain),
          localization_(localization),
          targetRad_(headingRad),
          maxSpeed_(maxSpeed),
          minSpeed_(minSpeed),
          earlyExitRad_(earlyExitRad),
          timeout_(timeoutMs) {}

    void initialize() override {
        pid_.reset();
        smallExit_.reset();
        largeExit_.reset();
        timeout_.start();
        prevError_ = error();
        prevOutput_ = 0.0;
        chained_ = false;
    }

    void execute() override {
        const double err = error();

        // Motion chaining: hand off at speed once inside the early exit range or past the target.
        if (minSpeed_ != 0.0 && (std::fabs(err) < earlyExitRad_ || std::signbit(err) != std::signbit(prevError_))) {
            chained_ = true;
            return;
        }
        prevError_ = err;

        smallExit_.update(err);
        largeExit_.update(err);

        double out = std::clamp(pid_.update(err), -maxSpeed_, maxSpeed_);
        out = motion::slew(out, prevOutput_, CONFIG::ANGULAR_CONTROLLER.slew);
        out = motion::applyMinSpeed(out, minSpeed_);
        prevOutput_ = out;

        drivetrain_->setPct(out, -out);
    }

    void end(bool /*interrupted*/) override { drivetrain_->stop(); }

    bool isFinished() override {
        return chained_ || timeout_.isDone() || smallExit_.getExit() || largeExit_.getExit();
    }

    std::vector<Subsystem*> getRequirements() override { return {drivetrain_}; }

private:
    double error() const { return CONFIG::wrapPi(targetRad_ - localization_->getAngle()); }

    DrivetrainSubsystem* drivetrain_;
    LocalizationSubsystem* localization_;
    double targetRad_;  // rad
    double maxSpeed_;      // pct of 12 V
    double minSpeed_;      // pct of 12 V
    double earlyExitRad_;  // rad
    motion::Timeout timeout_;

    PID pid_{CONFIG::ANGULAR_CONTROLLER};
    ExitCondition smallExit_{CONFIG::ANGULAR_CONTROLLER.smallError, CONFIG::ANGULAR_CONTROLLER.smallErrorTimeoutMs};
    ExitCondition largeExit_{CONFIG::ANGULAR_CONTROLLER.largeError, CONFIG::ANGULAR_CONTROLLER.largeErrorTimeoutMs};

    double prevError_ = 0.0;   // rad
    double prevOutput_ = 0.0;  // pct of 12 V
    bool chained_ = false;
};
