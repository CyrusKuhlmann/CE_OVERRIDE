#pragma once

#include "command/command.h"
#include "command/waitCommand.h"
#include "commands/driveDistance.h"
#include "commands/rotate.h"
#include "config.h"
#include "subsystems/drivetrain.h"
#include "subsystems/localization.h"

#include <cmath>
#include <optional>

// Turn to face (x, y), pause, then drive there. Target is computed in initialize() from the current pose.
// Timeouts are per stage, in ms; <= 0 disables that limit.
class DriveToPoint : public Command {
public:
    DriveToPoint(DrivetrainSubsystem* drivetrain, LocalizationSubsystem* localization, double xIn, double yIn,
                 bool finish = true, int turnTimeoutMs = 0, int driveTimeoutMs = 0, int pauseMs = 75)
        : drivetrain_(drivetrain),
          localization_(localization),
          targetX_(xIn),
          targetY_(yIn),
          finish_(finish),
          turnTimeoutMs_(turnTimeoutMs),
          driveTimeoutMs_(driveTimeoutMs),
          pauseMs_(pauseMs) {}

    void initialize() override {
        turn_.reset();
        pause_.reset();
        drive_.reset();
        driveStarted_ = false;

        const auto pose = localization_->getPose();
        const double dx = targetX_ - pose.x();
        const double dy = targetY_ - pose.y();
        if (std::hypot(dx, dy) < CONFIG::DISTANCE_FINISH_IN) {
            phase_ = Phase::kDone;
            return;
        }

        headingRad_ = std::atan2(dx, dy);  // CW from +Y
        const double headingErr = std::fabs(CONFIG::wrapPi(headingRad_ - pose.z()));
        if (headingErr < CONFIG::ANGLE_FINISH_RAD) {
            beginPauseOrDrive();
            return;
        }

        phase_ = Phase::kTurn;
        turn_.emplace(drivetrain_, localization_, headingRad_, true, turnTimeoutMs_);
        turn_->initialize();
    }

    void execute() override {
        if (phase_ == Phase::kDone) {
            drivetrain_->stop();
            return;
        }

        Command* cmd = active();
        cmd->execute();
        if (!cmd->isFinished()) return;

        cmd->end(false);
        if (phase_ == Phase::kTurn) {
            beginPauseOrDrive();
        } else if (phase_ == Phase::kPause) {
            beginDrive();
        } else {
            phase_ = Phase::kDone;
        }
    }

    void end(bool interrupted) override {
        if (Command* cmd = active()) cmd->end(interrupted);
        else drivetrain_->stop();
    }

    bool isFinished() override {
        if (phase_ != Phase::kDone) return false;
        if (!driveStarted_) return finish_;
        return true;
    }

    std::vector<Subsystem*> getRequirements() override { return {drivetrain_}; }

private:
    enum class Phase { kTurn, kPause, kDrive, kDone };

    Command* active() {
        switch (phase_) {
            case Phase::kTurn:
                return &*turn_;
            case Phase::kPause:
                return &*pause_;
            case Phase::kDrive:
                return &*drive_;
            case Phase::kDone:
                return nullptr;
        }
        return nullptr;
    }

    void beginPauseOrDrive() {
        if (pauseMs_ > 0) {
            phase_ = Phase::kPause;
            pause_.emplace(static_cast<float>(pauseMs_) * millisecond);
            pause_->initialize();
            return;
        }
        beginDrive();
    }

    void beginDrive() {
        const auto pose = localization_->getPose();
        const double remaining = (targetX_ - pose.x()) * std::sin(headingRad_) +
                                 (targetY_ - pose.y()) * std::cos(headingRad_);
        phase_ = Phase::kDrive;
        driveStarted_ = true;
        drive_.emplace(drivetrain_, localization_, remaining, headingRad_, finish_, driveTimeoutMs_);
        drive_->initialize();
    }

    DrivetrainSubsystem* drivetrain_;
    LocalizationSubsystem* localization_;
    double targetX_;  // in
    double targetY_;  // in
    bool finish_;
    int turnTimeoutMs_;   // ms; <= 0 disables
    int driveTimeoutMs_;  // ms; <= 0 disables
    int pauseMs_;         // ms between turn and drive
    double headingRad_ = 0.0;
    Phase phase_ = Phase::kTurn;
    bool driveStarted_ = false;
    std::optional<Rotate> turn_;
    std::optional<WaitCommand> pause_;
    std::optional<DriveDistance> drive_;
};
