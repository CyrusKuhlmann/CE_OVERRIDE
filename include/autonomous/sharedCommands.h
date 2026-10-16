#pragma once

#include "autonomous/auton.h"
#include "command/includes.h"
#include "commands/driveDistance.h"
#include "commands/driveToPoint.h"
#include "commands/rotate.h"
#include "config.h"
#include "subsystems/subsystems.h"

#include <functional>
#include <string>
#include <utility>

class SharedCommands {
public:
    static void applyStartPose(const auton::StartPose& pose) {
        localizationSubsystem->setPose(pose.xIn, pose.yIn, CONFIG::degToRad(pose.headingDeg));
    }

    // Motions mirror LemLib's calls: target, then timeout(s) (ms; <= 0 disables), then speed options.
    // Speeds are pct of 12 V. A nonzero minSpeed enables motion chaining: the motion hands off at speed
    // once within earlyExitRange of the target (in for drives, deg for turns).

    static Command* drive(double distanceIn, double headingDeg, int timeoutMs = 0,
                          double maxSpeed = CONFIG::DRIVE_MAX_SPEED, double minSpeed = 0.0,
                          double earlyExitRangeIn = 0.0) {
        return new DriveDistance(drivetrainSubsystem, localizationSubsystem, distanceIn, CONFIG::degToRad(headingDeg),
                                 timeoutMs, maxSpeed, minSpeed, earlyExitRangeIn);
    }

    static Command* turnTo(double headingDeg, int timeoutMs = 0, double maxSpeed = CONFIG::TURN_MAX_SPEED,
                           double minSpeed = 0.0, double earlyExitRangeDeg = 0.0) {
        return new Rotate(drivetrainSubsystem, localizationSubsystem, CONFIG::degToRad(headingDeg), timeoutMs, maxSpeed,
                          minSpeed, CONFIG::degToRad(earlyExitRangeDeg));
    }

    static Command* driveTo(double xIn, double yIn, int turnTimeoutMs = 0, int driveTimeoutMs = 0, bool forwards = true,
                            double maxSpeed = CONFIG::DRIVE_MAX_SPEED, double minSpeed = 0.0,
                            double earlyExitRangeIn = 0.0, int pauseMs = 75) {
        return new DriveToPoint(drivetrainSubsystem, localizationSubsystem, xIn, yIn, turnTimeoutMs, driveTimeoutMs,
                                forwards, maxSpeed, minSpeed, earlyExitRangeIn, pauseMs);
    }

    static Command* wait(double ms) { return new WaitCommand(static_cast<float>(ms) * millisecond); }

    static Command* stop() {
        return new InstantCommand([] { drivetrainSubsystem->stop(); }, {drivetrainSubsystem});
    }
};
