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

    // pct is [-1, 1] of 12 V, applied equally to both sides.
    static Command* driveRaw(double pct, double durationMs) {
        return timed(new FunctionalCommand([] {}, [pct] { drivetrainSubsystem->setPct(pct, pct); },
                                           [](bool) { drivetrainSubsystem->stop(); }, [] { return false; },
                                           {drivetrainSubsystem}),
                     durationMs);
    }

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

    static Command* setGlobalPosition(double xIn, double yIn, double headingDeg) {
        return localizationSubsystem->setPoseCommand(xIn, yIn, CONFIG::degToRad(headingDeg));
    }

    static Command* intakeToClaw() {
        return new Sequence({
            intakeFor(100, 200),
            clawSubsystem->clawClose(),
            liftFor(100, 900),
            clawSubsystem->wristUp(),
        });
    }

    static Command* intakeToClawPinOnly() {
        return new Sequence({
            intakeFor(100, 200),
            intakeFor(-100, 200),
            clawSubsystem->clawClose(),
            liftFor(100, 900),
            clawSubsystem->wristUp(),
        });
    }

    static Command* toggleClaw() { return clawSubsystem->clawToggle(); }

    // pct is percent of 12 V. Positive raises, negative lowers.
    static Command* liftFor(double pct, double durationMs) {
        return timed(new FunctionalCommand([] {}, [pct] { liftSubsystem->setPct(pct / 100.0); },
                                           [](bool) { liftSubsystem->stop(); }, [] { return false; }, {liftSubsystem}),
                     durationMs);
    }

    static Command* clawToIntake() {
        return new Sequence({
            clawSubsystem->clawOpen(),
            clawSubsystem->wristDown(),
        });
    }

private:
    static Command* timed(Command* command, double durationMs) {
        return command->withTimeout(static_cast<float>(durationMs) * millisecond);
    }

    static Command* intakeFor(double pct, double durationMs) {
        return timed(new FunctionalCommand([] {}, [pct] { intakeSubsystem->setPct(pct); },
                                           [](bool) { intakeSubsystem->stop(); }, [] { return false; },
                                           {intakeSubsystem}),
                     durationMs);
    }
};
