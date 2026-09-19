#pragma once

#include "autonomous/auton.h"
#include "command/includes.h"
#include "commands/driveDistance.h"
#include "commands/driveToPoint.h"
#include "commands/rotate.h"
#include "config.h"
#include "subsystems/subsystems.h"

class SharedCommands {
public:
    static void applyStartPose(const auton::StartPose& pose) {
        localizationSubsystem->setPose(pose.xIn, pose.yIn, CONFIG::degToRad(pose.headingDeg));
    }

    static Command* drive(double distanceIn, double headingDeg) {
        return new DriveDistance(drivetrainSubsystem, localizationSubsystem, distanceIn, CONFIG::degToRad(headingDeg));
    }

    static Command* turnTo(double headingDeg) {
        return new Rotate(drivetrainSubsystem, localizationSubsystem, CONFIG::degToRad(headingDeg));
    }

    static Command* driveTo(double xIn, double yIn) {
        return new DriveToPoint(drivetrainSubsystem, localizationSubsystem, xIn, yIn);
    }

    static Command* wait(double ms) { return new WaitCommand(static_cast<float>(ms) * millisecond); }

    static Command* stop() {
        return new InstantCommand([] { drivetrainSubsystem->stop(); }, {drivetrainSubsystem});
    }
};
