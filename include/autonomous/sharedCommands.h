#pragma once

#include "autonomous/auton.h"
#include "command/includes.h"
#include "commands/driveDistance.h"
#include "commands/driveToPoint.h"
#include "commands/print.h"
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

    static Command* drive(double distanceIn, double headingDeg, int timeoutMs = 0) {
        return new DriveDistance(drivetrainSubsystem, localizationSubsystem, distanceIn, CONFIG::degToRad(headingDeg),
                                 true, timeoutMs);
    }

    static Command* turnTo(double headingDeg, int timeoutMs = 0) {
        return new Rotate(drivetrainSubsystem, localizationSubsystem, CONFIG::degToRad(headingDeg), true, timeoutMs);
    }

    static Command* driveTo(double xIn, double yIn) {
        return new DriveToPoint(drivetrainSubsystem, localizationSubsystem, xIn, yIn);
    }

    static Command* wait(double ms) { return new WaitCommand(static_cast<float>(ms) * millisecond); }

    static Command* stop() {
        return new InstantCommand([] { drivetrainSubsystem->stop(); }, {drivetrainSubsystem});
    }

    template <typename... Args>
    static Command* print(int line, std::string format, Args&&... args) {
        return new Print(line, std::move(format), std::forward<Args>(args)...);
    }

    static Command* print(int line, std::function<std::string()> message) {
        return new Print(line, std::move(message));
    }
};
