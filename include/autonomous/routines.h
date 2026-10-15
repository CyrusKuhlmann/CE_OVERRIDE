#pragma once

#include "autonomous/auton.h"
#include "autonomous/sharedCommands.h"
#include "command/sequence.h"

#include <cstdio>

class Routines {
public:
    static constexpr auton::StartPose kTestStart{-24.0, -24.0, 0.0};

    static Command* test() {
        return new ParallelCommandGroup({
            new Sequence({
                SharedCommands::turnTo(270),
                SharedCommands::turnTo(180),
                SharedCommands::turnTo(90),
                SharedCommands::turnTo(0),
                SharedCommands::turnTo(270),
                SharedCommands::turnTo(180),
                SharedCommands::turnTo(90),
                SharedCommands::turnTo(0),
                SharedCommands::turnTo(270),
                SharedCommands::turnTo(180),
                SharedCommands::turnTo(90),
                SharedCommands::turnTo(0),
            }),
            (new RunCommand(
                 [] {
                     const auto pose = localizationSubsystem->getPose();
                     char buf[48];
                     const double headingDeg = CONFIG::radToDeg(CONFIG::wrapPi(pose.z()));
                     std::snprintf(buf, sizeof(buf), "x: %.2f y: %.2f h: %.2f deg", pose.x(), pose.y(), headingDeg);
                     pros::lcd::set_text(0, buf);
                 },
                 {}))
                ->until([] { return !pros::competition::is_autonomous(); }),
        });
    }
};
