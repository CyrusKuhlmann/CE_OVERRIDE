#pragma once

#include "autonomous/auton.h"
#include "autonomous/sharedCommands.h"
#include "command/sequence.h"

class Routines {
public:
    static constexpr auton::StartPose kTestStart{0.0, -48.0, 0.0};

    static Command* test() {
        return new Sequence({
            SharedCommands::drive(24.0, 0.0),
            SharedCommands::print(1, "Current Pose: {}, {}, {}", [] { return localizationSubsystem->getX(); },
                                  [] { return localizationSubsystem->getY(); },
                                  [] { return localizationSubsystem->getAngle(); }),
            SharedCommands::wait(1000),
            SharedCommands::turnTo(90.0),
            SharedCommands::print(2, "Current Pose: {}, {}, {}", [] { return localizationSubsystem->getX(); },
                                  [] { return localizationSubsystem->getY(); },
                                  [] { return localizationSubsystem->getAngle(); }),
            SharedCommands::wait(1000),
            SharedCommands::driveTo(0.0, 0.0, 1000, 1000, 1000),
            SharedCommands::print(3, "Current Pose: {}, {}, {}", [] { return localizationSubsystem->getX(); },
                                  [] { return localizationSubsystem->getY(); },
                                  [] { return localizationSubsystem->getAngle(); }),
            SharedCommands::stop(),
        });
    }
};
