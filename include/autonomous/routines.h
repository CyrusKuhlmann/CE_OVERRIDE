#pragma once

#include "autonomous/auton.h"
#include "autonomous/sharedCommands.h"
#include "command/sequence.h"

class Routines {
public:
    static constexpr auton::StartPose kTestStart{0.0, -48.0, 0.0};

    static Command* test() {
        return new Sequence({
            SharedCommands::driveTo(-24.0, 0.0),
            SharedCommands::print(1, "Current Pose: {}, {}, {}", [] { return std::round(localizationSubsystem->getX()*10)/10; },
                                  [] { return std::round(localizationSubsystem->getY()*10)/10; },
                                  [] { return std::round(localizationSubsystem->getAngle()*10)/10; }),
            SharedCommands::wait(1000),
            SharedCommands::driveTo(0.0, 24.0),
            SharedCommands::print(2, "Current Pose: {}, {}, {}", [] { return std::round(localizationSubsystem->getX()*10)/10; },
                                  [] { return std::round(localizationSubsystem->getY()*10)/10; },
                                  [] { return std::round(localizationSubsystem->getAngle()*10)/10; }),
            SharedCommands::wait(1000),
            SharedCommands::driveTo(24, 0.0),
            SharedCommands::print(3, "Current Pose: {}, {}, {}", [] { return std::round(localizationSubsystem->getX()*10)/10; },
                                  [] { return std::round(localizationSubsystem->getY()*10)/10; },
                                  [] { return std::round(localizationSubsystem->getAngle()*10)/10; }),
            SharedCommands::driveTo(0.0, -48.0),
            SharedCommands::print(4, "Current Pose: {}, {}, {}", [] { return std::round(localizationSubsystem->getX()*10)/10; },
                                  [] { return std::round(localizationSubsystem->getY()*10)/10; },
                                  [] { return std::round(localizationSubsystem->getAngle()*10)/10; }),
            SharedCommands::turnTo(0.0),
            SharedCommands::print(5, "Current Pose: {}, {}, {}", [] { return std::round(localizationSubsystem->getX()*10)/10; },
                                  [] { return std::round(localizationSubsystem->getY()*10)/10; },
                                  [] { return std::round(localizationSubsystem->getAngle()*10)/10; }),
            SharedCommands::stop(),
        });
    }
};
