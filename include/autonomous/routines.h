#pragma once

#include "autonomous/auton.h"
#include "autonomous/sharedCommands.h"
#include "command/sequence.h"

class Routines {
public:
    static constexpr auton::StartPose kTestStart{0, -56.3, 180.0};

    static Command* test() {
        return new Sequence({
            SharedCommands::drive(3.0, 180.0, 300, 100, 50),
            SharedCommands::drive(-3.0, 180.0, 300, 100, 50),
            SharedCommands::drive(1.0, 180.0, 300, 100, 50),
            SharedCommands::drive(-1.0, 180.0, 300, 100, 50),

            SharedCommands::wait(1000),

            SharedCommands::drive(6.5, 180.0, 1000, 80, 50),
            SharedCommands::drive(-3.5, 180.0, 1000, 100, 50),

            SharedCommands::wait(300),

            SharedCommands::drive(6.5, 180.0, 1000, 80, 50),
        });
    }
};
