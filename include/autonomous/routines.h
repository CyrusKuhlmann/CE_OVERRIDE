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
            SharedCommands::wait(1000),
            SharedCommands::turnTo(90.0),
            SharedCommands::wait(1000),
            SharedCommands::driveTo(0.0, 0.0),
            SharedCommands::stop(),
        });
    }
};
