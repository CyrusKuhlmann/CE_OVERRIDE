#pragma once

#include "autonomous/auton.h"
#include "autonomous/routines.h"
#include "command/instantCommand.h"

class AutonomousCommands {
public:
    static Command* getAuton() {
        switch (AUTON) {
        case TEST:
            return Routines::test();
        case NONE:
        default:
            return new InstantCommand([] {}, {});
        }
    }

    static auton::StartPose getStartPose() {
        switch (AUTON) {
        case TEST:
            return Routines::kTestStart;
        case NONE:
        default:
            return {};
        }
    }
};
