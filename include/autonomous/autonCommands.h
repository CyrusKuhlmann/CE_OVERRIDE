#pragma once

#include "autonomous/auton.h"
#include "autonomous/routines.h"
#include "command/instantCommand.h"

class AutonomousCommands {
public:
    static Command* getAuton() {
        switch (AUTON) {
        case CLOSE:
            return Routines::close();
        case RIGHT:
            return Routines::right();
        case SKILLS:
            return Routines::skills();
        case TEST:
            return Routines::test();
        case NONE:
        default:
            return new InstantCommand([] {}, {});
        }
    }

    static auton::StartPose getStartPose() {
        switch (AUTON) {
        case CLOSE:
            return Routines::kCloseStart;
        case RIGHT:
            return Routines::kRightStart;
        case SKILLS:
            return Routines::kSkillsStart;
        case TEST:
            return Routines::kTestStart;
        case NONE:
        default:
            return {};
        }
    }
};
