#pragma once

#include "autonomous/auton.h"
#include "autonomous/sharedCommands.h"
#include "command/sequence.h"

class Routines {
public:
    static constexpr auton::StartPose kCloseStart{0, -60.5, 180.0};
    static constexpr auton::StartPose kRightStart{60.5, 0, 90.0};
    static constexpr auton::StartPose kSkillsStart{0, -60.5, 180.0};
    static constexpr auton::StartPose kTestStart{0, -60.5, 180.0};

    static Command* close() {
        return new Sequence({
            SharedCommands::toggleClaw(), // claw now closed

            SharedCommands::wait(100),

            SharedCommands::drive(-4.5, 180.0, 300, 100, 0),
            SharedCommands::drive(-3.5, 180.0, 300, 100, 0),
            SharedCommands::drive(3.5, 180.0, 300, 100, 0),
            SharedCommands::wait(300),
            SharedCommands::drive(-3.5, 180.0, 300, 100, 0),
            SharedCommands::drive(3.5, 180.0, 300, 100, 0),

            SharedCommands::wait(300),

            SharedCommands::drive(5.5, 180.0, 1200, 10, 0),

            SharedCommands::drive(-3.5, 180.0, 1500, 10, 0, 0),

            SharedCommands::wait(300),

            SharedCommands::drive(5.5, 180.0, 1200, 10, 0),

            // set global position to current x current theta, and -60.5 y
            SharedCommands::setGlobalPosition(localizationSubsystem->getX(), -60.5, 180.0),

            // intake to claw
            SharedCommands::intakeToClaw(),

            SharedCommands::wait(300),

            // driveTo (0, -47.425) moving backwards
            SharedCommands::driveTo(0, -47.425, 500, 1500, false, 30, 0, 0),

            // driveTo (12.425, -47.425) moving backwards
            SharedCommands::driveTo(12.425, -47.425, 1200, 1750, false, 25, 0, 0, 300),

            // turnTo 90 degrees
            SharedCommands::turnTo(-90, 500, 10, 0, 0),

            // relese claw
            SharedCommands::toggleClaw(),





            // SharedCommands::drive(6.5, 180.0, 300, 100, 0),
            // SharedCommands::drive(-3.5, 180.0, 1000, 100, 0),

            // SharedCommands::wait(1000),

            // SharedCommands::drive(6.5, 180.0, 1000, 75, 0),
            // SharedCommands::drive(-3.5, 180.0, 1000, 100, 0),

            // SharedCommands::wait(300),

            // SharedCommands::drive(6.5, 180.0, 1000, 80, 0),

            // SharedCommands::drive(-14, 180.0, 1500, 60, 0, 0),

            // SharedCommands::intakeToClaw(),

            // SharedCommands::turnTo(270),

            // SharedCommands::drive(-15.25, 270.0, 2000, 60, 0),

            // SharedCommands::drive(1.5, 270.0, 2000, 40, 0),

            // SharedCommands::liftFor(-70, 760),

            // SharedCommands::wait(200),

            // SharedCommands::toggleClaw(),

            // first scored
        });
    }

    static Command* right() {
        return new Sequence({
            SharedCommands::toggleClaw(), // claw now closed

            SharedCommands::wait(100),

            SharedCommands::drive(6.5, 90.0, 300, 100, 0),
            SharedCommands::drive(-3.5, 90.0, 1000, 100, 0),

            SharedCommands::wait(1000),

            SharedCommands::drive(6.5, 90.0, 1000, 75, 0),
            SharedCommands::drive(-3.5, 90.0, 1000, 100, 0),

            SharedCommands::wait(300),

            SharedCommands::drive(6.5, 90.0, 1000, 80, 0),

            SharedCommands::drive(-14, 90.0, 1500, 60, 0, 0),

            SharedCommands::intakeToClaw(),

            SharedCommands::turnTo(0),

            SharedCommands::drive(-15.25, 0.0, 2000, 60, 0),

            SharedCommands::drive(1.5, 0.0, 2000, 40, 0),

            SharedCommands::liftFor(-70, 760),

            SharedCommands::wait(200),

            SharedCommands::toggleClaw(),

            // first scored
        });
    }

    static Command* skills() {
        return new Sequence({
            SharedCommands::toggleClaw(), // claw now closed

            SharedCommands::wait(100),

            SharedCommands::drive(6.5, 180.0, 300, 100, 0),
            SharedCommands::drive(-3.5, 180.0, 1000, 100, 0),

            SharedCommands::wait(1000),

            SharedCommands::drive(6.5, 180.0, 1000, 75, 0),
            SharedCommands::drive(-3.5, 180.0, 1000, 100, 0),

            SharedCommands::wait(300),

            SharedCommands::drive(6.5, 180.0, 1000, 80, 0),

            SharedCommands::drive(-14, 180.0, 1500, 60, 0, 0),

            SharedCommands::intakeToClaw(),

            SharedCommands::turnTo(270),

            SharedCommands::drive(-15.25, 270.0, 2000, 60, 0),

            SharedCommands::drive(1.5, 270.0, 2000, 40, 0),

            SharedCommands::liftFor(-70, 760),

            SharedCommands::wait(200),

            SharedCommands::toggleClaw(),

            // first scored
            SharedCommands::setGlobalPosition(48, -14, localizationSubsystem->getAngle()),

            SharedCommands::wait(200),

            SharedCommands::drive(11, 270.0, 1000, 50, 0),

            SharedCommands::turnTo(-135),

            SharedCommands::drive(-20, 225, 2500, 1, 0),

            SharedCommands::toggleClaw(),

            SharedCommands::liftFor(70, 850),

            SharedCommands::drive(-10, 225, 750, 30, 0),

            SharedCommands::turnTo(0),

            SharedCommands::drive(-15, 0, 1500, 50, 0),

            SharedCommands::liftFor(-70, 850),

            SharedCommands::toggleClaw(),
        });
    }

    static Command* test() { return new Sequence({}); }
};
