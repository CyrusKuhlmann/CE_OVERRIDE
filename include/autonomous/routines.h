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

            // driveTo (0, -47.425) moving backwards
            SharedCommands::driveTo(0, -47.275, 750, 2000, false, 10, 0, 0),

            // intake to claw
            SharedCommands::intakeToClaw(),

            SharedCommands::wait(300),

            // driveTo (12.425, -47.425) moving backwards
            SharedCommands::driveTo(12.425, -47.425, 2000, 3000, false, 7, 0, 0, 300),

            // turnTo 90 degrees
            SharedCommands::turnTo(-90, 500, 10, 0, 0),

            SharedCommands::liftFor(-70, 850),

            SharedCommands::wait(300),

            // relese claw
            SharedCommands::toggleClaw(),

            // first scored
        });
    }

    static Command* right() {
        return new Sequence({
            SharedCommands::toggleClaw(), // claw now closed

            SharedCommands::wait(100),

            SharedCommands::drive(-4.5, 90.0, 300, 100, 0),
            SharedCommands::drive(-3.5, 90.0, 300, 100, 0),
            SharedCommands::drive(3.5, 90.0, 300, 100, 0),
            SharedCommands::wait(300),
            SharedCommands::drive(-3.5, 90.0, 300, 100, 0),
            SharedCommands::drive(3.5, 90.0, 300, 100, 0),

            SharedCommands::wait(300),

            SharedCommands::drive(5.5, 90.0, 1200, 10, 0),

            SharedCommands::drive(-3.5, 90.0, 1500, 10, 0, 0),

            SharedCommands::wait(300),

            SharedCommands::drive(5.5, 90.0, 1200, 10, 0),

            // set global position to current x current theta, and -60.5 y
            SharedCommands::setGlobalPosition(60.5, localizationSubsystem->getY(), 90.0),

            // driveTo (47.425, 0) moving backwards
            SharedCommands::driveTo(47.425, 0, 500, 1500, false, 30, 0, 0),

            // intake to claw
            SharedCommands::intakeToClaw(),

            SharedCommands::wait(300),

            // driveTo (48, -12.425) moving backwards
            SharedCommands::driveTo(48, -12.425, 1200, 1750, false, 25, 0, 0, 300),

            // turnTo 90 degrees
            SharedCommands::turnTo(0, 500, 10, 0, 0),

            SharedCommands::liftFor(-70, 850),

            SharedCommands::wait(300),

            // relese claw
            SharedCommands::toggleClaw(),

            // first scored
        });
    }

    static Command* skills() {
        return new Sequence({
            SharedCommands::toggleClaw(), // claw now closed

            SharedCommands::wait(100),

            SharedCommands::drive(-4.5, 180.0, 300, 1, 0),
            SharedCommands::drive(-3.5, 180.0, 300, 1, 0),
            SharedCommands::drive(3.5, 180.0, 300, 1, 0),
            SharedCommands::wait(300),
            SharedCommands::drive(-3.5, 180.0, 300, 1, 0),
            SharedCommands::drive(3.5, 180.0, 300, 1, 0),

            SharedCommands::wait(300),

            SharedCommands::drive(5.5, 180.0, 1200, 1, 0),

            SharedCommands::drive(-3.5, 180.0, 1500, 1, 0, 0),

            SharedCommands::wait(300),

            SharedCommands::drive(5.5, 180.0, 1200, 1, 0),

            // set global position to current x current theta, and -60.5 y
            SharedCommands::setGlobalPosition(localizationSubsystem->getX(), -60.5, 180.0),

            // driveTo (0, -47.425) moving backwards
            SharedCommands::driveTo(0, -47.275, 750, 2000, false, 1, 0, 0),

            // intake to claw
            SharedCommands::intakeToClaw(),

            SharedCommands::wait(300),

            // driveTo (12.425, -47.425) moving backwards
            SharedCommands::driveTo(12.425, -47.425, 2000, 3000, false, 1, 0, 0, 300),

            // turnTo 90 degrees
            SharedCommands::turnTo(-90, 500, 1, 0, 0),

            SharedCommands::liftFor(-70, 850),

            SharedCommands::wait(300),

            // relese claw
            SharedCommands::toggleClaw(),

            // first scored

            SharedCommands::wait(600),

            SharedCommands::driveTo(0, -48, 1500, 2200, true, 0.50, 0, 0, 300),

            SharedCommands::driveTo(12.6, -34.6, 1500, 2800, false, 0.50, 0, 0, 300),

            SharedCommands::turnTo(225, 1000, 0.50, 0, 0),

            new ParallelCommandGroup({
                SharedCommands::drive(-5, 225.0, 1500, 0.3, 0, 0),

                new Sequence({
                    SharedCommands::wait(300),

                    // claw now closed
                    SharedCommands::toggleClaw(),
                    SharedCommands::wait(150),
                    SharedCommands::liftFor(100, 1500),
                }),
            }),

            SharedCommands::driveTo(22, -43.5, 1500, 2200, false, 0.50, 0, 0, 300),

            SharedCommands::driveRaw(0.2, 200),

            SharedCommands::liftFor(-70, 1200),

            // release claw
            SharedCommands::toggleClaw(),

            SharedCommands::wait(300),

            SharedCommands::driveRaw(0.5, 500),

            SharedCommands::driveTo(0, 0, 1500, 2200, true, 0.50, 0, 0, 300),

        });
    }

    static Command* test() { return new Sequence({}); }
};
