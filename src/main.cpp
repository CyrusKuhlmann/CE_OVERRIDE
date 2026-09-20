/**
 * CE_OVERRIDE PROS 4 project.
 *
 * Command-based: one drivetrain subsystem, a 10 ms CommandScheduler task, and
 * no extra tasks or mutexes. Driving in opcontrol is the drivetrain default command.
 */
#include "main.h"

#include "autonomous/autonCommands.h"
#include "command/commandScheduler.h"
#include "subsystems/subsystems.h"

#include <cstdint>

static Command* autonCommand = nullptr;

static void schedulerLoop() {
    while (true) {
        std::uint32_t start = pros::millis();
        CommandScheduler::run();
        pros::Task::delay_until(&start, 10);
    }
}

void initialize() {
    pros::lcd::initialize();

    subsystemInit();

    pros::Task scheduler(schedulerLoop, "Command Scheduler");

    autonCommand = AutonomousCommands::getAuton();
    SharedCommands::applyStartPose(AutonomousCommands::getStartPose());
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
    if (autonCommand != nullptr) {
        CommandScheduler::schedule(autonCommand);
    }
}

void opcontrol() {
    if (autonCommand != nullptr) {
        autonCommand->cancel();
    }
    while (true) {
        pros::delay(20);
    }
}
