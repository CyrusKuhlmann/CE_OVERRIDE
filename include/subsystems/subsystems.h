#pragma once

#include "command/commandScheduler.h"
#include "command/runCommand.h"
#include "config.h"
#include "subsystems/drivetrain.h"

#include "pros/misc.hpp"

inline DrivetrainSubsystem* drivetrainSubsystem = nullptr;
inline pros::Controller primary(pros::E_CONTROLLER_MASTER);

inline void subsystemInit() {
    drivetrainSubsystem =
        new DrivetrainSubsystem({CONFIG::LEFT_DRIVE[0], CONFIG::LEFT_DRIVE[1], CONFIG::LEFT_DRIVE[2]},
                                {CONFIG::RIGHT_DRIVE[0], CONFIG::RIGHT_DRIVE[1], CONFIG::RIGHT_DRIVE[2]},
                                CONFIG::IMU_PORT, CONFIG::FWD_ROTATION_PORT, CONFIG::LAT_ROTATION_PORT);

    CommandScheduler::registerSubsystem(drivetrainSubsystem, drivetrainSubsystem->tankDrive(primary));
}
