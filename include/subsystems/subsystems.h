#pragma once

#include "command/commandScheduler.h"
#include "command/runCommand.h"
#include "config.h"
#include "subsystems/drivetrain.h"
#include "subsystems/localization.h"
#include "localization/odometry.h"

#include "pros/misc.hpp"

inline DrivetrainSubsystem* drivetrainSubsystem = nullptr;
inline LocalizationSubsystem* localizationSubsystem = nullptr;
inline pros::Controller primary(pros::E_CONTROLLER_MASTER);

inline void subsystemInit() {
    ILocalizer* localizer = new OdometryLocalizer();
    localizationSubsystem =
        new LocalizationSubsystem(CONFIG::IMU_PORT, CONFIG::FWD_ROTATION_PORT, CONFIG::LAT_ROTATION_PORT, *localizer);
    drivetrainSubsystem =
        new DrivetrainSubsystem({CONFIG::LEFT_DRIVE[0], CONFIG::LEFT_DRIVE[1], CONFIG::LEFT_DRIVE[2]},
                                {CONFIG::RIGHT_DRIVE[0], CONFIG::RIGHT_DRIVE[1], CONFIG::RIGHT_DRIVE[2]});

    CommandScheduler::registerSubsystem(localizationSubsystem, new RunCommand([] {}, {localizationSubsystem}));
    CommandScheduler::registerSubsystem(drivetrainSubsystem, drivetrainSubsystem->drive(primary));
}
