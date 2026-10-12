#pragma once

#include "command/commandController.h"
#include "command/commandScheduler.h"
#include "command/runCommand.h"
#include "config.h"
#include "subsystems/claw.h"
#include "subsystems/drivetrain.h"
#include "subsystems/intake.h"
#include "subsystems/lift.h"
#include "subsystems/localization.h"
#include "localization/odometry.h"

#include "pros/misc.hpp"

inline DrivetrainSubsystem* drivetrainSubsystem = nullptr;
inline IntakeSubsystem* intakeSubsystem = nullptr;
inline LiftSubsystem* liftSubsystem = nullptr;
inline ClawSubsystem* clawSubsystem = nullptr;
inline LocalizationSubsystem* localizationSubsystem = nullptr;
inline CommandController primary(pros::E_CONTROLLER_MASTER);

inline void subsystemInit() {
    ILocalizer* localizer = new OdometryLocalizer();
    localizationSubsystem =
        new LocalizationSubsystem(CONFIG::IMU_PORT, CONFIG::FWD_ROTATION_PORT, CONFIG::LAT_ROTATION_PORT, *localizer);
    drivetrainSubsystem =
        new DrivetrainSubsystem({CONFIG::LEFT_DRIVE[0], CONFIG::LEFT_DRIVE[1], CONFIG::LEFT_DRIVE[2]},
                                {CONFIG::RIGHT_DRIVE[0], CONFIG::RIGHT_DRIVE[1], CONFIG::RIGHT_DRIVE[2]});
    intakeSubsystem = new IntakeSubsystem(CONFIG::INTAKE_PORT);
    liftSubsystem = new LiftSubsystem(CONFIG::LIFT_PORT);
    clawSubsystem = new ClawSubsystem(CONFIG::WRIST_PORT, CONFIG::CLAW_PORT);

    CommandScheduler::registerSubsystem(localizationSubsystem, new RunCommand([] {}, {localizationSubsystem}));
    CommandScheduler::registerSubsystem(drivetrainSubsystem, drivetrainSubsystem->drive(primary));
    CommandScheduler::registerSubsystem(intakeSubsystem, intakeSubsystem->intake(primary));
    CommandScheduler::registerSubsystem(liftSubsystem, liftSubsystem->lift(primary));
    CommandScheduler::registerSubsystem(clawSubsystem, clawSubsystem->toggle(primary));
}
