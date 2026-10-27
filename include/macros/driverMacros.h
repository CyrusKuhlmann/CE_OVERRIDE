#pragma once

#include "autonomous/sharedCommands.h"
#include "command/includes.h"
#include "subsystems/subsystems.h"

class DriverMacros {
 public:
  static Command* intakeToClaw() { return SharedCommands::intakeToClaw(); }

  static Command* intakeToClawPinOnly() { return SharedCommands::intakeToClawPinOnly(); }

  static Command* clawToIntake() { return SharedCommands::clawToIntake(); }

  static void bind(CommandController& pad) {
    pad.getTrigger(pros::E_CONTROLLER_DIGITAL_UP)->onTrue(intakeToClaw());
    pad.getTrigger(pros::E_CONTROLLER_DIGITAL_DOWN)->onTrue(clawToIntake());
    pad.getTrigger(pros::E_CONTROLLER_DIGITAL_LEFT)->onTrue(intakeToClawPinOnly());
  }
};
