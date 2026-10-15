#pragma once

#include "autonomous/sharedCommands.h"
#include "command/includes.h"
#include "subsystems/subsystems.h"

class DriverMacros {
 public:
  static Command* intakeToClaw() {  
    return new ParallelCommandGroup({
      intakeFor(100, 500),
      new Sequence({
        clawSubsystem->clawClose(),
        liftFor(100, 1000),
        clawSubsystem->wristUp(),  
      }),
    });
  };

  static Command* clawToIntake() {  // open claw, wrist down
    return new Sequence({
        clawSubsystem->clawOpen(),
        clawSubsystem->wristDown(),
    });
  };

  static void bind(CommandController& pad) {
    pad.getTrigger(pros::E_CONTROLLER_DIGITAL_UP)->onTrue(intakeToClaw());
    pad.getTrigger(pros::E_CONTROLLER_DIGITAL_DOWN)->onTrue(clawToIntake());
  }

 private:
  static Command* timed(Command* command, double durationMs) {
    return command->withTimeout(static_cast<float>(durationMs) * millisecond);
  }

  static Command* liftFor(double pct, double durationMs) {
    return timed(
        new FunctionalCommand([] {}, [pct] { liftSubsystem->setPct(pct); },
                              [](bool) { liftSubsystem->stop(); },
                              [] { return false; }, {liftSubsystem}),
        durationMs);
  }

  static Command* intakeFor(double pct, double durationMs) {
    return timed(
        new FunctionalCommand([] {}, [pct] { intakeSubsystem->setPct(pct); },
                              [](bool) { intakeSubsystem->stop(); },
                              [] { return false; }, {intakeSubsystem}),
        durationMs);
  }
};
