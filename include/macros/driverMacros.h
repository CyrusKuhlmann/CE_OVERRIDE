#pragma once

#include "autonomous/sharedCommands.h"
#include "command/includes.h"
#include "subsystems/subsystems.h"

class DriverMacros {
 public:
  static Command* intakeToClaw() { // close claw, raise lift for 1 second on 50% power, finally wrist up
    return new Sequence({
      new FunctionalCommand([] {}, []{}, [](bool) { clawSubsystem->clawOpen(); }, [] { return false; }, {clawSubsystem}),
      liftFor(50, 1000),
      new FunctionalCommand([] {}, []{}, [](bool) { clawSubsystem->wristUp(); }, [] { return false; }, {clawSubsystem}),
    });
   };

  static void bind(CommandController& pad) {
    pad.getTrigger(pros::E_CONTROLLER_DIGITAL_A)->onTrue(intakeToClaw());
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
};
