#include "subsystems/claw.h"

#include "command/instantCommand.h"
#include "command/runCommand.h"

ClawSubsystem::ClawSubsystem(char wristPort, char clawPort) : wrist_(wristPort, false), claw_(clawPort, false) {
}

InstantCommand* ClawSubsystem::wristUp() {
    return new InstantCommand([this] { wrist_.set_value(true); }, {this});
}

InstantCommand* ClawSubsystem::wristDown() {
    return new InstantCommand([this] { wrist_.set_value(false); }, {this});
}

InstantCommand* ClawSubsystem::clawOpen() {
    return new InstantCommand([this] { claw_.retract(); }, {this});
}

InstantCommand* ClawSubsystem::clawClose() {
    return new InstantCommand([this] { claw_.extend(); }, {this});
}

InstantCommand* ClawSubsystem::clawToggle() {
    return new InstantCommand([this] { claw_.toggle(); }, {this});
}

RunCommand* ClawSubsystem::toggle(pros::Controller& controller) {
    auto* pad = &controller;
    return new RunCommand(
        [this, pad, pressed = false]() mutable {
            if (!pad->is_connected()) {
                return;
            }
            const bool now = pad->get_digital(pros::E_CONTROLLER_DIGITAL_A) == 1;
            if (now && !pressed) {
                claw_.toggle();
            }
            pressed = now;
        },
        {this});
}

void ClawSubsystem::stop() {
    wrist_.set_value(false);
    claw_.retract();
}