#include "subsystems/claw.h"

#include "command/runCommand.h"

ClawSubsystem::ClawSubsystem(char wristPort, char clawPort) : wrist_(wristPort, false), claw_(clawPort, true) {
}

RunCommand* ClawSubsystem::wristUp() {
    return new RunCommand(
        [this]() {
            wrist_.set_value(true);
        },
        {this});
}

RunCommand* ClawSubsystem::wristDown() {
    return new RunCommand(
        [this]() {
            wrist_.set_value(false);
        },
        {this});
}

RunCommand* ClawSubsystem::clawOpen() {
    return new RunCommand(
        [this]() {
            claw_.set_value(false);
        },
        {this});
}

RunCommand* ClawSubsystem::clawClose() {
    return new RunCommand(
        [this]() {
            claw_.set_value(true);
        },
        {this});
}

void ClawSubsystem::stop() {
    wrist_.set_value(false);
    claw_.set_value(false);
}