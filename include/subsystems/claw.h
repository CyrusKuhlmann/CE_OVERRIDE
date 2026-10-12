#pragma once

#include "command/instantCommand.h"
#include "command/runCommand.h"
#include "api.h"

class ClawSubsystem : public Subsystem {
public:
    ClawSubsystem(char wristPort, char clawPort);

    void periodic() override {}

    void setPct(double pct) {};

    void setVoltages(int mv) {};

    InstantCommand* wristUp();
    InstantCommand* wristDown();
    InstantCommand* clawOpen();
    InstantCommand* clawClose();

    RunCommand* toggle(pros::Controller& controller);

    void stop();
private:
    pros::adi::Pneumatics wrist_;
    pros::adi::Pneumatics claw_;
};
