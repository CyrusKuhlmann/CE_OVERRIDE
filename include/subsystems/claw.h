#pragma once

#include "command/runCommand.h"
#include "api.h"

class ClawSubsystem : public Subsystem {
public:
    ClawSubsystem(char wristPort, char clawPort);

    void periodic() override {}

    void setPct(double pct) {};

    void setVoltages(int mv) {};

    RunCommand* wristUp();
    RunCommand* wristDown();
    RunCommand* clawOpen();
    RunCommand* clawClose();

    void stop();
private:
    pros::adi::Pneumatics wrist_;
    pros::adi::Pneumatics claw_;
};
