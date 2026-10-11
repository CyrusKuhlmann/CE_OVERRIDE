#pragma once

#include "command/runCommand.h"
#include "api.h"

class IntakeSubsystem : public Subsystem {
public:
    IntakeSubsystem(int port);

    void periodic() override {}

    void setPct(double pct);
    void setVoltages(int mv);

    RunCommand* intake(pros::Controller& controller);

    void stop();
private:
    pros::Motor intakeMotor_;
};