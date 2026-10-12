#pragma once

#include "command/runCommand.h"

#include "pros/misc.hpp"
#include "pros/motors.hpp"

class LiftSubsystem : public Subsystem {
public:
    LiftSubsystem(int port);

    void periodic() override {}

    void setPct(double pct);
    void setVoltages(int mv);

    RunCommand* lift(pros::Controller& controller);
    void stop();

private:
    pros::Motor liftMotor_;
};