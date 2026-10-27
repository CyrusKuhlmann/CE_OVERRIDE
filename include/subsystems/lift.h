#pragma once

#include "command/runCommand.h"

#include "pros/misc.hpp"
#include "pros/motor_group.hpp"

#include <cstdint>
#include <initializer_list>

class LiftSubsystem : public Subsystem {
public:
    LiftSubsystem(std::initializer_list<int8_t> ports);

    void periodic() override {}

    void setPct(double pct);
    void setVoltages(int mv);

    RunCommand* lift(pros::Controller& controller);
    void stop();

private:
    pros::MotorGroup liftMotors_;
};