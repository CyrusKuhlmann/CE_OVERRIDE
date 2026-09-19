#pragma once

#include "command/runCommand.h"

#include "pros/misc.hpp"
#include "pros/motor_group.hpp"

#include <cstdint>
#include <initializer_list>

class DrivetrainSubsystem : public Subsystem {
public:
    DrivetrainSubsystem(std::initializer_list<int8_t> leftPorts, std::initializer_list<int8_t> rightPorts);

    void periodic() override {}

    void setPct(double left, double right);  // [-1, 1]
    void setVoltages(int leftMv, int rightMv);  // mV
    void stop();

    RunCommand* tankDrive(pros::Controller& controller);
    RunCommand* doubleArcadeDrive(pros::Controller& controller);
    RunCommand* singleArcadeDrive(pros::Controller& controller);
    RunCommand* drive(pros::Controller& controller);

private:
    static double mappedDriveAxis(pros::Controller& controller, pros::controller_analog_e_t axis);
    static double applyDriveNonlinearity(double x);

    pros::MotorGroup leftMotors_;
    pros::MotorGroup rightMotors_;
};
