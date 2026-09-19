#include "subsystems/drivetrain.h"

#include "command/runCommand.h"
#include "config.h"

#include <algorithm>
#include <cmath>

DrivetrainSubsystem::DrivetrainSubsystem(std::initializer_list<int8_t> leftPorts,
                                         std::initializer_list<int8_t> rightPorts)
    : leftMotors_(leftPorts), rightMotors_(rightPorts) {
    leftMotors_.set_gearing_all(pros::MotorGears::blue);
    rightMotors_.set_gearing_all(pros::MotorGears::blue);
    leftMotors_.set_encoder_units_all(pros::MotorUnits::degrees);
    rightMotors_.set_encoder_units_all(pros::MotorUnits::degrees);
    leftMotors_.set_brake_mode_all(pros::MotorBrake::brake);
    rightMotors_.set_brake_mode_all(pros::MotorBrake::brake);
}

void DrivetrainSubsystem::setPct(double left, double right) {
    left = std::clamp(left, -1.0, 1.0);
    right = std::clamp(right, -1.0, 1.0);
    setVoltages(static_cast<int>(left * 12000.0), static_cast<int>(right * 12000.0));
}

void DrivetrainSubsystem::setVoltages(int leftMv, int rightMv) {
    leftMotors_.move_voltage(leftMv);
    rightMotors_.move_voltage(rightMv);
}

void DrivetrainSubsystem::stop() { setPct(0.0, 0.0); }

inline double DrivetrainSubsystem::applyDriveNonlinearity(double x) {
    return CONFIG::DRIVE_NONLINEARITY * x * x * x + (1.0 - CONFIG::DRIVE_NONLINEARITY) * x;
}

double DrivetrainSubsystem::mappedDriveAxis(pros::Controller& controller, pros::controller_analog_e_t axis) {
    double x = std::clamp(static_cast<double>(controller.get_analog(axis)), -127.0, 127.0) / 127.0;
    if (std::fabs(x) < CONFIG::DRIVE_DEADBAND) return 0.0;
    return applyDriveNonlinearity(x);
}

RunCommand* DrivetrainSubsystem::tankDrive(pros::Controller& controller) {
    auto* pad = &controller;
    return new RunCommand(
        [this, pad]() {
            if (!pad->is_connected()) {
                this->stop();
                return;
            }
            const double left = mappedDriveAxis(*pad, pros::E_CONTROLLER_ANALOG_LEFT_Y);
            const double right = mappedDriveAxis(*pad, pros::E_CONTROLLER_ANALOG_RIGHT_Y);
            this->setPct(left, right);
        },
        {this});
}

RunCommand* DrivetrainSubsystem::doubleArcadeDrive(pros::Controller& controller) {
    auto* pad = &controller;
    return new RunCommand(
        [this, pad]() {
            if (!pad->is_connected()) {
                this->stop();
                return;
            }
            const double throttle = mappedDriveAxis(*pad, pros::E_CONTROLLER_ANALOG_RIGHT_Y);
            const double turn = mappedDriveAxis(*pad, pros::E_CONTROLLER_ANALOG_LEFT_X);

            double left = throttle + turn;
            double right = throttle - turn;

            const double max = std::max(1.0, std::max(std::fabs(left), std::fabs(right)));

            left /= max;
            right /= max;

            this->setPct(left, right);
        },
        {this});
}

RunCommand* DrivetrainSubsystem::singleArcadeDrive(pros::Controller& controller) {
    auto* pad = &controller;
    return new RunCommand(
        [this, pad]() {
            if (!pad->is_connected()) {
                this->stop();
                return;
            }
            const double throttle = mappedDriveAxis(*pad, pros::E_CONTROLLER_ANALOG_RIGHT_Y);
            const double turn = mappedDriveAxis(*pad, pros::E_CONTROLLER_ANALOG_RIGHT_X);

            double left = throttle + turn;
            double right = throttle - turn;

            const double max = std::max(1.0, std::max(std::fabs(left), std::fabs(right)));

            left /= max;
            right /= max;

            this->setPct(left, right);
        },
        {this});
}

RunCommand* DrivetrainSubsystem::drive(pros::Controller& controller) {
    switch (CONFIG::DEFAULT_DRIVE_TYPE) {
        case CONFIG::DOUBLE_ARCADE:
            return doubleArcadeDrive(controller);
        case CONFIG::SINGLE_ARCADE:
            return singleArcadeDrive(controller);
        case CONFIG::TANK:
            return tankDrive(controller);
    }
}
