#include "subsystems/lift.h"
#include "command/runCommand.h"

LiftSubsystem::LiftSubsystem(int port)
    : liftMotor_(port) {
    liftMotor_.set_gearing(pros::MotorGear::green);
    liftMotor_.set_encoder_units(pros::MotorUnits::degrees);
    liftMotor_.set_brake_mode(pros::MotorBrake::brake);
}

void LiftSubsystem::setPct(double pct) {
    this->setVoltages(pct * 12000);
}

void LiftSubsystem::setVoltages(int mv) {
    liftMotor_.move_voltage(mv);
}

void LiftSubsystem::stop() {
    this->setVoltages(0);
}

RunCommand* LiftSubsystem::lift(pros::Controller& controller) {
    auto* pad = &controller;
    return new RunCommand(
        [this, pad]() {
            if (!pad->is_connected()) {
                this->stop();
                return;
            }
            if (pad->get_digital(pros::E_CONTROLLER_DIGITAL_R1) == 1) {
                this->setVoltages(12000);
            } else if (pad->get_digital(pros::E_CONTROLLER_DIGITAL_R2) == 1) {
                this->setVoltages(-12000);
            } else {
                this->stop();
            }
        },
        {this});
}