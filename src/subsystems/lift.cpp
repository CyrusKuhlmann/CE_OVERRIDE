#include "subsystems/lift.h"
#include "command/runCommand.h"

LiftSubsystem::LiftSubsystem(std::initializer_list<int8_t> ports)
    : liftMotors_(ports) {
    liftMotors_.set_gearing_all(pros::MotorGears::blue);
    liftMotors_.set_encoder_units_all(pros::MotorUnits::degrees);
    liftMotors_.set_brake_mode_all(pros::MotorBrake::brake);
}

void LiftSubsystem::setPct(double pct) {
    this->setVoltages(pct * 12000);
}

void LiftSubsystem::setVoltages(int mv) {
    liftMotors_.move_voltage(mv);
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