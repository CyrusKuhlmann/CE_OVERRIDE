#include "subsystems/intake.h"
#include "command/runCommand.h"

IntakeSubsystem::IntakeSubsystem(int port)
    : intakeMotor_(port) {
    intakeMotor_.set_gearing(pros::MotorGear::blue);
    intakeMotor_.set_encoder_units(pros::MotorUnits::degrees);
    intakeMotor_.set_brake_mode(pros::MotorBrake::brake);
}

void IntakeSubsystem::setPct(double pct) {
    this->setVoltages(pct * 12000);
}

void IntakeSubsystem::setVoltages(int mv) {
    intakeMotor_.move_voltage(mv);
}

void IntakeSubsystem::stop() {
    this->setVoltages(0);
}

RunCommand* IntakeSubsystem::intake(pros::Controller& controller) {
    auto* pad = &controller;
    return new RunCommand(
        [this, pad]() {
            if (!pad->is_connected()) {
                this->stop();
                return;
            }
            if (pad->get_digital(pros::E_CONTROLLER_DIGITAL_L1) == 1) {
                this->setVoltages(12000);
            } else if (pad->get_digital(pros::E_CONTROLLER_DIGITAL_L2) == 1) {
                this->setVoltages(-12000);
            } else {
                this->stop();
            }
        },
        {this});
}