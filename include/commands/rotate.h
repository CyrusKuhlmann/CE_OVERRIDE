#pragma once

#include "command/command.h"
#include "config.h"
#include "controllers/pidController.h"
#include "subsystems/drivetrain.h"

#include "pros/rtos.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

// Point-turn to a field heading. Factories pass degrees; this command stores radians.
class Rotate : public Command {
public:
    Rotate(DrivetrainSubsystem* drivetrain, double headingRad, bool finish = true)
        : drivetrain_(drivetrain), targetRad_(headingRad), finish_(finish) {}

    void initialize() override {
        pid_ = PID(CONFIG::TURN_PID);
        pid_.enableContinuousInput(-CONFIG::PI, CONFIG::PI);
        pid_.setOutputLimit(-CONFIG::TURN_OUTPUT_LIMIT, CONFIG::TURN_OUTPUT_LIMIT);
        pid_.setSetpoint(targetRad_);
        pid_.reset();
        lastMs_ = pros::millis();
        settleStartMs_ = 0;
    }

    void execute() override {
        const std::uint32_t now = pros::millis();
        const double dt = std::max((now - lastMs_) * 0.001, 0.001);
        lastMs_ = now;
        const double out = pid_.calculate(drivetrain_->getAngle(), dt);
        drivetrain_->setPct(out, -out);
    }

    void end(bool /*interrupted*/) override { drivetrain_->stop(); }

    bool isFinished() override {
        if (!finish_) return false;
        const double err = std::fabs(CONFIG::wrapPi(targetRad_ - drivetrain_->getAngle()));
        if (err > CONFIG::ANGLE_FINISH_RAD) {
            settleStartMs_ = 0;
            return false;
        }
        if (settleStartMs_ == 0) settleStartMs_ = pros::millis();
        return (pros::millis() - settleStartMs_) >= static_cast<std::uint32_t>(CONFIG::SETTLE_MS);
    }

    std::vector<Subsystem*> getRequirements() override { return {drivetrain_}; }

private:
    DrivetrainSubsystem* drivetrain_;
    double targetRad_;  // rad
    bool finish_;
    PID pid_{CONFIG::TURN_PID};
    std::uint32_t lastMs_ = 0;
    std::uint32_t settleStartMs_ = 0;
};
