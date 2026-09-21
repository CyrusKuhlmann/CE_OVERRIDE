#pragma once

#include "command/command.h"
#include "config.h"
#include "controllers/pidController.h"
#include "subsystems/drivetrain.h"
#include "subsystems/localization.h"

#include "pros/rtos.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

// Point-turn to a field heading. Factories pass degrees; this command stores radians.
class Rotate : public Command {
public:
    Rotate(DrivetrainSubsystem* drivetrain, LocalizationSubsystem* localization, double headingRad,
           bool finish = true, int timeoutMs = 0)
        : drivetrain_(drivetrain),
          localization_(localization),
          targetRad_(headingRad),
          finish_(finish),
          timeoutMs_(timeoutMs) {}

    void initialize() override {
        pid_ = PID(CONFIG::TURN_PID);
        pid_.enableContinuousInput(-CONFIG::PI, CONFIG::PI);
        pid_.setOutputLimit(-CONFIG::TURN_OUTPUT_LIMIT, CONFIG::TURN_OUTPUT_LIMIT);
        pid_.setSetpoint(targetRad_);
        pid_.reset();
        lastMs_ = pros::millis();
        startMs_ = lastMs_;
        settleStartMs_ = 0;
        overshot_ = false;
        approachSign_ = 0.0;
    }

    void execute() override {
        const std::uint32_t now = pros::millis();
        const double dt = std::max((now - lastMs_) * 0.001, 0.001);
        lastMs_ = now;

        const double err = CONFIG::wrapPi(targetRad_ - localization_->getAngle());
        noteOvershoot(err);
        double out = pid_.calculate(localization_->getAngle(), dt);
        out = correctedOutput(out, err, exitTolerance(), CONFIG::TURN_CORRECTION_MIN, overshot_);
        drivetrain_->setPct(out, -out);
    }

    void end(bool /*interrupted*/) override { drivetrain_->stop(); }

    bool isFinished() override {
        if (timedOut()) return true;
        if (!finish_) return false;
        const double err = std::fabs(CONFIG::wrapPi(targetRad_ - localization_->getAngle()));
        if (err > exitTolerance()) {
            settleStartMs_ = 0;
            return false;
        }
        if (settleStartMs_ == 0) settleStartMs_ = pros::millis();
        return (pros::millis() - settleStartMs_) >= static_cast<std::uint32_t>(CONFIG::SETTLE_MS);
    }

    std::vector<Subsystem*> getRequirements() override { return {drivetrain_}; }

private:
    double exitTolerance() const { return overshot_ ? CONFIG::ANGLE_CORRECTION_RAD : CONFIG::ANGLE_FINISH_RAD; }

    bool timedOut() const {
        return timeoutMs_ > 0 && (pros::millis() - startMs_) >= static_cast<std::uint32_t>(timeoutMs_);
    }

    // Latch the first time the wrapped heading error changes sign, then turn back.
    void noteOvershoot(double err) {
        if (overshot_) return;
        if (approachSign_ == 0.0) {
            if (std::fabs(err) > 1e-4) approachSign_ = std::copysign(1.0, err);
            return;
        }
        if (err * approachSign_ < 0.0) {
            overshot_ = true;
            settleStartMs_ = 0;
            pid_.reset();
        }
    }

    static double correctedOutput(double output, double err, double tolerance, double minOutput, bool correcting) {
        if (!correcting || std::fabs(err) <= tolerance) return output;
        if (output * err > 0.0 && std::fabs(output) >= minOutput) return output;
        return std::copysign(minOutput, err);
    }

    DrivetrainSubsystem* drivetrain_;
    LocalizationSubsystem* localization_;
    double targetRad_;  // rad
    bool finish_;
    int timeoutMs_;  // ms; <= 0 disables
    PID pid_{CONFIG::TURN_PID};
    std::uint32_t lastMs_ = 0;
    std::uint32_t startMs_ = 0;
    std::uint32_t settleStartMs_ = 0;
    bool overshot_ = false;
    double approachSign_ = 0.0;
};
