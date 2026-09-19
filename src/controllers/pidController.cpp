#include "controllers/pidController.h"

#include <algorithm>
#include <cmath>

namespace {

double inputModulus(double value, double min, double max) {
    const double range = max - min;
    double wrapped = std::fmod(value - min, range);
    if (wrapped < 0.0) wrapped += range;
    return wrapped + min;
}

}  // namespace

PID::PID(PIDGains gains) : gains_(gains) {}

PID::PID(double kP, double kI, double kD) : gains_{kP, kI, kD} {}

void PID::setSetpoint(double setpoint) { setpoint_ = setpoint; }

double PID::calculate(double measurement, double dt) {
    if (dt <= 0.0) return 0.0;

    double error = setpoint_ - measurement;
    if (continuous_) {
        const double bound = (inputMax_ - inputMin_) / 2.0;
        error = inputModulus(error, -bound, bound);
    }

    const double P = gains_.kP * error;
    integral_ += error * dt;
    if (hasIntegralLimit_) {
        integral_ = std::clamp(integral_, -integralLimit_, integralLimit_);
    }
    const double I = gains_.kI * integral_;

    double D = 0.0;
    if (!firstRun_) {
        double delta = measurement - prevMeasurement_;
        if (continuous_) {
            const double bound = (inputMax_ - inputMin_) / 2.0;
            delta = inputModulus(delta, -bound, bound);
        }
        D = -gains_.kD * delta / dt;
    }
    firstRun_ = false;
    prevMeasurement_ = measurement;

    double output = P + I + D;
    if (hasOutputLimit_) {
        output = std::clamp(output, outputMin_, outputMax_);
    }
    return output;
}

void PID::reset() {
    integral_ = 0.0;
    prevMeasurement_ = 0.0;
    firstRun_ = true;
}

void PID::setIntegralLimit(double limit) {
    hasIntegralLimit_ = true;
    integralLimit_ = std::abs(limit);
}

void PID::setOutputLimit(double min, double max) {
    hasOutputLimit_ = true;
    outputMin_ = min;
    outputMax_ = max;
}

void PID::enableContinuousInput(double min, double max) {
    continuous_ = true;
    inputMin_ = min;
    inputMax_ = max;
}

void PID::disableContinuousInput() { continuous_ = false; }
