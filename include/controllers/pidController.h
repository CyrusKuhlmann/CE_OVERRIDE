#pragma once

#include "pros/rtos.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

// Tuning for one motion axis, modelled on lemlib::ControllerSettings.
// Errors are in the units of the axis (in for lateral, rad for angular).
struct ControllerSettings {
    double kP = 0.0;
    double kI = 0.0;
    double kD = 0.0;
    double windupRange = 0.0;     // integral only accumulates while |error| < this; 0 = always
    double smallError = 0.0;      // exit once |error| stays under this ...
    int smallErrorTimeoutMs = 0;  // ... for this long (ms)
    double largeError = 0.0;      // exit once |error| stays under this ...
    int largeErrorTimeoutMs = 0;  // ... for this long (ms)
    double slew = 0.0;            // max output change per update (pct of 12 V); 0 disables
};

// Error-driven PID, modelled on lemlib::PID. dt is measured between update() calls,
// so gains are per second rather than per loop iteration.
class PID {
public:
    PID(double kP, double kI, double kD, double windupRange = 0.0, bool signFlipReset = false)
        : kP_(kP), kI_(kI), kD_(kD), windupRange_(windupRange), signFlipReset_(signFlipReset) {}

    explicit PID(const ControllerSettings& settings, bool signFlipReset = false)
        : PID(settings.kP, settings.kI, settings.kD, settings.windupRange, signFlipReset) {}

    double update(double error) {
        const std::uint32_t now = pros::millis();
        const double dt = std::max((now - lastMs_) * 0.001, 0.001);  // s
        lastMs_ = now;

        double derivative = 0.0;
        if (!firstRun_) {
            integral_ += error * dt;
            derivative = (error - prevError_) / dt;
            if (signFlipReset_ && std::signbit(error) != std::signbit(prevError_)) integral_ = 0.0;
        }
        if (windupRange_ != 0.0 && std::fabs(error) > windupRange_) integral_ = 0.0;

        prevError_ = error;
        firstRun_ = false;
        return kP_ * error + kI_ * integral_ + kD_ * derivative;
    }

    void reset() {
        integral_ = 0.0;
        prevError_ = 0.0;
        firstRun_ = true;
    }

private:
    double kP_;
    double kI_;
    double kD_;
    double windupRange_;
    bool signFlipReset_;
    double integral_ = 0.0;
    double prevError_ = 0.0;
    std::uint32_t lastMs_ = 0;
    bool firstRun_ = true;
};

// Latches true once |error| has stayed within range for timeMs. Modelled on lemlib::ExitCondition.
class ExitCondition {
public:
    ExitCondition(double range, int timeMs) : range_(range), timeMs_(timeMs) {}

    bool update(double error) {
        const int now = static_cast<int>(pros::millis());
        if (std::fabs(error) > range_) startMs_ = -1;
        else if (startMs_ == -1) startMs_ = now;
        else if (now - startMs_ >= timeMs_) done_ = true;
        return done_;
    }

    bool getExit() const { return done_; }

    void reset() {
        startMs_ = -1;
        done_ = false;
    }

private:
    double range_;
    int timeMs_;  // ms
    int startMs_ = -1;
    bool done_ = false;
};
