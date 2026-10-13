#pragma once

#include "pros/rtos.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

// Output shaping shared by the motion commands, modelled on lemlib/util.hpp. Outputs are pct of 12 V.
namespace motion {

    // Limit how far output can move from its previous value in one update; maxChange <= 0 disables.
    inline double slew(double target, double current, double maxChange) {
        if (maxChange <= 0.0) return target;
        return current + std::clamp(target - current, -maxChange, maxChange);
    }

    // Raise a nonzero output to at least minSpeed, keeping its sign.
    inline double applyMinSpeed(double output, double minSpeed) {
        if (output == 0.0 || std::fabs(output) >= minSpeed) return output;
        return std::copysign(minSpeed, output);
    }

    // Shrink lateral so |lateral| + |angular| <= maxSpeed. Steering takes priority over speed.
    inline double prioritizeAngular(double lateral, double angular, double maxSpeed) {
        const double overturn = std::fabs(lateral) + std::fabs(angular) - maxSpeed;
        if (overturn <= 0.0) return lateral;
        return lateral - std::copysign(overturn, lateral);
    }

    // Lateral shaping for drives. Until close: slew and never drive away from the target.
    // direction is +1 when the target is ahead, -1 when behind.
    inline double shapeLateral(double output, double prevOutput, double direction, bool close, double maxSpeed,
                               double minSpeed, double slewRate) {
        output = std::clamp(output, -maxSpeed, maxSpeed);
        if (!close) {
            output = slew(output, prevOutput, slewRate);
            output = direction * std::max(output * direction, 0.0);
        }
        if (output * direction > 0.0) output = applyMinSpeed(output, minSpeed);
        return output;
    }

    // Motion timeout. A duration <= 0 never expires.
    class Timeout {
    public:
        explicit Timeout(int ms) : ms_(ms) {}

        void start() { startMs_ = pros::millis(); }
        bool isDone() const { return ms_ > 0 && pros::millis() - startMs_ >= static_cast<std::uint32_t>(ms_); }

    private:
        int ms_;  // ms
        std::uint32_t startMs_ = 0;
    };

}  // namespace motion
