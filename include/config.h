#pragma once

#include "controllers/pidController.h"

#include <cmath>
#include <cstdint>

// Internals: inches, radians (CW from +Y), seconds. Auton APIs take degrees.

namespace CONFIG {

constexpr double PI = 3.14159265358979323846;

constexpr int8_t LEFT_DRIVE[] = {-1, -2, -3};
constexpr int8_t RIGHT_DRIVE[] = {4, 5, 6};
constexpr int8_t IMU_PORT = 13;
constexpr int8_t FWD_ROTATION_PORT = 11;
constexpr int8_t LAT_ROTATION_PORT = -12;  // reversed in hardware

constexpr double TRACKING_WHEEL_DIAMETER_IN = 2.0;  // in
constexpr double FWD_WHEEL_RIGHT_OFFSET_IN = 1.0;   // in; wheel is left_in = -1
constexpr double LAT_WHEEL_FWD_OFFSET_IN = -5.5;    // in

constexpr double FIELD_HALF_IN = 72.0;  // in; 12 ft field, origin at centre

inline PIDGains TURN_PID = {.kP = 1.4, .kI = 0.0, .kD = 0.08};
inline PIDGains DRIVE_PID = {.kP = 0.055, .kI = 0.001, .kD = 0.016};
inline PIDGains DRIVE_HEADING_PID = {.kP = 0.9, .kI = 0.0, .kD = 0.04};

constexpr double ANGLE_FINISH_RAD = 0.035;   // rad ~ 2 deg
constexpr double DISTANCE_FINISH_IN = 0.75;  // in
constexpr int SETTLE_MS = 150;               // ms
constexpr double TURN_OUTPUT_LIMIT = 0.55;   // pct of 12 V
constexpr double DRIVE_OUTPUT_LIMIT = 0.85;  // pct of 12 V

constexpr double DRIVE_DEADBAND = 0.05;
constexpr double DRIVE_NONLINEARITY = 0.5;  // 0 = linear, 1 = cubic

inline double degToRad(double deg) { return deg * PI / 180.0; }
inline double radToDeg(double rad) { return rad * 180.0 / PI; }

inline double wrapPi(double a) {
    a = std::fmod(a + PI, 2.0 * PI);
    if (a < 0.0) a += 2.0 * PI;
    return a - PI;
}

enum driveType {
    DOUBLE_ARCADE,
    SINGLE_ARCADE,
    TANK,
};
constexpr driveType DEFAULT_DRIVE_TYPE = DOUBLE_ARCADE;

}  // namespace CONFIG
