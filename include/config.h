#pragma once

#include "controllers/pidController.h"

#include <cmath>
#include <cstdint>
#include "Eigen/Dense"

// Internals: inches, radians (CW from +Y), seconds. Auton APIs take degrees.

namespace CONFIG {

constexpr double PI = 3.14159265358979323846;

// Drivetrain

constexpr int8_t LEFT_DRIVE[] = {2, -9, -20};
constexpr int8_t RIGHT_DRIVE[] = {-1, 14, 10};
constexpr int8_t INTAKE_PORT = 18;
constexpr int8_t LIFT_PORTS[] = {8, 6};
constexpr char WRIST_PORT = 'a';
constexpr char CLAW_PORT = 'b';
constexpr int8_t IMU_PORT = 15;
constexpr int8_t FWD_ROTATION_PORT = 3;
constexpr int8_t LAT_ROTATION_PORT = 16;  // reversed in hardware

enum driveType {
    DOUBLE_ARCADE,
    SINGLE_ARCADE,
    TANK,
};
constexpr driveType DEFAULT_DRIVE_TYPE = TANK;

// Localization

constexpr double TRACKING_WHEEL_DIAMETER_IN = 2.0;  // in
constexpr double FWD_WHEEL_RIGHT_OFFSET_IN = 0.1;  // in, side to side
constexpr double LAT_WHEEL_FWD_OFFSET_IN = 0.225;    // in, forward back

// get_rotation() under-reports. A 3-turn spin (expected ±6π rad), measured after the
// robot was set back on the start pose, averaged ±18.7467 rad in both directions
// (one CCW sample of -28.72 rad dropped). Scale = 6π / 18.7467.
constexpr double IMU_HEADING_SCALE = 1.00549;

constexpr double FIELD_HALF_IN = 72.0;  // in; 12 ft field, origin at centre
constexpr int NUM_PARTICLES = 350;
// Robot-frame ToF mounts: +x forward (in), +y left (in), yaw CW from forward (rad).
const Eigen::Vector3f FRONT_SENSOR_MOUNT = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
const Eigen::Vector3f RIGHT_SENSOR_MOUNT = Eigen::Vector3f(0.0f, -7.0f, static_cast<float>(PI / 2.0));
const Eigen::Vector3f BACK_SENSOR_MOUNT = Eigen::Vector3f(0.0f, 0.0f, static_cast<float>(PI));
const Eigen::Vector3f LEFT_SENSOR_MOUNT = Eigen::Vector3f(0.0f, 7.0f, static_cast<float>(-PI / 2.0));
constexpr double SIGMA_MOTION = 0.15;
constexpr double SIGMA_SENSOR = 1.0;
constexpr double SIGMA_HEADING = 0.04;
constexpr double PF_ESS_RATIO = 0.5;  // resample when ESS < ratio * N
constexpr double PF_INIT_STDEV_IN = 1.5;
constexpr double PF_INIT_STDEV_RAD = 0.05;

constexpr double DIST_FOV_SHORT_DEG = 36.0;
constexpr double DIST_FOV_LONG_DEG = 24.0;
constexpr double DIST_NEAR_IN = 200.0 / 25.4;
constexpr double DIST_MAX_IN = 2000.0 / 25.4;

// Motion Controllers
// Lateral errors are in inches, angular errors in radians. Speeds are pct of 12 V.

inline ControllerSettings LATERAL_CONTROLLER = {
    .kP = 0.235,
    .kI = 0.001,
    .kD = 0.01,
    .windupRange = 1.0,  // in
    .smallError = 0.75,  // in
    .smallErrorTimeoutMs = 150,
    .largeError = 3.0,  // in
    .largeErrorTimeoutMs = 500,
    .slew = 0.0,
};

inline ControllerSettings ANGULAR_CONTROLLER = {
    .kP = 2.35,
    .kI = 0.02,
    .kD = 0.1,
    .windupRange = 0.087,  // rad ~ 5 deg
    .smallError = 0.035,   // rad ~ 2 deg
    .smallErrorTimeoutMs = 150,
    .largeError = 0.052,  // rad ~ 3 deg
    .largeErrorTimeoutMs = 500,
    .slew = 0.0,
};

// Heading correction while driving; only the gains are used.
inline ControllerSettings HEADING_CONTROLLER = {.kP = 0.9, .kI = 0.0, .kD = 0.04};

constexpr double TURN_MAX_SPEED = 0.55;   // pct of 12 V
constexpr double DRIVE_MAX_SPEED = 0.85;  // pct of 12 V
constexpr double DRIVE_CLOSE_IN = 7.5;    // in; inside this, driving stops steering and may reverse

// Operator Control

constexpr double DRIVE_DEADBAND = 0.05;
constexpr double DRIVE_NONLINEARITY = 0.5;  // 0 = linear, 1 = cubic

// Utility Functions

inline double degToRad(double deg) { return deg * PI / 180.0; }
inline double radToDeg(double rad) { return rad * 180.0 / PI; }

inline double wrapPi(double a) {
    a = std::fmod(a + PI, 2.0 * PI);
    if (a < 0.0) a += 2.0 * PI;
    return a - PI;
}


}  // namespace CONFIG
