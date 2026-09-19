#pragma once

// Active autonomous selection.

enum Auton { NONE, TEST };

#define AUTON TEST

namespace auton {

struct StartPose {
    double xIn = 0.0;         // in, east
    double yIn = 0.0;         // in, north
    double headingDeg = 0.0;  // deg, CW from +Y
};

}  // namespace auton
