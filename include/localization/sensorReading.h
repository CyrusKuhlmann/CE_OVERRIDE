#pragma once

#include "Eigen/Dense"

class ISensorReading {
    public:
    virtual ~ISensorReading() = default;
    virtual double getDistance() = 0;
    virtual double getAngle() = 0;
};