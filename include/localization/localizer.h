#pragma once

#include "Eigen/Dense"

class ILocalizer {
    public:
    virtual ~ILocalizer() = default;
    virtual void motionUpdate(double dFwd, double dLatRight, double deltaTheta) = 0;
    virtual void sensorUpdate(double distance, double angle) = 0;
    virtual void setPose(const Eigen::Vector3f& newPose) = 0;
    virtual Eigen::Vector3f getPose() = 0;
};