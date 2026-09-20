#pragma once

#include "config.h"
#include "localizer.h"

#include <cmath>
#include <utility>

namespace Odometry {

    inline double centerFwd(double deltaFwd, double deltaTheta) {
        return deltaFwd + deltaTheta * CONFIG::FWD_WHEEL_RIGHT_OFFSET_IN;  // in
    }
    
    inline double centerLatRight(double deltaLat, double deltaTheta) {
        return deltaLat - deltaTheta * CONFIG::LAT_WHEEL_FWD_OFFSET_IN;  // in
    }
    
    // World-frame displacement (east, north) from center-frame forward / right travel.
    inline std::pair<double, double> arcStep(double dFwd, double dLatRight, double th0, double deltaTheta) {
        const double th1 = th0 + deltaTheta;
        if (std::fabs(deltaTheta) < 1e-9) {
            const double dx = dFwd * std::sin(th0) + dLatRight * std::cos(th0);
            const double dy = dFwd * std::cos(th0) - dLatRight * std::sin(th0);
            return {dx, dy};
        }
        const double inv = 1.0 / deltaTheta;
        const double cosDiff = std::cos(th0) - std::cos(th1);
        const double sinDiff = std::sin(th1) - std::sin(th0);
        const double dx = inv * (dFwd * cosDiff + dLatRight * sinDiff);
        const double dy = inv * (dFwd * sinDiff - dLatRight * cosDiff);
        return {dx, dy};
    }
    
    }  // namespace Odometry

class OdometryLocalizer : public ILocalizer {
    public:
    OdometryLocalizer() : pose(0.0, 0.0, 0.0) {}
    ~OdometryLocalizer() override = default;
    void motionUpdate(double dFwd, double dLatRight, double deltaTheta) override {
        std::pair<double, double> displacement = Odometry::arcStep(dFwd, dLatRight, pose.z(), deltaTheta);
        pose.x() += displacement.first;
        pose.y() += displacement.second;
        pose.z() += deltaTheta;
    };
    void sensorUpdate(double distance, double angle) override {};
    void setPose(const Eigen::Vector3f& newPose) override {
        pose = newPose;
    };
    Eigen::Vector3f getPose() override {
        return pose;
    };
    private:
    Eigen::Vector3f pose;
};
