#include "subsystems/localization.h"

#include "command/instantCommand.h"
#include "config.h"
#include "localization/odometry.h"

#include "pros/error.h"
#include "pros/llemu.hpp"

#include <cmath>

LocalizationSubsystem::LocalizationSubsystem(int8_t imuPort, int8_t fwdRotationPort, int8_t latRotationPort, ILocalizer& localizer)
    : imu_(imuPort),
      fwdRotation_(fwdRotationPort),
      latRotation_(latRotationPort),
      localizer_(localizer),
      fwdPod_(CONFIG::TRACKING_WHEEL_DIAMETER_IN),
      latPod_(CONFIG::TRACKING_WHEEL_DIAMETER_IN) {
    imu_.reset(true);
    imu_.tare();
}

Eigen::Vector3f LocalizationSubsystem::getPose() const {
    return localizer_.getPose();
}

void LocalizationSubsystem::setPose(double xIn, double yIn, double thetaRad) {
    localizer_.setPose(Eigen::Vector3f(xIn, yIn, thetaRad));
}

void LocalizationSubsystem::periodic() {
    const double imuRaw = imu_.get_rotation();
    if (!std::isfinite(imuRaw)) return;

    if (!imuReady_) {
        const int32_t fwdRaw = fwdRotation_.get_position();
        const int32_t latRaw = latRotation_.get_position();
        if (fwdRaw == PROS_ERR || latRaw == PROS_ERR) return;
        fwdPod_.seed(fwdRaw);
        latPod_.seed(latRaw);
        prevImuDeg_ = imuRaw;
        imuReady_ = true;
        return;
    }

    const int32_t fwdRaw = fwdRotation_.get_position();
    const int32_t latRaw = latRotation_.get_position();
    if (fwdRaw == PROS_ERR || latRaw == PROS_ERR) return;

    const double deltaFwd = fwdPod_.update(fwdRaw);
    const double deltaLat = latPod_.update(latRaw);
    const double deltaTheta = CONFIG::degToRad(imuRaw - prevImuDeg_);
    prevImuDeg_ = imuRaw;

    // Pods read wheel travel. Offsets convert that to travel of the robot centre.
    // The lateral pod is reversed in hardware, so positive already means right.
    localizer_.motionUpdate(Odometry::centerFwd(deltaFwd, deltaTheta), Odometry::centerLatRight(deltaLat, deltaTheta),
                            deltaTheta);
}

InstantCommand* LocalizationSubsystem::setPoseCommand(double xIn, double yIn, double thetaRad) {
    return new InstantCommand([this, xIn, yIn, thetaRad]() { this->setPose(xIn, yIn, thetaRad); }, {this});
}
