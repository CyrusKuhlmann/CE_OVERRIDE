#include "subsystems/localization.h"

#include "command/instantCommand.h"
#include "config.h"
#include "localization/odometry.h"

#include "pros/error.h"
#include "pros/llemu.hpp"

#include <cmath>

LocalizationSubsystem::LocalizationSubsystem(int8_t imuPort, int8_t fwdRotationPort, int8_t latRotationPort)
    : imu_(imuPort),
      fwdRotation_(fwdRotationPort),
      latRotation_(latRotationPort),
      fwdPod_(CONFIG::TRACKING_WHEEL_DIAMETER_IN),
      latPod_(CONFIG::TRACKING_WHEEL_DIAMETER_IN) {
    imu_.reset(true);
    imu_.tare();
}

Eigen::Vector3f LocalizationSubsystem::getPose() const {
    return Eigen::Vector3f(static_cast<float>(odomX_), static_cast<float>(odomY_), static_cast<float>(odomTheta_));
}

void LocalizationSubsystem::setPose(double xIn, double yIn, double thetaRad) {
    odomX_ = xIn;
    odomY_ = yIn;
    odomTheta_ = CONFIG::wrapPi(thetaRad);
}

void LocalizationSubsystem::integrateOdometry(double deltaFwd, double deltaLat, double deltaTheta) {
    if (!std::isfinite(deltaFwd) || !std::isfinite(deltaLat) || !std::isfinite(deltaTheta)) return;

    const double dFwdC = Odometry::centerFwd(deltaFwd, deltaTheta);
    const double dLatC = Odometry::centerLatRight(deltaLat, deltaTheta);
    const auto [dx, dy] = Odometry::arcStep(dFwdC, dLatC, odomTheta_, deltaTheta);

    odomX_ += dx;
    odomY_ += dy;
    odomTheta_ = CONFIG::wrapPi(odomTheta_ + deltaTheta);
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

    integrateOdometry(deltaFwd, deltaLat, deltaTheta);

    const auto pose = getPose();
    pros::lcd::print(3, "x %.1f  y %.1f  h %.1f", pose.x(), pose.y(), CONFIG::radToDeg(pose.z()));
    pros::lcd::print(4, "imu hdg %.1f  rot %.1f", imu_.get_heading(), imu_.get_rotation());
}

InstantCommand* LocalizationSubsystem::setPoseCommand(double xIn, double yIn, double thetaRad) {
    return new InstantCommand([this, xIn, yIn, thetaRad]() { this->setPose(xIn, yIn, thetaRad); }, {this});
}
