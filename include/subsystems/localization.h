#pragma once

#include "command/instantCommand.h"
#include "localization/odomPod.h"

#include "Eigen/Eigen"
#include "pros/imu.hpp"
#include "pros/rotation.hpp"

#include <cstdint>

class LocalizationSubsystem : public Subsystem {
public:
    LocalizationSubsystem(int8_t imuPort, int8_t fwdRotationPort, int8_t latRotationPort);

    void periodic() override;

    Eigen::Vector3f getPose() const;  // x east (in), y north (in), theta rad CW from +Y
    double getX() const { return getPose().x(); }
    double getY() const { return getPose().y(); }
    double getAngle() const { return getPose().z(); }  // rad

    void setPose(double xIn, double yIn, double thetaRad);
    InstantCommand* setPoseCommand(double xIn, double yIn, double thetaRad);

    double imuHeadingDeg() const { return imu_.get_heading(); }
    double imuRotationDeg() const { return imu_.get_rotation(); }

private:
    void integrateOdometry(double deltaFwd, double deltaLat, double deltaTheta);

    pros::Imu imu_;
    pros::Rotation fwdRotation_;
    pros::Rotation latRotation_;

    OdomPod fwdPod_;
    OdomPod latPod_;
    bool imuReady_ = false;
    double prevImuDeg_ = 0.0;

    double odomX_ = 0.0;      // in
    double odomY_ = 0.0;      // in
    double odomTheta_ = 0.0;  // rad
};
