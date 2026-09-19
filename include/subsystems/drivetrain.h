#pragma once

#include "command/instantCommand.h"
#include "command/runCommand.h"
#include "config.h"
#include "localization/odomPod.h"
#include "localization/odometry.h"

#include "Eigen/Eigen"
#include "pros/imu.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"

#include <cstdint>
#include <initializer_list>

class DrivetrainSubsystem : public Subsystem {
public:
    DrivetrainSubsystem(std::initializer_list<int8_t> leftPorts, std::initializer_list<int8_t> rightPorts,
                        int8_t imuPort, int8_t fwdRotationPort, int8_t latRotationPort);

    void periodic() override;

    void setPct(double left, double right);  // [-1, 1]
    void setVoltages(int leftMv, int rightMv);  // mV
    void stop();

    Eigen::Vector3f getPose() const;  // x east (in), y north (in), theta rad CW from +Y
    double getX() const { return getPose().x(); }
    double getY() const { return getPose().y(); }
    double getAngle() const { return getPose().z(); }  // rad

    void setPose(double xIn, double yIn, double thetaRad);

    RunCommand* tankDrive(pros::Controller& controller);
    RunCommand* doubleArcadeDrive(pros::Controller& controller);
    RunCommand* singleArcadeDrive(pros::Controller& controller);
    RunCommand* drive(pros::Controller& controller);
    InstantCommand* setPoseCommand(double xIn, double yIn, double thetaRad);

    double imuHeadingDeg() const { return imu_.get_heading(); }
    double imuRotationDeg() const { return imu_.get_rotation(); }

private:
    void integrateOdometry(double deltaFwd, double deltaLat, double deltaTheta);
    static double mappedDriveAxis(pros::Controller& controller, pros::controller_analog_e_t axis);
    static double applyDriveNonlinearity(double x);

    pros::MotorGroup leftMotors_;
    pros::MotorGroup rightMotors_;
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
