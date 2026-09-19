#include "subsystems/drivetrain.h"

#include "command/instantCommand.h"
#include "command/runCommand.h"
#include "config.h"

#include "pros/error.h"
#include "pros/llemu.hpp"

#include <algorithm>
#include <cmath>

DrivetrainSubsystem::DrivetrainSubsystem(std::initializer_list<int8_t> leftPorts,
                                         std::initializer_list<int8_t> rightPorts, int8_t imuPort,
                                         int8_t fwdRotationPort, int8_t latRotationPort)
    : leftMotors_(leftPorts),
      rightMotors_(rightPorts),
      imu_(imuPort),
      fwdRotation_(fwdRotationPort),
      latRotation_(latRotationPort),
      fwdPod_(CONFIG::TRACKING_WHEEL_DIAMETER_IN),
      latPod_(CONFIG::TRACKING_WHEEL_DIAMETER_IN) {
    leftMotors_.set_gearing_all(pros::MotorGears::blue);
    rightMotors_.set_gearing_all(pros::MotorGears::blue);
    leftMotors_.set_encoder_units_all(pros::MotorUnits::degrees);
    rightMotors_.set_encoder_units_all(pros::MotorUnits::degrees);
    leftMotors_.set_brake_mode_all(pros::MotorBrake::brake);
    rightMotors_.set_brake_mode_all(pros::MotorBrake::brake);

    imu_.reset(true);
    imu_.tare();
}

void DrivetrainSubsystem::setPct(double left, double right) {
    left = std::clamp(left, -1.0, 1.0);
    right = std::clamp(right, -1.0, 1.0);
    setVoltages(static_cast<int>(left * 12000.0), static_cast<int>(right * 12000.0));
}

void DrivetrainSubsystem::setVoltages(int leftMv, int rightMv) {
    leftMotors_.move_voltage(leftMv);
    rightMotors_.move_voltage(rightMv);
}

void DrivetrainSubsystem::stop() { setPct(0.0, 0.0); }

inline double DrivetrainSubsystem::applyDriveNonlinearity(double x) {
    return CONFIG::DRIVE_NONLINEARITY * x * x * x + (1.0 - CONFIG::DRIVE_NONLINEARITY) * x;
}

double DrivetrainSubsystem::mappedDriveAxis(pros::Controller& controller, pros::controller_analog_e_t axis) {
    double x = std::clamp(static_cast<double>(controller.get_analog(axis)), -127.0, 127.0) / 127.0;
    if (std::fabs(x) < CONFIG::DRIVE_DEADBAND) return 0.0;
    return applyDriveNonlinearity(x);
}

Eigen::Vector3f DrivetrainSubsystem::getPose() const {
    return Eigen::Vector3f(static_cast<float>(odomX_), static_cast<float>(odomY_), static_cast<float>(odomTheta_));
}

void DrivetrainSubsystem::setPose(double xIn, double yIn, double thetaRad) {
    odomX_ = xIn;
    odomY_ = yIn;
    odomTheta_ = CONFIG::wrapPi(thetaRad);
}

void DrivetrainSubsystem::integrateOdometry(double deltaFwd, double deltaLat, double deltaTheta) {
    if (!std::isfinite(deltaFwd) || !std::isfinite(deltaLat) || !std::isfinite(deltaTheta)) return;

    const double dFwdC = Odometry::centerFwd(deltaFwd, deltaTheta);
    const double dLatC = Odometry::centerLatRight(deltaLat, deltaTheta);
    const auto [dx, dy] = Odometry::arcStep(dFwdC, dLatC, odomTheta_, deltaTheta);

    odomX_ += dx;
    odomY_ += dy;
    odomTheta_ = CONFIG::wrapPi(odomTheta_ + deltaTheta);
}

void DrivetrainSubsystem::periodic() {
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

RunCommand* DrivetrainSubsystem::tankDrive(pros::Controller& controller) {
    auto* pad = &controller;
    return new RunCommand(
        [this, pad]() {
            if (!pad->is_connected()) {
                this->stop();
                return;
            }
            const double left = mappedDriveAxis(*pad, pros::E_CONTROLLER_ANALOG_LEFT_Y);
            const double right = mappedDriveAxis(*pad, pros::E_CONTROLLER_ANALOG_RIGHT_Y);
            this->setPct(left, right);
        },
        {this});
}

RunCommand* DrivetrainSubsystem::doubleArcadeDrive(pros::Controller& controller) {
    auto* pad = &controller;
    return new RunCommand(
        [this, pad]() {
            if (!pad->is_connected()) {
                this->stop();
                return;
            }
            const double throttle = mappedDriveAxis(*pad, pros::E_CONTROLLER_ANALOG_RIGHT_Y);
            const double turn = mappedDriveAxis(*pad, pros::E_CONTROLLER_ANALOG_LEFT_X);

            double left = throttle + turn;
            double right = throttle - turn;

            const double max = std::max(1.0, std::max(std::fabs(left), std::fabs(right)));

            left /= max;
            right /= max;

            this->setPct(left, right);
        },
        {this});
}

RunCommand* DrivetrainSubsystem::singleArcadeDrive(pros::Controller& controller) {
    auto* pad = &controller;
    return new RunCommand(
        [this, pad]() {
            if (!pad->is_connected()) {
                this->stop();
                return;
            }
            const double throttle = mappedDriveAxis(*pad, pros::E_CONTROLLER_ANALOG_RIGHT_Y);
            const double turn =  mappedDriveAxis(*pad, pros::E_CONTROLLER_ANALOG_RIGHT_X);

            double left = throttle + turn;
            double right = throttle - turn;

            const double max = std::max(1.0, std::max(std::fabs(left), std::fabs(right)));

            left /= max;
            right /= max;

            this->setPct(left, right);
        },
        {this});
}

RunCommand* DrivetrainSubsystem::drive(pros::Controller& controller) {
    switch (CONFIG::DEFAULT_DRIVE_TYPE) {
        case CONFIG::DOUBLE_ARCADE:
            return doubleArcadeDrive(controller);
        case CONFIG::SINGLE_ARCADE:
            return singleArcadeDrive(controller);
        case CONFIG::TANK:
            return tankDrive(controller);
    }
}

InstantCommand* DrivetrainSubsystem::setPoseCommand(double xIn, double yIn, double thetaRad) {
    return new InstantCommand([this, xIn, yIn, thetaRad]() { this->setPose(xIn, yIn, thetaRad); }, {this});
}
