#pragma once

struct PIDGains {
    double kP = 0.0;
    double kI = 0.0;
    double kD = 0.0;
};

class PID {
public:
    explicit PID(PIDGains gains);
    PID(double kP, double kI, double kD);

    void setSetpoint(double setpoint);
    double getSetpoint() const { return setpoint_; }

    double calculate(double measurement, double dt);  // dt in s

    void reset();
    void setIntegralLimit(double limit);
    void setOutputLimit(double min, double max);
    void enableContinuousInput(double min, double max);
    void disableContinuousInput();

private:
    PIDGains gains_;
    double setpoint_ = 0.0;
    double integral_ = 0.0;
    double prevMeasurement_ = 0.0;
    bool firstRun_ = true;
    bool hasIntegralLimit_ = false;
    double integralLimit_ = 0.0;
    bool hasOutputLimit_ = false;
    double outputMin_ = 0.0;
    double outputMax_ = 0.0;
    bool continuous_ = false;
    double inputMin_ = 0.0;
    double inputMax_ = 0.0;
};
