#include "velocity_pid.h"

VelocityPID::VelocityPID(double kp, double ki, double kd, double velocity_scale)
    : kp_(kp), ki_(ki), kd_(kd), integral_(0.0), previous_error_(0.0), velocity_scale_(velocity_scale) {}

int32_t VelocityPID::compute(double setpoint, double measured_value, double dt) {
    double error = setpoint * velocity_scale_ - measured_value;
    integral_ += error * dt;
    float derivative = (error - previous_error_) / dt;
    previous_error_ = error;
    return static_cast<int32_t>((kp_ * error) + (ki_ * integral_) + (kd_ * derivative));
}

void VelocityPID::reset() {
    integral_ = 0.0f;
    previous_error_ = 0.0f;
}