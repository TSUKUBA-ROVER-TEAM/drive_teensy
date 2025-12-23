#ifndef VELOCITY_PID_HPP
#define VELOCITY_PID_HPP

#include <cstdint>

class VelocityPID {
public:
    VelocityPID(double kp, double ki, double kd, double velocity_scale);
    int32_t compute(double setpoint, double measured_value, double dt);
    void reset();

private:
    double kp_;
    double ki_;
    double kd_;
    double integral_;
    double previous_error_;
    double velocity_scale_;
};

#endif // VELOCITY_PID_HPP