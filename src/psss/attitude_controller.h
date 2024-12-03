#ifndef ATTITUDE_CONTROLLER_H
#define ATTITUDE_CONTROLLER_H

#include <Eigen/Dense>

class PIDController {
public:
    PIDController(float kp, float ki, float kd)
        : _kp(kp), _ki(ki), _kd(kd), _prev_error(0.0f), _integral(0.0f) {}

    float update(float target, float current, float dt) {
        float error = target - current;
        _integral += error * dt;
        float derivative = (error - _prev_error) / dt;
        _prev_error = error;
        return (_kp * error) + (_ki * _integral) + (_kd * derivative);
    }

private:
    float _kp, _ki, _kd;
    float _prev_error, _integral;
};

void controlAttitude(const Eigen::VectorXf& currentState, const RCInput& rcInput, float dt);

#endif