#include "attitude_controller.h"
#include "motor_control.h"  // 모터 제어 헤더 포함
#include "pose_estimator.h"
#include "../ioss/rc_input.h"

// 각 축에 대한 PID 제어기 설정
PIDController roll_pid(0.1f, 0.01f, 0.05f, 10.0f, 100.0f); // integral_limit과 output_limit 추가
PIDController pitch_pid(0.1f, 0.01f, 0.05f, 10.0f, 100.0f);
PIDController yaw_pid(0.2f, 0.01f, 0.1f, 10.0f, 100.0f);

void controlAttitude(const Eigen::VectorXf& currentState, const RCInput& rcInput, float dt) {
    if (dt < 0.001f) {
        return;  // 너무 작은 dt 값 방어
    }

    // 목표 각도 설정 (RC 입력에 따라)
    float targetRoll = rcInput.roll * 45.0f;
    float targetPitch = rcInput.pitch * 45.0f;
    float targetYaw = rcInput.yaw * 180.0f;

    // 현재 각도 (PoseEstimator에서 상태 값으로 받아옴)
    float currentRoll = currentState(6);
    float currentPitch = currentState(7);
    float currentYaw = currentState(8);

    // Yaw 각도 wrap-around 처리
    float yawError = targetYaw - currentYaw;
    if (yawError > 180.0f) yawError -= 360.0f;
    if (yawError < -180.0f) yawError += 360.0f;

    // PID 제어기로 각 축에 대한 제어 신호 계산
    float roll_output = roll_pid.update(targetRoll, currentRoll, dt);
    float pitch_output = pitch_pid.update(targetPitch, currentPitch, dt);
    float yaw_output = yaw_pid.update(yawError, 0.0f, dt); // Yaw는 각도 에러만 사용

    // 각 모터에 전달할 PWM 신호 계산
    float pwm_motor1 = roll_output - pitch_output + yaw_output;
    float pwm_motor2 = -roll_output + pitch_output + yaw_output;
    float pwm_motor3 = -roll_output - pitch_output - yaw_output;
    float pwm_motor4 = roll_output + pitch_output - yaw_output;

    // 모터 PWM 값 제한
    pwm_motor1 = fmaxf(fminf(pwm_motor1, PWM_MAX), PWM_MIN);
    pwm_motor2 = fmaxf(fminf(pwm_motor2, PWM_MAX), PWM_MIN);
    pwm_motor3 = fmaxf(fminf(pwm_motor3, PWM_MAX), PWM_MIN);
    pwm_motor4 = fmaxf(fminf(pwm_motor4, PWM_MAX), PWM_MIN);

    // 3 1 모터 넘버
    // 2 4

    // 모터 제어 함수 호출 (PWM 신호 전달)
    setMotorPWM(1, pwm_motor1);
    setMotorPWM(2, pwm_motor2);
    setMotorPWM(3, pwm_motor3);
    setMotorPWM(4, pwm_motor4);
}