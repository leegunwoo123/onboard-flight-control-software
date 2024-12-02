#include "attitude_controller.h"
#include "pose_estimator.h"
#include "rc_input.h"
#include "motor_control.h"
#include <array>
#include <cmath>
#include <iostream>
#include <chrono>
#include <unistd.h>
#include <Eigen/Dense>
#include <iomanip>
#include <termios.h>

// 각 축에 대한 PID 제어기 설정
PIDController roll_pid(0.1f, 0.01f, 0.05f);
PIDController pitch_pid(0.1f, 0.01f, 0.05f);
PIDController yaw_pid(0.2f, 0.01f, 0.1f);

// PoseEstimator 객체 생성
PoseEstimator poseEstimator;  // pose_estimator 초기화

// IMU 캘리브레이션 오프셋 값
const float offsetX = 0.12407f;
const float offsetY = -0.13802f;
const float offsetZ = -9.73542f;

// 각도 제한 상수
const float MAX_TARGET_ANGLE = 30.0f;  // 최대 목표 각도 (롤, 피치)

std::array<float, 4> controlAttitude(float dt) {
    if (dt < 0.001f) {
        std::cout << "false" << std::endl;
        return {PWM_MIN, PWM_MIN, PWM_MIN, PWM_MIN};  // dt가 너무 작을 경우 안전한 최소값 반환
    }

    // RC 입력 값을 직접 읽어 목표 각도를 설정
    int rollInput = readRCChannel(1) / 1;  // 채널 1: 롤 입력
    int pitchInput = readRCChannel(2) / 1; // 채널 2: 피치 입력
    int yawInput = readRCChannel(4) / 1;   // 채널 4: 요 입력
    int throttleInput = readRCChannel(3) / 1; // 채널 3: 스로틀 입력

    // 유효하지 않은 입력값 처리
    // if (rollInput == -1 || pitchInput == -1 || yawInput == -1) {
    //     std::cerr << "Error reading RC input" << std::endl;
    //     return {1000.0f, 1100.0f, 1200.0f, 1300.0f};
    // }

    // RC 입력이 없을 경우 목표 각도를 0으로 설정하여 수평 상태 유지
    float targetRoll = (fabs(rollInput) < 0.05f) ? offsetX : rollInput * MAX_TARGET_ANGLE;
    float targetPitch = (fabs(pitchInput) < 0.05f) ? offsetY : pitchInput * MAX_TARGET_ANGLE;
    float targetYaw = yawInput * 180.0f;

    // 현재 자세 추정값 가져오기 (x, y, z 위치와 roll, pitch, yaw만 사용)
    Eigen::VectorXf currentPose = poseEstimator.getPose();
    float currentRoll = currentPose(3);  // roll 값 (상태 벡터의 4번째 요소)
    float currentPitch = currentPose(4); // pitch 값 (상태 벡터의 5번째 요소)
    float currentYaw = currentPose(5);   // yaw 값 (상태 벡터의 6번째 요소)

    // 각도 오차 계산
    float yawError = targetYaw - currentYaw;
    if (yawError > 180.0f) yawError -= 360.0f;
    if (yawError < -180.0f) yawError += 360.0f;

    // PID 제어기 업데이트
    float roll_output = roll_pid.update(targetRoll, currentRoll, dt);
    float pitch_output = pitch_pid.update(targetPitch, currentPitch, dt);
    float yaw_output = yaw_pid.update(yawError, 0.0f, dt);
    float throttle_out = throttleInput;

    // 모터에 대한 PWM 값 계산
    float pwm_motor1 = throttle_out + roll_output + pitch_output + yaw_output;  // 모터 1 (CW)
    float pwm_motor2 = throttle_out - roll_output + pitch_output - yaw_output;  // 모터 2 (CCW)
    float pwm_motor3 = throttle_out - roll_output - pitch_output + yaw_output;  // 모터 3 (CCW)
    float pwm_motor4 = throttle_out + roll_output - pitch_output - yaw_output;  // 모터 4 (CW)

    // std::cerr << "Motor PWM Outputs: "
    //       << "Motor 1: " << pwm_motor1 << ", "
    //       << "Motor 2: " << pwm_motor2 << ", "
    //       << "Motor 3: " << pwm_motor3 << ", "
    //       << "Motor 4: " << pwm_motor4 << std::endl;

    // PWM 값 제한
    pwm_motor1 = fmaxf(fminf(pwm_motor1, PWM_MAX), PWM_MIN);
    pwm_motor2 = fmaxf(fminf(pwm_motor2, PWM_MAX), PWM_MIN);
    pwm_motor3 = fmaxf(fminf(pwm_motor3, PWM_MAX), PWM_MIN);
    pwm_motor4 = fmaxf(fminf(pwm_motor4, PWM_MAX), PWM_MIN);

    // RC 입력이 중립에 있을 경우 PID 제어기 초기화
    if (fabs(rollInput) < 0.05f && fabs(pitchInput) < 0.05f && fabs(yawInput) < 0.05f) {
        roll_pid.reset();
        pitch_pid.reset();
        yaw_pid.reset();
    }

    return {pwm_motor1, pwm_motor2, pwm_motor3, pwm_motor4};
}

void setMotorPWM(int motor_index, float pwm_value);

void setMotorPWMImpl(int motor_index, float pwm_value) {
    // 모터 PWM 설정 함수 구현 (실제 환경에 맞게 수정 필요)
    std::cout << "Setting motor " << motor_index << " to PWM value: " << pwm_value << std::endl;
}

int main() {
    // RC 입력 초기화
    const std::string port = "/dev/ttyAMA0";
    const int baudRate = 115200; // B115200 대신 일반 정수 값 사용
    initRC(port, baudRate);

    // 루프를 돌면서 주기적으로 제어 수행
    auto last_time = std::chrono::steady_clock::now();
    while (true) { // 무한 반복
        // 현재 시간과 이전 시간의 차이를 계산하여 dt 계산
        auto current_time = std::chrono::steady_clock::now();
        float dt = std::chrono::duration<float>(current_time - last_time).count();
        last_time = current_time;
        // dt가 0인 경우 방지
        if (dt <= 0.0f) {
            dt = 0.001f; // 최소한의 값으로 설정하여 계산이 진행되도록 함
        }
        // 자세 제어 및 PWM 값 계산
        std::array<float, 4> pwm_values = controlAttitude(dt);
        // 모터 제어 (모터에 PWM 신호를 전송하는 부분은 실제 환경에 맞게 구현해야 함)
        setMotorPWMImpl(0, pwm_values[0]);
        setMotorPWMImpl(1, pwm_values[1]);
        setMotorPWMImpl(2, pwm_values[2]);
        setMotorPWMImpl(3, pwm_values[3]);

        // 20ms 대기 (50Hz 주기)
        usleep(20000);
    }

    return 0;
}
// #include "attitude_controller.h"
// #include "pose_estimator.h"
// #include "rc_input.h"
// #include "motor_control.h"
// #include <array>
// #include <cmath>
// #include <iostream>
// #include <chrono>
// #include <unistd.h>
// #include <Eigen/Dense>
// #include <iomanip>
// #include <termios.h>

// // 각 축에 대한 PID 제어기 설정
// PIDController roll_pid(0.1f, 0.01f, 0.05f);
// PIDController pitch_pid(0.1f, 0.01f, 0.05f);
// PIDController yaw_pid(0.2f, 0.01f, 0.1f);

// // PoseEstimator 객체 생성
// PoseEstimator poseEstimator;  // pose_estimator 초기화

// // IMU 캘리브레이션 오프셋 값
// const float offsetX = 0.12407f;
// const float offsetY = -0.13802f;
// const float offsetZ = -9.73542f;

// // 각도 제한 상수
// const float MAX_TARGET_ANGLE = 30.0f;  // 최대 목표 각도 (롤, 피치)

// // PCA9685 객체 생성 (I2C 주소를 0x40으로 가정)
// PCA9685 pca(0x40);

// int computeThrottlePWM(double throttle_normalized);

// std::array<float, 4> controlAttitude(float dt) {
//     if (dt < 0.001f) {
//         std::cout << "false" << std::endl;
//         return {PWM_MIN, PWM_MIN, PWM_MIN, PWM_MIN};  // dt가 너무 작을 경우 안전한 최소값 반환
//     }

//     // RC 입력 값을 직접 읽어 목표 각도를 설정
//     int rollInput = readRCChannel(1) / 1;
//     int pitchInput = readRCChannel(2) / 1;
//     int yawInput = readRCChannel(4) / 1;
//     int throttleInput = readRCChannel(3) / 1;

//     // RC 입력이 없을 경우 목표 각도를 IMU 캘리브레이션 오프셋 값으로 설정하여 수평 상태 유지
//     float targetRoll = (fabs(rollInput) < 0.05f) ? offsetX : rollInput * MAX_TARGET_ANGLE; // 수정된 부분
//     float targetPitch = (fabs(pitchInput) < 0.05f) ? offsetY : pitchInput * MAX_TARGET_ANGLE; // 수정된 부분
//     float targetYaw = (fabs(yawInput) < 0.05f) ? offsetZ : yawInput * 180.0f; // 수정된 부분


//     // 현재 자세 추정값 가져오기
//     Eigen::VectorXf currentPose = poseEstimator.getPose();
//     float currentRoll = currentPose(3);
//     float currentPitch = currentPose(4);
//     float currentYaw = currentPose(5);

//     // 각도 오차 계산
//     float yawError = targetYaw - currentYaw;
//     if (yawError > 180.0f) yawError -= 360.0f;
//     if (yawError < -180.0f) yawError += 360.0f;

//     // PID 제어기 업데이트
//     float roll_output = roll_pid.update(targetRoll, currentRoll, dt);
//     float pitch_output = pitch_pid.update(targetPitch, currentPitch, dt);
//     float yaw_output = yaw_pid.update(yawError, 0.0f, dt);
//     float throttle_out = throttleInput;

//     double throttle_normalized = throttle_out;
//     double aileron_normalized = roll_output;
//     double elevator_normalized = pitch_output;
//     double rudder_normalized = yaw_output;

//     // 모터에 대한 PWM 값 계산
//     float pwm_motor1 = throttle_normalized; + aileron_normalized; + elevator_normalized; + rudder_normalized;
//     float pwm_motor2 = throttle_normalized; - aileron_normalized; + elevator_normalized; - rudder_normalized;
//     float pwm_motor3 = throttle_normalized; - aileron_normalized; - elevator_normalized; + rudder_normalized;
//     float pwm_motor4 = throttle_normalized; + aileron_normalized; - elevator_normalized; - rudder_normalized;

//     // PWM 값 제한
//     pwm_motor1 = fmaxf(fminf(pwm_motor1, PWM_MAX), PWM_MIN);
//     pwm_motor2 = fmaxf(fminf(pwm_motor2, PWM_MAX), PWM_MIN);
//     pwm_motor3 = fmaxf(fminf(pwm_motor3, PWM_MAX), PWM_MIN);
//     pwm_motor4 = fmaxf(fminf(pwm_motor4, PWM_MAX), PWM_MIN);

//     // RC 입력이 중립에 있을 경우 PID 제어기 초기화
//     if (fabs(rollInput) < 0.05f && fabs(pitchInput) < 0.05f && fabs(yawInput) < 0.05f) {
//         roll_pid.reset();
//         pitch_pid.reset();
//         yaw_pid.reset();
//     }
//     std::cout << "Motor PWM Outputs: "
//               << "Motor 1: " << pwm_motor1 << ", "
//               << "Motor 2: " << pwm_motor2 << ", "
//               << "Motor 3: " << pwm_motor3 << ", "
//               << "Motor 4: " << pwm_motor4 << std::endl;
//     return {pwm_motor1, pwm_motor2, pwm_motor3, pwm_motor4};
// }

// void setMotorPWMImpl(int motor_index, float pwm_value) {
//     // PCA9685의 setMotorSpeed 메서드를 사용하여 모터 PWM 설정
//     int pwm_scaled = static_cast<int>(pwm_value * 4095 / 1000); // 0-1000 범위를 0-4095로 매핑
//     pca.setMotorSpeed(motor_index, pwm_scaled);
// }

//     // 스로틀 PWM 계산 함수
// int computeThrottlePWM(double throttle_normalized) {
//     return static_cast<int>(PWM_MIN + throttle_normalized * (PWM_MAX - PWM_MIN));
//     }

// int main() {
//     // RC 입력 초기화
//     const std::string port = "/dev/ttyAMA0";
//     const int baudRate = 115200;
//     initRC(port, baudRate);

//     for (int i = 0; i < 4; ++i) {
//         setMotorPWMImpl(i, PWM_MIN);  // 최소 안전한 PWM 값을 모든 모터에 설정
//     }
//     usleep(2000000); // ESC가 초기화될 수 있도록 2초 대기

//     // 루프를 돌면서 주기적으로 제어 수행
//     auto last_time = std::chrono::steady_clock::now();
//     while (true) {
//         auto current_time = std::chrono::steady_clock::now();
//         //float dt = std::chrono::duration<float>(current_time - last_time).count();
//         float dt = 0.1f;
//         last_time = current_time;
//         if (dt <= 0.0f) {
//             dt = 0.001f;
//         }
        
//         std::array<float, 4> pwm_values = controlAttitude(dt);

//         setMotorPWMImpl(0, pwm_values[0]);
//         setMotorPWMImpl(1, pwm_values[1]);
//         setMotorPWMImpl(2, pwm_values[2]);
//         setMotorPWMImpl(3, pwm_values[3]);

//         usleep(20000); // 20ms delay (50Hz 주기)
//     }

//     return 0;
// }

