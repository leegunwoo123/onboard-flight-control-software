// #include "attitude_controller.h"
// #include "motor_control.h"
// #include <chrono>
// #include <array>
// #include <iostream>
//  #include <unistd.h>

// // Delta time 계산 함수
// double getDeltaTime() {
//     static auto last_time = std::chrono::steady_clock::now();
//     auto current_time = std::chrono::steady_clock::now();
//     double dt = std::chrono::duration<double>(current_time - last_time).count();
//     last_time = current_time;
//     return dt;
// }

// int main() {
//     // PCA9685 모터 드라이버 초기화
//     PCA9685 pca9685;

//     while (true) {
//         // Delta time 계산
//         double dt = getDeltaTime();

//         // Attitude controller를 통해 PWM 값 계산
//         std::array<float, 4> motor_pwms = controlAttitude(dt);

//         // PCA9685 모듈을 사용해 각 모터에 PWM 값 전달
//         pca9685.setMotorSpeed(0, static_cast<int>(motor_pwms[0]));
//         pca9685.setMotorSpeed(1, static_cast<int>(motor_pwms[1]));
//         pca9685.setMotorSpeed(2, static_cast<int>(motor_pwms[2]));
//         pca9685.setMotorSpeed(3, static_cast<int>(motor_pwms[3]));

//         // PWM 값 출력
//         std::cout << "\rMotor1: " << motor_pwms[0]
//                   << " Motor2: " << motor_pwms[1]
//                   << " Motor3: " << motor_pwms[2]
//                   << " Motor4: " << motor_pwms[3] << std::flush;

//         // 10ms 대기
//         usleep(10000);
//     }

//     return 0;
// }
#include "attitude_controller.h"
#include "motor_control.h"
#include <array>
#include <iostream>
#include <unistd.h>
#include <chrono>

// Delta time 계산 함수
double getDeltaTime() {
    static auto last_time = std::chrono::steady_clock::now();
    auto current_time = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(current_time - last_time).count();
    last_time = current_time;
    return dt;
}

int controlMotors(PCA9685 &pca9685) {
    while (true) {
        // Delta time 계산
        double dt = getDeltaTime();

        // Attitude controller를 통해 PWM 값 계산
        std::array<float, 4> motor_pwms = controlAttitude(dt);

        // PCA9685 모듈을 사용해 각 모터에 PWM 값 전달
        pca9685.setMotorSpeed(0, static_cast<int>(motor_pwms[0]));
        pca9685.setMotorSpeed(1, static_cast<int>(motor_pwms[1]));
        pca9685.setMotorSpeed(2, static_cast<int>(motor_pwms[2]));
        pca9685.setMotorSpeed(3, static_cast<int>(motor_pwms[3]));

        // PWM 값 출력
        std::cout << "\rMotor1: " << motor_pwms[0]
                  << " Motor2: " << motor_pwms[1]
                  << " Motor3: " << motor_pwms[2]
                  << " Motor4: " << motor_pwms[3] << "       " << std::flush;

        // 10ms 대기
        usleep(10000);
    }

    return 0;
}

int main() {
    // PCA9685 모터 드라이버 초기화
    PCA9685 pca9685;
    controlMotors(pca9685);
    return 0;
}
