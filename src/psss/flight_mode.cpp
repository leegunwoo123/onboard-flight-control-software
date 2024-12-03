#include "flight_mode.h"
#include "rc_input.h"
#include "motor_control.h"
#include "gps_sensor.h"
#include "udp_communication.h"  // UDP 통신 관련
#include <Eigen/Dense>
#include <iostream>
#include <thread>  
#include <chrono>  // std::chrono::milliseconds

// 수동 비행 모드
void manualFlight(Eigen::VectorXf& state) {
    while (true) {
        // RC 입력을 읽어옵니다.
        RCInput rcInput = readRCInput();

        // 각 목표 각도 및 스로틀 계산
        float targetRoll = rcInput.roll * 45.0;    // 롤의 목표 각도 (±45도)
        float targetPitch = rcInput.pitch * 45.0;  // 피치의 목표 각도 (±45도)
        float targetYaw = rcInput.yaw * 180.0;     // 요의 목표 각도 (±180도)
        float targetThrottle = rcInput.throttle * 1000 + 1000;  // 스로틀 값 계산 (1000 ~ 2000)

        // 모터 출력 계산
        float motorOutputs[4];
        calculateMotorOutputs(state(6), state(7), state(8), targetRoll, targetPitch, targetYaw, targetThrottle, motorOutputs);

        // 모터 제어 신호 전달
        controlMotors(motorOutputs);

        // 일정 주기로 반복
        std::this_thread::sleep_for(std::chrono::milliseconds(20));  // 50Hz 주기로 실행 (20ms)
    }
}

// 자동 비행 모드
void autoFlight(Eigen::VectorXf& state) {
    while (true) {
        // GPS 센서 데이터를 읽어옵니다.
        GPSData gpsData = readGPSSensor();

        // 목표 위치 계산 (목표는 미리 설정된 경도, 위도, 고도 등)
        Eigen::Vector3f targetPosition(37.7749, -122.4194, 100.0);  // 예시로 특정 좌표와 고도 설정

        // 위치 오류 계산
        Eigen::Vector3f positionError = targetPosition - gpsData.position;

        // 위치 오류를 기반으로 모터 출력 계산
        float motorOutputs[4];
        calculateMotorOutputs(state(6), state(7), state(8), positionError(0), positionError(1), positionError(2), 1500, motorOutputs);

        // 모터 제어 신호 전달
        controlMotors(motorOutputs);

        // 일정 주기로 반복
        std::this_thread::sleep_for(std::chrono::milliseconds(20));  // 50Hz 주기로 실행
    }
}

// 고투(Goto) 비행 모드
void gotoFlight(Eigen::VectorXf& state) {
    // GPS 목표 좌표를 받아서 설정
    Eigen::Vector3f targetPosition = getGotoTargetPosition();

    while (true) {
        // GPS 센서 데이터를 읽어옵니다.
        GPSData gpsData = readGPSSensor();

        // 목표 위치까지의 거리 계산
        Eigen::Vector3f positionError = targetPosition - gpsData.position;

        // 위치 오류를 기반으로 모터 출력 계산
        float motorOutputs[4];
        calculateMotorOutputs(state(6), state(7), state(8), positionError(0), positionError(1), positionError(2), 1500, motorOutputs);

        // 모터 제어 신호 전달
        controlMotors(motorOutputs);

        // 목표 위치에 도달했는지 확인
        if (positionError.norm() < 0.5) {
            std::cout << "목표 지점에 도달했습니다!" << std::endl;
            break;
        }

        // 일정 주기로 반복
        std::this_thread::sleep_for(std::chrono::milliseconds(20));  // 50Hz 주기로 실행
    }
}
