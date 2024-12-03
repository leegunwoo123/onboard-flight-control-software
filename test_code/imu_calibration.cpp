#include "imu_calibration.h"
#include "imu_sensor.h"
#include <iostream>
#include <vector>
#include <numeric>  // accumulate 사용을 위한 헤더 추가
#include <unistd.h> // usleep 사용을 위한 헤더 추가
#include <termios.h> // B115200 정의를 위한 헤더 추가
#include <chrono> // 시간 측정을 위한 헤더 추가
#include <thread>
#include <cmath>        // 수학 함수 및 상수 사용
#define _USE_MATH_DEFINES  // M_PI 상수를 활성화

IMUCalibrationData calibrateIMU() {
    initIMU("/dev/ttyUSB0", B115200);
    IMUCalibrationData calibrationData;
    IMUData imuData;
    std::vector<float> accelX_samples, accelY_samples, accelZ_samples;
    std::vector<float> gyroX_samples, gyroY_samples, gyroZ_samples;
    std::vector<float> roll_samples, pitch_samples, yaw_samples;

    // 보완 필터 상수
    const float alpha = 0.98f;

    // 초기 각도 (static으로 선언하여 이전 값을 유지)
    float yaw = 0.0f;

    // 시작 시간 설정
    auto start_time = std::chrono::high_resolution_clock::now();
    int sample_count = 0;

    // 100개의 IMU 데이터 샘플 수집
    while (sample_count < 100) {
        IMUData imuData = readIMU();  // IMU 데이터 읽기

        // 각 축의 가속도 데이터를 샘플로 수집
        roll_samples.push_back(imuData.roll_angle);
        pitch_samples.push_back(imuData.pitch_angle);
        yaw_samples.push_back(imuData.yaw_angle);

        // 각 축의 자이로스코프 데이터를 샘플로 수집
        gyroX_samples.push_back(imuData.gyroX);
        gyroY_samples.push_back(imuData.gyroY);
        gyroZ_samples.push_back(imuData.gyroZ);

        usleep(25000);  // (100000)샘플 간 시간 간격 설정 (100ms)
        sample_count++;
    }

    // 평균값 계산하여 오프셋 설정
    if (!roll_samples.empty()) {
        calibrationData.offsetroll = std::accumulate(roll_samples.begin(), roll_samples.end(), 0.0f) / roll_samples.size();
    }

    if (!pitch_samples.empty()) {
        calibrationData.offsetpitch = std::accumulate(pitch_samples.begin(), pitch_samples.end(), 0.0f) / pitch_samples.size();
    }

    if (!yaw_samples.empty()) {
        calibrationData.offsetyaw = std::accumulate(yaw_samples.begin(), yaw_samples.end(), 0.0f) / yaw_samples.size();
    }

    // 자이로스코프 평균값도 필요하다면 아래와 같이 추가할 수 있습니다.
    if (!gyroX_samples.empty()) {
        calibrationData.offsetGyroX = std::accumulate(gyroX_samples.begin(), gyroX_samples.end(), 0.0f) / gyroX_samples.size();
    }

    if (!gyroY_samples.empty()) {
        calibrationData.offsetGyroY = std::accumulate(gyroY_samples.begin(), gyroY_samples.end(), 0.0f) / gyroY_samples.size();
    }

    if (!gyroZ_samples.empty()) {
        calibrationData.offsetGyroZ = std::accumulate(gyroZ_samples.begin(), gyroZ_samples.end(), 0.0f) / gyroZ_samples.size();
    }

    std::cout << "Calibration Complete:\n";

    std::cout << "Accelerometer Offsets:\n";
    std::cout << "roll: " << calibrationData.offsetroll << "\n";
    std::cout << "pitch: " << calibrationData.offsetpitch << "\n";
    std::cout << "yaw: " << calibrationData.offsetyaw << "\n";

    std::cout << "Gyroscope Offsets:\n";
    std::cout << "Offset Gyro X: " << calibrationData.offsetGyroX << "\n";
    std::cout << "Offset Gyro Y: " << calibrationData.offsetGyroY << "\n";
    std::cout << "Offset Gyro Z: " << calibrationData.offsetGyroZ << "\n";

    // 보정된 자이로스코프 데이터
    float gyroX_calibrated = imuData.gyroX - calibrationData.offsetGyroX;
    float gyroY_calibrated = imuData.gyroY - calibrationData.offsetGyroY;
    float gyroZ_calibrated = imuData.gyroZ - calibrationData.offsetGyroZ;

    std::cout << "target angle:\n";
    std::cout << "roll: " << calibrationData.offsetroll << "\n";
    std::cout << "pitch: " << calibrationData.offsetpitch << "\n";
    std::cout << "yaw: " << calibrationData.offsetyaw << "\n";
    
    return calibrationData;
}