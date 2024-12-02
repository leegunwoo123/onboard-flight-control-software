#include "imu_calibration.h"
#include "imu_sensor.h" // 왜 안 들어가 있었지?
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
    // initIMU("/dev/ttyUSB0", B115200);
    IMUCalibrationData calibrationData;
    
    std::vector<float> accelX_samples, accelY_samples, accelZ_samples;
    std::vector<float> gyroX_samples, gyroY_samples, gyroZ_samples;
    std::vector<float> roll_samples, pitch_samples;

    // 시작 시간 설정
    auto start_time = std::chrono::high_resolution_clock::now();
    int sample_count = 0;

    // 3초 동안 IMU 데이터 샘플 수집
    while (true) {
        auto current_time = std::chrono::high_resolution_clock::now();
        auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
        
        if (elapsed_time >= 3) break;
        IMUData imuData = readIMU();  // IMU 데이터 읽기

        // 각 축의 가속도 데이터를 샘플로 수집
        accelX_samples.push_back(imuData.accelX);
        accelY_samples.push_back(imuData.accelY);
        accelZ_samples.push_back(imuData.accelZ);

        // 각 축의 자이로스코프 데이터를 샘플로 수집
        gyroX_samples.push_back(imuData.gyroX);
        gyroY_samples.push_back(imuData.gyroY);
        gyroZ_samples.push_back(imuData.gyroZ);
        
        float roll = atan2(imuData.accelY, imuData.accelZ) * 180.0 / M_PI;
        float pitch = atan2(-imuData.accelX, sqrt(imuData.accelY * imuData.accelY + imuData.accelZ * imuData.accelZ)) * 180.0 / M_PI;

        roll_samples.push_back(roll);
        pitch_samples.push_back(pitch);

        usleep(10000);  // 샘플 간 시간 간격 설정 (10ms)
        sample_count++;
    }

    // 평균값 계산하여 오프셋 설정
    calibrationData.offsetX = std::accumulate(accelX_samples.begin(), accelX_samples.end(), 0.0f) / sample_count;
    calibrationData.offsetY = std::accumulate(accelY_samples.begin(), accelY_samples.end(), 0.0f) / sample_count;
    calibrationData.offsetZ = std::accumulate(accelZ_samples.begin(), accelZ_samples.end(), 0.0f) / sample_count;

    calibrationData.offsetGyroX = std::accumulate(gyroX_samples.begin(), gyroX_samples.end(), 0.0f) / sample_count;
    calibrationData.offsetGyroY = std::accumulate(gyroY_samples.begin(), gyroY_samples.end(), 0.0f) / sample_count;
    calibrationData.offsetGyroZ = std::accumulate(gyroZ_samples.begin(), gyroZ_samples.end(), 0.0f) / sample_count;

    calibrationData.stlRoll = std::accumulate(roll_samples.begin(), roll_samples.end(), 0.0f) / sample_count;
    calibrationData.stlPitch = std::accumulate(pitch_samples.begin(), pitch_samples.end(), 0.0f) / sample_count;

    std::cout << "Calibration Complete:\n";

    std::cout << "Accelerometer Offsets:\n";
    std::cout << "Offset Accel X: " << calibrationData.offsetX << "\n";
    std::cout << "Offset Accel Y: " << calibrationData.offsetY << "\n";
    std::cout << "Offset Accel Z: " << calibrationData.offsetZ << "\n";

    std::cout << "Gyroscope Offsets:\n";
    std::cout << "Offset Gyro X: " << calibrationData.offsetGyroX << "\n";
    std::cout << "Offset Gyro Y: " << calibrationData.offsetGyroY << "\n";
    std::cout << "Offset Gyro Z: " << calibrationData.offsetGyroZ << "\n";

    std::cout << "Roll Pitch cali:\n";
    std::cout << "Roll: " << calibrationData.stlRoll << "\n";
    std::cout << "Pitch: " << calibrationData.stlPitch << "\n";
    return calibrationData;
}