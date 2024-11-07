// #include "pose_estimator.h"
// #include "ekf.h"
// #include "../ioss/imu_sensor.h"
// #include "../ioss/gps_sensor.h"
// #include <math.h>
// #include <iostream> // 출력 테스트를 위해 사용
// #include <iomanip>
// #include <thread>  // 스레드를 사용하기 위해 필요

// PoseEstimator::PoseEstimator() : ekf(), running(true) {
//     currentState = Eigen::VectorXf::Zero(9);  // 9차원의 상태 벡터 초기화
//     imuAccel = Eigen::Vector3f::Zero();  // IMU 가속도 초기화
//     imuGyro = Eigen::Vector3f::Zero();   // IMU 자이로 초기화
//     gpsPos = Eigen::Vector3f::Zero();    // GPS 위치 초기화
//     gpsVel = Eigen::Vector3f::Zero();    // GPS 속도 초기화
    
//     imuThread = std::thread(&PoseEstimator::processIMU, this);  // IMU 스레드 생성
//     gpsThread = std::thread(&PoseEstimator::processGPS, this);  // GPS 스레드 생성
//     estimationThread = std::thread(&PoseEstimator::calculatePose, this);
// }

// PoseEstimator::~PoseEstimator() {
//     running = false;  // 스레드 종료 요청
//     if (estimationThread.joinable()) {
//         estimationThread.join();  // 스레드가 종료될 때까지 대기
//     }
//     if (imuThread.joinable()) {
//         imuThread.join();  // IMU 스레드 종료 대기
//     }
//     if (gpsThread.joinable()) {
//         gpsThread.join();  // GPS 스레드 종료 대기
//     }
// }

// // 라디안을 도로 변환하는 함수
// double radianToDegree(double radian) {
//     return radian * (180.0 / M_PI);
// }

// void PoseEstimator::calculatePose() {
//     while (running) {
//         float dt = 0.01f;  // 주기

//         // IMU 데이터가 유효하지 않으면 마지막 유효 데이터를 그대로 사용하거나 기본값 유지
//         if (imuAccel.hasNaN() || imuGyro.hasNaN()) {
//             std::cerr << "Invalid IMU data detected, using last valid data" << std::endl;
//             imuAccel.setZero();  // 기본값 사용 (정지 상태로 가정)
//             imuGyro.setZero();   // 자이로도 0으로 설정
//         }

//         ekf.predict(imuAccel, imuGyro, dt);  // EKF 예측 단계
//         ekf.update(gpsPos, gpsVel);  // EKF 업데이트 단계

//         // EKF의 상태 벡터를 currentState에 저장하여 업데이트
//         {
//             std::lock_guard<std::mutex> lock(poseMutex);  // 뮤텍스 사용
//             currentState = ekf.getState();  // EKF의 상태를 가져와 currentState에 저장
//         }

//         std::this_thread::sleep_for(std::chrono::milliseconds(10));  // 10ms 주기
//     }
// }

// void PoseEstimator::processIMU() {
//     while (running) {
//         IMUData imuData = readIMU();  // IMU 데이터 읽기
//         Eigen::Vector3f newAccel = Eigen::Vector3f(imuData.accelX, imuData.accelY, imuData.accelZ);
//         Eigen::Vector3f newGyro = Eigen::Vector3f(imuData.gyroX, imuData.gyroY, imuData.gyroZ);

//         // IMU 데이터가 유효하지 않으면 마지막 유효 데이터를 그대로 사용
//         if (newAccel.hasNaN() || newGyro.hasNaN()) {
//             std::cerr << "Invalid IMU data, keeping last valid data" << std::endl;
//             // 마지막 유효 데이터를 유지 (imuAccel과 imuGyro는 그대로)
//         } else {
//             imuAccel = newAccel;  // 새로운 유효한 IMU 데이터로 업데이트
//             imuGyro = newGyro;
//         }

//         std::this_thread::sleep_for(std::chrono::milliseconds(100));  // 10Hz 주기로 동작 (100ms 대기)
//     }
// }

// // 오일러 사용
// #include "pose_estimator.h"
// #include "ekf.h"
// #include "../ioss/imu_sensor.h"
// #include "../ioss/gps_sensor.h"
// #include <math.h>
// #include <iostream> // 출력 테스트를 위해 사용
// #include <iomanip>
// #include <thread>  // 스레드를 사용하기 위해 필요

// PoseEstimator::PoseEstimator() : ekf(), running(true) {
//     currentState = Eigen::VectorXf::Zero(9);  // 9차원의 상태 벡터 초기화
//     imuAccel = Eigen::Vector3f::Zero();  // IMU 가속도 초기화
//     imuGyro = Eigen::Vector3f::Zero();   // IMU 자이로 초기화
//     gpsPos = Eigen::Vector3f::Zero();    // GPS 위치 초기화
//     gpsVel = Eigen::Vector3f::Zero();    // GPS 속도 초기화
    
//     imuThread = std::thread(&PoseEstimator::processIMU, this);  // IMU 스레드 생성
//     gpsThread = std::thread(&PoseEstimator::processGPS, this);  // GPS 스레드 생성
//     estimationThread = std::thread(&PoseEstimator::calculatePose, this);
// }

// PoseEstimator::~PoseEstimator() {
//     running = false;  // 스레드 종료 요청
//     if (estimationThread.joinable()) {
//         estimationThread.join();  // 스레드가 종료될 때까지 대기
//     }
//     if (imuThread.joinable()) {
//         imuThread.join();  // IMU 스레드 종료 대기
//     }
//     if (gpsThread.joinable()) {
//         gpsThread.join();  // GPS 스레드 종료 대기
//     }
// }

// // 라디안을 도로 변환하는 함수
// double radianToDegree(double radian) {
//     return radian * (180.0 / M_PI);
// }

// void PoseEstimator::calculatePose() {
//     while (running) {
//         float dt = 0.01f;  // 주기

//         // IMU 데이터가 유효하지 않으면 마지막 유효 데이터를 그대로 사용하거나 기본값 유지
//         if (imuAccel.hasNaN() || imuGyro.hasNaN()) {
//             std::cerr << "Invalid IMU data detected, using last valid data" << std::endl;
//             imuAccel.setZero();  // 기본값 사용 (정지 상태로 가정)
//             imuGyro.setZero();   // 자이로도 0으로 설정
//         }

//         ekf.predict(imuAccel, imuGyro, dt);  // EKF 예측 단계
//         ekf.update(gpsPos, gpsVel);  // EKF 업데이트 단계

//         // EKF의 상태 벡터를 currentState에 저장하여 업데이트
//         {
//             std::lock_guard<std::mutex> lock(poseMutex);  // 뮤텍스 사용
//             currentState = ekf.getState();  // EKF의 상태를 가져와 currentState에 저장
//         }

//         std::this_thread::sleep_for(std::chrono::milliseconds(10));  // 10ms 주기
//     }
// }

// void PoseEstimator::processIMU() {
//     while (running) {
//         IMUData imuData = readIMU();  // IMU 데이터 읽기
//         Eigen::Vector3f newAccel = Eigen::Vector3f(imuData.accelX, imuData.accelY, imuData.accelZ);
//         Eigen::Vector3f newGyro = Eigen::Vector3f(imuData.gyroX, imuData.gyroY, imuData.gyroZ);

//         // IMU 데이터가 유효하지 않으면 마지막 유효 데이터를 그대로 사용
//         if (newAccel.hasNaN() || newGyro.hasNaN()) {
//             std::cerr << "Invalid IMU data, keeping last valid data" << std::endl;
//             // 마지막 유효 데이터를 유지 (imuAccel과 imuGyro는 그대로)
//         } else {
//             imuAccel = newAccel;  // 새로운 유효한 IMU 데이터로 업데이트
//             imuGyro = newGyro;
//         }

//         std::this_thread::sleep_for(std::chrono::milliseconds(100));  // 10Hz 주기로 동작 (100ms 대기)
//     }
// }

// void PoseEstimator::processGPS() {
//     while (running) {
//         GPSData gpsData = readGPS();  // GPS 데이터 읽기
//         Eigen::Vector3f newPos = Eigen::Vector3f(gpsData.latitude / 1e7, gpsData.longitude / 1e7, gpsData.altitude / 1000.0f);
//         Eigen::Vector3f newVel = Eigen::Vector3f(gpsData.velocityX, gpsData.velocityY, gpsData.velocityZ);
//         // GPS 데이터가 유효하지 않으면 마지막 유효 데이터를 그대로 사용
//         if (newPos.hasNaN() || newVel.hasNaN()) {
//             std::cerr << "Invalid GPS data, keeping last valid data" << std::endl;
//             // 마지막 유효 데이터를 유지 (gpsPos와 gpsVel은 그대로)
//         } else {
//             gpsPos = newPos;  // 새로운 유효한 GPS 데이터로 업데이트
//             gpsVel = newVel;
//         }

//         std::this_thread::sleep_for(std::chrono::milliseconds(100));  // 10Hz 주기로 동작 (100ms 대기)
//     }
// }

// Eigen::VectorXf PoseEstimator::getPose() {
//     std::lock_guard<std::mutex> lock(poseMutex);  // 뮤텍스를 사용하여 데이터 동시 접근 방지
//     return currentState;  // 현재 상태 벡터 반환
// }


// 쿼터니언 사용
#include "pose_estimator.h"
#include "ekf.h"
#include "imu_sensor.h"
#include "gps_sensor.h"
#include <math.h>
#include <iostream>
#include <iomanip>
#include <thread>

Eigen::Vector3f quaternionToEuler(const Eigen::Quaternionf& q) {
    Eigen::Vector3f euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
    return euler * (180.0f / M_PI);
}

PoseEstimator::PoseEstimator() : ekf(), running(true) {
    currentState = Eigen::VectorXf::Zero(10);
    imuAccel = Eigen::Vector3f::Zero();
    imuGyro = Eigen::Vector3f::Zero();
    gpsPos = Eigen::Vector3f::Zero();
    gpsVel = Eigen::Vector3f::Zero();
    gyroOffset = Eigen::Vector3f::Zero();

    imuThread = std::thread(&PoseEstimator::processIMU, this);
    gpsThread = std::thread(&PoseEstimator::processGPS, this);
    estimationThread = std::thread(&PoseEstimator::calculatePose, this);
}

PoseEstimator::~PoseEstimator() {
    running = false;
    if (estimationThread.joinable()) {
        estimationThread.join();
    }
    if (imuThread.joinable()) {
        imuThread.join();
    }
    if (gpsThread.joinable()) {
        gpsThread.join();
    }
}

void PoseEstimator::calibrateGyro() {
    int sampleCount = 100;
    Eigen::Vector3f gyroSum = Eigen::Vector3f::Zero();
    for (int i = 0; i < sampleCount; ++i) {
        IMUData imuData = readIMU();
        gyroSum += Eigen::Vector3f(imuData.gyroX, imuData.gyroY, imuData.gyroZ);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    gyroOffset = gyroSum / sampleCount;
    isGyroCalibrated = true;
}

void PoseEstimator::calculatePose() {
    while (running) {
        float dt = 0.1f;

        if (imuAccel.hasNaN() || imuGyro.hasNaN()) {
            std::cerr << "Invalid IMU data detected, using last valid data" << std::endl;
            imuAccel.setZero();
            imuGyro.setZero();
        }

        ekf.predict(imuAccel, imuGyro, dt);
        ekf.update(gpsPos, gpsVel);

        {
            std::lock_guard<std::mutex> lock(poseMutex);
            currentState = ekf.getState();
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void PoseEstimator::processIMU() {
    if (!isGyroCalibrated) {
        calibrateGyro();
    }

    while (running) {
        IMUData imuData = readIMU();
        Eigen::Vector3f newAccel = Eigen::Vector3f(imuData.accelX, imuData.accelY, imuData.accelZ);
        Eigen::Vector3f newGyro = Eigen::Vector3f(imuData.gyroX, imuData.gyroY, imuData.gyroZ) - gyroOffset;

        if (newAccel.hasNaN() || newGyro.hasNaN()) {
            std::cerr << "Invalid IMU data, keeping last valid data" << std::endl;
        } else {
            imuAccel = newAccel;
            imuGyro = newGyro;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void PoseEstimator::processGPS() {
    while (running) {
        // 기존의 GPS 데이터 읽기 부분을 주석 처리합니다.
        /*
        GPSData gpsData = readGPS();
        Eigen::Vector3f newPos = Eigen::Vector3f(gpsData.latitude / 1e7, gpsData.longitude / 1e7, gpsData.altitude / 1000.0f);
        Eigen::Vector3f newVel = Eigen::Vector3f(gpsData.velocityX, gpsData.velocityY, gpsData.velocityZ);

        // GPS 데이터가 유효하지 않으면 마지막 유효 데이터를 그대로 사용
        if (newPos.hasNaN() || newVel.hasNaN()) {
            std::cerr << "Invalid GPS data, keeping last valid data" << std::endl;
        } else {
            gpsPos = newPos;  // 새로운 유효한 GPS 데이터로 업데이트
            gpsVel = newVel;
        }
        */

        // GPS 데이터 대신 0 값을 사용하여 EKF에 넣습니다.
        gpsPos = Eigen::Vector3f::Zero(); // GPS 위치를 0으로 설정
        gpsVel = Eigen::Vector3f::Zero(); // GPS 속도를 0으로 설정

        std::this_thread::sleep_for(std::chrono::milliseconds(100));  // 10Hz 주기로 동작 (100ms 대기)
    }
}

Eigen::VectorXf PoseEstimator::getPose() {
    std::lock_guard<std::mutex> lock(poseMutex);

    Eigen::VectorXf pose(9);
    pose.segment<3>(0) = currentState.segment<3>(0);
    pose.segment<3>(3) = currentState.segment<3>(3);

    Eigen::Quaternionf attitude(currentState(6), currentState(7), currentState(8), currentState(9));
    pose.segment<3>(6) = quaternionToEuler(attitude);
    
    return pose;
}