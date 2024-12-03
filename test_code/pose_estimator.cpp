// 쿼터니언 사용
#include "pose_estimator.h"
#include "ekf.h"
#include "imu_sensor.h"
#include "gps_sensor.h"
#include <math.h>
#include <iostream>
#include <iomanip>
#include <thread>
#include <mutex>

Eigen::Vector3f quaternionToEuler(const Eigen::Quaternionf& q) {
    float ysqr = q.y() * q.y();

    float t0 = +2.0f * (q.w() * q.x() + q.y() * q.z());
    float t1 = +1.0f - 2.0f * (q.x() * q.x() + ysqr);
    float roll = std::atan2(t0, t1);

    float t2 = +2.0f * (q.w() * q.y() - q.z() * q.x());
    t2 = std::clamp(t2, -1.0f, 1.0f);
    float pitch = std::asin(t2);

    float t3 = +2.0f * (q.w() * q.z() + q.x() * q.y());
    float t4 = +1.0f - 2.0f * (ysqr + q.z() * q.z());  
    float yaw = std::atan2(t3, t4);

    return Eigen::Vector3f(roll, pitch, yaw) * (180.0f / M_PI);
}

// PoseEstimator 생성자
PoseEstimator::PoseEstimator() : ekf(), running(true) {
    currentState = Eigen::VectorXf::Zero(16);
    imuAccel = Eigen::Vector3f::Zero();
    imuGyro = Eigen::Vector3f::Zero();
    imuMag = Eigen::Vector3f::Zero();
    gpsPos = Eigen::Vector3f::Zero();
    gpsVel = Eigen::Vector3f::Zero();
    gyroOffset = Eigen::Vector3f::Zero();

    imuThread = std::thread(&PoseEstimator::processIMU, this);
    gpsThread = std::thread(&PoseEstimator::processGPS, this);
    estimationThread = std::thread(&PoseEstimator::calculatePose, this);
}

// PoseEstimator 소멸자
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

// 자이로 캘리브레이션 함수
void PoseEstimator::calibrateGyro() {
    int sampleCount = 50;
    Eigen::Vector3f gyroSum = Eigen::Vector3f::Zero();
    for (int i = 0; i < sampleCount; ++i) {
        IMUData imuData = readIMU();
        gyroSum += Eigen::Vector3f(imuData.gyroX, imuData.gyroY, imuData.gyroZ);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    gyroOffset = gyroSum / sampleCount;
    isGyroCalibrated = true;
}

// 포즈 계산 함수
void PoseEstimator::calculatePose() {
    while (running) {
        float dt = 0.01f;

        // 유효하지 않은 IMU 데이터가 감지되면 대체 값으로 초기화
        if (imuAccel.hasNaN() || imuGyro.hasNaN() || imuMag.hasNaN()) {
            std::cerr << "Invalid IMU data detected, using last valid data" << std::endl;
            imuAccel.setZero();
            imuGyro.setZero();
            imuMag.setZero();
        }

        // EKF 예측 및 자기장 업데이트 단계 실행
        ekf.predict(imuAccel, imuGyro, dt);
        // ekf.updateWithMag(imuMag);
        ekf.updateWithGPS(gpsPos ,gpsVel);

        // 현재 상태를 보호된 상태로 업데이트
        {
            std::lock_guard<std::mutex> lock(poseMutex);
            currentState = ekf.getState();
        }

        // 계산 주기 설정 (100ms)
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

// IMU 데이터 처리 함수
void PoseEstimator::processIMU() {
    // if (!isGyroCalibrated) {
    //     calibrateGyro();  // 자이로 캘리브레이션 수행
    // }

    while (running) {
        IMUData imuData = readIMU();  // IMU 센서에서 데이터 읽기
        Eigen::Vector3f newAccel = Eigen::Vector3f(imuData.accelX, imuData.accelY, imuData.accelZ);
        Eigen::Vector3f newGyro = Eigen::Vector3f(imuData.gyroX, imuData.gyroY, imuData.gyroZ) - gyroOffset;
        Eigen::Vector3f newMag = Eigen::Vector3f(imuData.magX, imuData.magY, imuData.magZ);

        // 유효한 IMU 데이터인 경우에만 업데이트
        if (!newAccel.hasNaN() && !newGyro.hasNaN() && !newMag.hasNaN()) {
            {
                std::lock_guard<std::mutex> lock(poseMutex);  // 동기화 보호
                imuAccel = newAccel;
                imuGyro = newGyro;
                imuMag = newMag;
            }
        } else {
            std::cerr << "Invalid IMU data, keeping last valid data" << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));  // 주기 설정
    }
}

// GPS 데이터 처리 함수
void PoseEstimator::processGPS() {
    while (running) {
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
        // 테스트를 위한 GPS 데이터 생성
        // gpsPos = Eigen::Vector3f::Zero();
        // gpsVel = Eigen::Vector3f::Zero();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

// 현재 포즈를 얻는 함수
Eigen::VectorXf PoseEstimator::getPose() {
    std::lock_guard<std::mutex> lock(poseMutex);

    Eigen::VectorXf pose(9);  // 위치, 속도, 오일러 각을 포함한 9차원 벡터
    pose.segment<3>(0) = currentState.segment<3>(0);  // 위치 (x, y, z)
    pose.segment<3>(3) = currentState.segment<3>(3);  // 속도 (vx, vy, vz)

    // 쿼터니언에서 Roll, Pitch, Yaw 계산
    Eigen::Quaternionf attitude(currentState(6), currentState(7), currentState(8), currentState(9));
    attitude.normalize();  // 쿼터니언 정규화
    pose.segment<3>(6) = quaternionToEuler(attitude);  // 오일러 각 (Roll, Pitch, Yaw) 추가

    return pose;
}
