// #ifndef POSE_ESTIMATOR_H
// #define POSE_ESTIMATOR_H

// #include <Eigen/Dense>
// #include <thread>
// #include <atomic>
// #include <mutex>


// class PoseEstimator {
// public:
//     PoseEstimator();
//     ~PoseEstimator();
    
//     Eigen::VectorXf getPose();  // 현재 위치와 자세 값을 반환하는 함수
    
// private:
//     class EKF ekf;  // EKF 객체
    
//     std::thread estimationThread;  // 주기적으로 동작하는 쓰레드
//     std::atomic<bool> running;  // 쓰레드 상태 제어
    
//     Eigen::VectorXf currentState;  // 최신 상태 저장 변수
//     std::mutex poseMutex;  // 상태 보호를 위한 뮤텍스
    
//     void calculatePose();  // 상태 계산 함수 (쓰레드 내부에서 실행)
    
//     const std::chrono::milliseconds loopDuration = std::chrono::milliseconds(10);  // 10ms 주기
// };

// #endif

// #ifndef POSE_ESTIMATOR_H
// #define POSE_ESTIMATOR_H

// #include <Eigen/Dense>
// #include <thread>
// #include <atomic>
// #include <mutex>
// #include <chrono>
// #include "ekf.h"  // EKF 클래스 헤더 파일 포함
// #include "imu_sensor.h"  // IMU 센서 헤더 파일 포함
// #include "gps_sensor.h"  // GPS 센서 헤더 파일 포함

// class PoseEstimator {
// public:
//     PoseEstimator();
//     ~PoseEstimator();
    
//     Eigen::VectorXf getPose();  // 현재 위치와 자세 값을 반환하는 함수
    
// private:
//     EKF ekf;  // EKF 객체
    
//     std::thread estimationThread;  // 주기적으로 동작하는 쓰레드
//     std::atomic<bool> running;  // 쓰레드 상태 제어
    
//     Eigen::VectorXf currentState;  // 최신 상태 저장 변수
//     std::mutex poseMutex;  // 상태 보호를 위한 뮤텍스
    
//     void calculatePose();  // 상태 계산 함수 (쓰레드 내부에서 실행)
    
//     const std::chrono::milliseconds loopDuration = std::chrono::milliseconds(10);  // 10ms 주기
// };

// #endif // POSE_ESTIMATOR_H


// // 오일러 사용
// #ifndef POSE_ESTIMATOR_H
// #define POSE_ESTIMATOR_H

// #include <Eigen/Dense>
// #include <thread>
// #include <atomic>
// #include <mutex>
// #include <chrono>
// #include "ekf.h"  // EKF 클래스 헤더 파일 포함
// #include "imu_sensor.h"  // IMU 센서 헤더 파일 포함
// #include "gps_sensor.h"  // GPS 센서 헤더 파일 포함

// class PoseEstimator {
// public:
//     PoseEstimator();
//     ~PoseEstimator();
    
//     Eigen::VectorXf getPose();  // 현재 위치와 자세 값을 반환하는 함수
    
// private:
//     EKF ekf;  // EKF 객체
    
//     std::thread estimationThread;  // 주기적으로 동작하는 쓰레드
//     std::thread imuThread;  // IMU 데이터를 처리하는 쓰레드
//     std::thread gpsThread;  // GPS 데이터를 처리하는 쓰레드
//     std::atomic<bool> running;  // 쓰레드 상태 제어
    
//     Eigen::VectorXf currentState;  // 최신 상태 저장 변수
//     Eigen::Vector3f imuAccel;  // IMU 가속도 데이터
//     Eigen::Vector3f imuGyro;   // IMU 자이로 데이터
//     Eigen::Vector3f gpsPos;    // GPS 위치 데이터
//     Eigen::Vector3f gpsVel;    // GPS 속도 데이터
//     std::mutex poseMutex;  // 상태 보호를 위한 뮤텍스
    
//     void calculatePose();  // 상태 계산 함수 (쓰레드 내부에서 실행)
//     void processIMU();  // IMU 데이터를 처리하는 함수
//     void processGPS();  // GPS 데이터를 처리하는 함수
    
//     const std::chrono::milliseconds loopDuration = std::chrono::milliseconds(10);  // 10ms 주기
// };

// #endif // POSE_ESTIMATOR_H


// 쿼터니언 사용
#ifndef POSE_ESTIMATOR_H
#define POSE_ESTIMATOR_H

#include <Eigen/Dense>
#include <thread>
#include <atomic>
#include <mutex>
#include <chrono>
#include "ekf.h"
#include "imu_sensor.h"
#include "gps_sensor.h"

class PoseEstimator {
public:
    PoseEstimator();
    ~PoseEstimator();
    
    Eigen::VectorXf getPose();
    
private:
    EKF ekf;

    std::thread estimationThread;
    std::thread imuThread;
    std::thread gpsThread;
    std::atomic<bool> running;
    
    Eigen::VectorXf currentState;
    Eigen::Vector3f imuAccel;
    Eigen::Vector3f imuGyro;
    Eigen::Vector3f gpsPos;
    Eigen::Vector3f gpsVel;
    std::mutex poseMutex;

    Eigen::Vector3f gyroOffset;
    bool isGyroCalibrated = false;
    
    void calibrateGyro();
    void calculatePose();
    void processIMU();
    void processGPS();
    
    const std::chrono::milliseconds loopDuration = std::chrono::milliseconds(20);
};

#endif // POSE_ESTIMATOR_H