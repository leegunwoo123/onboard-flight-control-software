// 쿼터니언 사용
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
    Eigen::Vector3f imuMag;
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