// imu_calibration.h
#ifndef IMU_CALIBRATION_H
#define IMU_CALIBRATION_H

#include "imu_sensor.h"

struct IMUCalibrationData {
    float offsetX;
    float offsetY;
    float offsetZ;
    float offsetroll;
    float offsetpitch;
    float offsetyaw;
    float offsetGyroX;
    float offsetGyroY;
    float offsetGyroZ;
    float roll_acc;
    float pitch_acc;
    float yaw;
    float accelX_calibrated;
    float accelY_calibrated;
    float accelZ_calibrated;
    float gyroX_calibrated;
    float gyroY_calibrated;
    float gyroZ_calibrated;
};

IMUCalibrationData calibrateIMU();

#endif // IMU_CALIBRATION_H