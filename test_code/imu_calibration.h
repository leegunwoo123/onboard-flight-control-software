// imu_calibration.h
#ifndef IMU_CALIBRATION_H
#define IMU_CALIBRATION_H

#include "imu_sensor.h"

struct IMUCalibrationData {
    float offsetX;
    float offsetY;
    float offsetZ;
    float offsetGyroX;
    float offsetGyroY;
    float offsetGyroZ;
    float stlRoll;
    float stlPitch;
};

IMUCalibrationData calibrateIMU();

#endif // IMU_CALIBRATION_H
