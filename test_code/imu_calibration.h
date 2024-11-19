// imu_calibration.h
#ifndef IMU_CALIBRATION_H
#define IMU_CALIBRATION_H

#include "imu_sensor.h"

struct IMUCalibrationData {
    float offsetX;
    float offsetY;
    float offsetZ;
};

IMUCalibrationData calibrateIMU();

#endif // IMU_CALIBRATION_H
