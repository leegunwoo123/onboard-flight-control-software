#ifndef IMU_SENSOR_H
#define IMU_SENSOR_H

#include <string>   
#include <signal.h> 

// IMU 데이터를 저장하는 구조체
struct IMUData {
    float accelX;    // X축 가속도
    float accelY;    // Y축 가속도
    float accelZ;    // Z축 가속도
    float gyroX;     // X축 자이로스코프
    float gyroY;     // Y축 자이로스코프
    float gyroZ;     // Z축 자이로스코프
    float magX;      // X축 자기장
    float magY;      // Y축 자기장
    float magZ;      // Z축 자기장
    double timestamp;    // 타임스탬프
    double elapsed_time; // 경과 시간
};

void initIMU(const std::string& port, int baudRate);
IMUData readIMU();

#endif // IMU_SENSOR_H