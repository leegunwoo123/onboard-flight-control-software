#ifndef BAROMETER_SENSOR_H
#define BAROMETER_SENSOR_H

#include <string>
#include <signal.h>

// 기압 데이터를 저장하는 구조체
struct BarometerData {
    float pressure;
    float temperature;
};

// 기압 센서를 초기화하는 함수
void initBarometer(const std::string& port, int baudRate);

// 기압 데이터를 읽는 함수
BarometerData readBarometer();

#endif // BAROMETER_SENSOR_H
