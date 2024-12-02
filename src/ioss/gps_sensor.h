// #ifndef GPS_SENSOR_H
// #define GPS_SENSOR_H

// #include <string>

// // GPSData 구조체 정의
// struct GPSData {
//     int32_t longitude;            // 경도
//     int32_t latitude;            // 위도
//     int32_t altitude;       // 고도
//     int32_t gSpeed;         // 지상 속도
//     uint8_t numSV;          // 위성 수
//     int32_t velocityX;      // NED 북 방향 속도 (mm/s)
//     int32_t velocityY;      // NED 동 방향 속도 (mm/s)
//     int32_t velocityZ;      // NED 하강 방향 속도 (mm/s)
// };

// // GPS 초기화 함수
// void initGPS(const std::string& port, int baudRate);

// // GPS 데이터를 읽는 함수
// GPSData readGPS();

// #endif

#ifndef GPS_SENSOR_H
#define GPS_SENSOR_H

#include <string>
#include <cstdint> // int64_t를 사용하기 위한 헤더

// GPSData 구조체 정의
struct GPSData {
    int64_t longitude;  // 경도 (1e-7 도 단위로 표현 가능)
    int64_t latitude;   // 위도 (1e-7 도 단위로 표현 가능)
    int64_t altitude;   // 고도 (더 큰 범위 지원)
    int64_t gSpeed;     // 지상 속도 (더 큰 범위 지원)
    uint8_t numSV;      // 위성 수
    int64_t velocityX;  // NED 북 방향 속도 (mm/s, 더 큰 범위 지원)
    int64_t velocityY;  // NED 동 방향 속도 (mm/s, 더 큰 범위 지원)
    int64_t velocityZ;  // NED 하강 방향 속도 (mm/s, 더 큰 범위 지원)
};

// GPS 초기화 함수
void initGPS(const char* port, int baudRate);

// GPS 데이터를 읽는 함수
GPSData readGPS();

#endif