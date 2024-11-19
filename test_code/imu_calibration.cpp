#include "imu_calibration.h"
#include "imu_sensor.h" // 왜 안 들어가 있었지?
#include <iostream>
#include <vector>
#include <numeric>  // accumulate 사용을 위한 헤더 추가
#include <unistd.h> // usleep 사용을 위한 헤더 추가
#include <termios.h> // B115200 정의를 위한 헤더 추가
#include <chrono> // 시간 측정을 위한 헤더 추가

// 캘리브레이션 함수
// IMUCalibrationData calibrateIMU() {
//     initIMU("/dev/ttyUSB0", B115200);
//     std::cerr << "cali1" << std::endl; // 디버그 메시지
//     IMUCalibrationData calibrationData;
//     IMUData imuData;
//     std::vector<float> accelX_samples, accelY_samples, accelZ_samples;

//     int sample_count = 0;
//     const int target_samples = 1000;

//     // 1000개의 IMU 데이터 샘플 수집
//     while (sample_count < target_samples) {
//         imuData = readIMU();  // IMU 데이터 읽기

//         // 각 축의 가속도 데이터를 샘플로 수집
//         accelX_samples.push_back(imuData.accelX);
//         accelY_samples.push_back(imuData.accelY);
//         accelZ_samples.push_back(imuData.accelZ);

//         usleep(5000);  // 샘플 간 시간 간격 설정 (5ms)
//         sample_count++;
//     }
IMUCalibrationData calibrateIMU() {
    // initIMU("/dev/ttyUSB0", B115200);
    std::cerr << "cali1" << std::endl; // 디버그 메시지
    IMUCalibrationData calibrationData;
    // IMUData imuData;
    std::vector<float> accelX_samples, accelY_samples, accelZ_samples;

    // 시작 시간 설정
    auto start_time = std::chrono::high_resolution_clock::now();
    int sample_count = 0;

    // 3초 동안 IMU 데이터 샘플 수집
    while (true) {
        auto current_time = std::chrono::high_resolution_clock::now();
        auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
        
        if (elapsed_time >= 3) break;
        IMUData imuData = readIMU();  // IMU 데이터 읽기
        // 각 축의 가속도 데이터를 샘플로 수집
        accelX_samples.push_back(imuData.accelX);
        accelY_samples.push_back(imuData.accelY);
        accelZ_samples.push_back(imuData.accelZ);
 
        usleep(100000);  // 샘플 간 시간 간격 설정 (10ms)
        sample_count++;
    }

    // 평균값 계산하여 오프셋 설정
    calibrationData.offsetX = std::accumulate(accelX_samples.begin(), accelX_samples.end(), 0.0f) / sample_count;
    calibrationData.offsetY = std::accumulate(accelY_samples.begin(), accelY_samples.end(), 0.0f) / sample_count;
    calibrationData.offsetZ = std::accumulate(accelZ_samples.begin(), accelZ_samples.end(), 0.0f) / sample_count;

    std::cout << "Calibration Complete:\n";
    std::cout << "Offset X: " << calibrationData.offsetX << "\n";
    std::cout << "Offset Y: " << calibrationData.offsetY << "\n";
    std::cout << "Offset Z: " << calibrationData.offsetZ << "\n";
    return calibrationData;
}


// #include "imu_sensor.h"
// #include <iostream>
// #include <vector>
// #include <numeric>  // accumulate 사용을 위한 헤더 추가
// #include <unistd.h> // usleep 사용을 위한 헤더 추가
// #include <termios.h> // B115200 정의를 위한 헤더 추가
// #include <chrono> // 시간 측정을 위한 헤더 추가

// struct IMUCalibrationData {
//     float offsetX = 0;
//     float offsetY = 0;
//     float offsetZ = 0;
// };

// // 캘리브레이션 함수
// IMUCalibrationData calibrateIMU() {
//     initIMU("/dev/ttyUSB0", B115200);
//     std::cerr << "cali1" << std::endl; // 디버그 메시지
//     IMUCalibrationData calibrationData;
//     IMUData imuData;
//     std::vector<float> accelX_samples, accelY_samples, accelZ_samples;

//     // 시작 시간 설정
//     auto start_time = std::chrono::high_resolution_clock::now();
//     int sample_count = 0;

//     // 5초 동안 IMU 데이터 샘플 수집
//     while (true) {
//         auto current_time = std::chrono::high_resolution_clock::now();
//         auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
        
//         if (elapsed_time >= 5) break;

//         imuData = readIMU();  // IMU 데이터 읽기

//         // 각 축의 가속도 데이터를 샘플로 수집
//         accelX_samples.push_back(imuData.accelX);
//         accelY_samples.push_back(imuData.accelY);
//         accelZ_samples.push_back(imuData.accelZ);

//         usleep(5000);  // 샘플 간 시간 간격 설정 (5ms)
//         sample_count++;
//     }

//     // 평균값 계산하여 오프셋 설정
//     calibrationData.offsetX = std::accumulate(accelX_samples.begin(), accelX_samples.end(), 0.0f) / sample_count;
//     calibrationData.offsetY = std::accumulate(accelY_samples.begin(), accelY_samples.end(), 0.0f) / sample_count;
//     calibrationData.offsetZ = std::accumulate(accelZ_samples.begin(), accelZ_samples.end(), 0.0f) / sample_count;

//     std::cout << "Calibration Complete:\n";
//     std::cout << "Offset X: " << calibrationData.offsetX << "\n";
//     std::cout << "Offset Y: " << calibrationData.offsetY << "\n";
//     std::cout << "Offset Z: " << calibrationData.offsetZ << "\n";

//     return calibrationData;
// }

// int main() {
//     try {
//         // 시리얼 포트 및 통신 속도 설정
//         initIMU("/dev/ttyUSB0", B115200);
//         std::cerr << "cali3" << std::endl; // 디버그 메시지

//         // 캘리브레이션 수행
//         IMUCalibrationData calibrationData = calibrateIMU();

//         // 캘리브레이션 완료 후 데이터 출력
//         std::cout << "IMU Calibration offsets are set." << std::endl;
//     } catch (const std::exception &e) {
//         std::cerr << "Error: " << e.what() << std::endl;
//         return -1;
//     }

//     return 0;
// }