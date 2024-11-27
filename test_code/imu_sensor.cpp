// #include "imu_sensor.h"
// #include <stdio.h>
// #include <string.h>
// #include <stdlib.h>
// #include <unistd.h>
// #include <fcntl.h>
// #include <termios.h>
// #include <sys/time.h>
// #include <iostream>
// #include <sstream>
// #include <vector>
// #include <stdexcept>
// #include <csignal>

// #define BUFFER_SIZE 1024  // 버퍼 크기 정의
// #define COMMAND_SIZE 100  // 명령어 크기 정의
// #define CRC_SIZE 4        // CRC 크기 정의

// // 시그널 플래그
// static int serial_port;
// static long previous_timestamp = 0; // 이전 타임스탬프 저장 변수

// // CRC 계산 함수 (데이터 유효성 검증에 사용)
// static unsigned short calculateCRC(const unsigned char* data, unsigned int length) {
//     unsigned short crc = 0;
//     for (unsigned int i = 0; i < length; i++) {
//         crc = (unsigned char)(crc >> 8) | (crc << 8);
//         crc ^= data[i];
//         crc ^= (unsigned char)(crc & 0xff) >> 4;
//         crc ^= crc << 12;
//         crc ^= (crc & 0x00ff) << 5;
//     }
//     return crc;
// }

// // 시리얼 포트 설정 함수
// static int configureSerial(const std::string& port, int baudrate) {
//     serial_port = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);  // 포트 열기
//     if (serial_port == -1) {  // 실패 시 에러 메시지 출력
//         perror("Failed to open serial port");
//         return -1;
//     }

//     struct termios options;
//     tcgetattr(serial_port, &options); // 현재 포트 설정 가져오기
//     cfsetispeed(&options, baudrate);  // 입력 보드레이트 설정
//     cfsetospeed(&options, baudrate);  // 출력 보드레이트 설정
//     options.c_cflag |= (CLOCAL | CREAD);  // 포트 활성화
//     options.c_cflag &= ~PARENB;           // 패리티 비트 비활성화
//     options.c_cflag &= ~CSTOPB;           // 스톱 비트 비활성화
//     options.c_cflag &= ~CSIZE;            // 데이터 크기 설정
//     options.c_cflag |= CS8;               // 8비트 데이터 설정
//     options.c_iflag &= ~(IXON | IXOFF | IXANY); // 소프트웨어 흐름 제어 비활성화
//     options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // 표준 모드 비활성화
//     options.c_oflag &= ~OPOST;            // 출력 처리 비활성화
//     tcsetattr(serial_port, TCSANOW, &options);  // 설정 즉시 적용

//     return serial_port;
// }

// // IMU 초기화 함수
// void initIMU(const std::string& port, int baudRate) {
//     // 시리얼 포트 설정 실패 시 예외 발생
//     if (configureSerial(port, baudRate) == -1) {
//         throw std::runtime_error("Unable to configure serial port");
//     }
// }

// // IMU 데이터 요청 함수
// void sendIMURequest() {
//     char command[COMMAND_SIZE];
//     snprintf(command, sizeof(command), "$VNRRG,20");  // IMU 데이터 요청 명령어
//     unsigned short crc = calculateCRC((unsigned char *)command + 1, strlen(command) - 1); // CRC 계산
//     snprintf(command, sizeof(command), "$VNRRG,20*%04X\r\n", crc); // 명령어 포맷팅
//     write(serial_port, command, strlen(command)); // 시리얼 포트에 쓰기
// }

// IMUData readIMU() {
//     char response[256];  // 응답 데이터를 저장할 배열
//     char buffer[BUFFER_SIZE];  // IMU 데이터 저장 버퍼
//     int buffer_index = 0;  // 버퍼 인덱스 초기화
//     IMUData imuData = {};  // IMU 데이터 구조체 초기화

//     // 원하는 데이터가 올 때까지 대기
//     bool data_received = false; // 데이터 수신 여부 플래그
//     while (!data_received) {

//         sendIMURequest(); // IMU 데이터 요청
//         usleep(10000); // 100ms 대기 (데이터 반환 주기 설정)

//         int bytes_read = read(serial_port, response, sizeof(response) - 1);
//         if (bytes_read > 0) {
//             response[bytes_read] = '\0';  // 응답 데이터 문자열 종료

//             // 버퍼 오버플로우 방지
//             if (buffer_index + bytes_read < BUFFER_SIZE) {
//                 memcpy(buffer + buffer_index, response, bytes_read); // 버퍼에 데이터 복사
//                 buffer_index += bytes_read;  // 인덱스 증가
//             } else {
//                 //fprintf(stderr, "Buffer overflow!\n"); // 버퍼 오버플로우 에러 메시지 출력
//                 buffer_index = 0;  // 인덱스 초기화
//             }

//             // 한 줄씩 데이터 처리
//             buffer[buffer_index] = '\0';  // 버퍼 종료 문자 추가
//             char* line_start = buffer;
//             char* line_end;
//             char* last_line = NULL;

//             // 줄 단위로 데이터를 분리하고 마지막 줄 찾기
//             while ((line_end = strchr(line_start, '\n')) != NULL) {
//                 *line_end = '\0';  // 줄 종료 문자 삽입
//                 if (strncmp(line_start, "$VNRRG", 6) == 0) {  // $VNRRG로 시작하는 줄 찾기
//                     last_line = line_start;  // 마지막 줄로 저장
//                     data_received = true; // 데이터 수신 플래그 설정
//                 }
//                 line_start = line_end + 1;
//             }

//             // 유효한 데이터가 있으면 파싱 시작
//             if (last_line) {
//                 char* end_of_data = strchr(last_line, '*'); // CRC 확인을 위해 '*' 문자 찾기
//                 if (end_of_data) {
//                     *end_of_data = '\0';  // CRC 제외한 데이터로 자르기

//                     // ','를 기준으로 데이터를 분리
//                     std::vector<std::string> parts;
//                     std::istringstream ss(last_line);
//                     std::string token;

//                     while (std::getline(ss, token, ',')) {
//                         parts.push_back(token);
//                     }

//                     // 10개의 데이터가 존재할 때만 파싱 진행
//                     if (parts.size() >= 11) {
//                         // CRC 계산
//                         unsigned short received_crc = std::stoi(end_of_data + 1, nullptr, 16); // 수신한 CRC
//                         unsigned short calculated_crc = calculateCRC((unsigned char *)last_line + 1, strlen(last_line) - 1); // 계산된 CRC

//                         if (received_crc == calculated_crc) {  // CRC가 맞으면
//                             imuData.accelX = std::stof(parts[5]);  // X축 가속도
//                             imuData.accelY = std::stof(parts[6]);  // Y축 가속도
//                             imuData.accelZ = std::stof(parts[7]);  // Z축 가속도
//                             imuData.gyroX = std::stof(parts[8]);   // X축 자이로스코프
//                             imuData.gyroY = std::stof(parts[9]);   // Y축 자이로스코프
//                             imuData.gyroZ = std::stof(parts[10]);  // Z축 자이로스코프

//                             // 타임스탬프 계산
//                             struct timeval current_time;
//                             gettimeofday(&current_time, NULL);  // 현재 시간 가져오기
//                             imuData.timestamp = (current_time.tv_sec * 1000.0) + (current_time.tv_usec / 1000.0);  // 밀리초 단위 타임스탬프 계산
//                             imuData.elapsed_time = imuData.timestamp - previous_timestamp;  // 경과 시간 계산
//                             previous_timestamp = imuData.timestamp;  // 이전 타임스탬프 갱신
//                         } else {
//                             fprintf(stderr, "CRC mismatch: Received: %04X, Calculated: %04X\n", received_crc, calculated_crc); // CRC 불일치 에러 메시지 출력
//                         }
//                     } else {
//                         fprintf(stderr, "Invalid data format\n"); // 잘못된 데이터 형식 에러 메시지 출력
//                     }
//                 }
//             }
//         }
//     }

//     return imuData;  // IMU 데이터 반환
// }

#include "imu_sensor.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/time.h>
#include <iostream>
#include <sstream>
#include <vector>
#include <stdexcept>
#include <csignal>
#include <Eigen/Dense>  // Eigen 라이브러리 포함 (옵션)

// 버퍼 크기 정의
#define BUFFER_SIZE 1024  // 버퍼 크기 정의
#define COMMAND_SIZE 100  // 명령어 크기 정의
#define CRC_SIZE 4        // CRC 크기 정의

// 시그널 플래그
static int serial_port;
static long previous_timestamp = 0; // 이전 타임스탬프 저장 변수

// IMU 데이터 구조체 정의 (imu_sensor.h에 정의되어 있다고 가정)
struct IMUData {
    float accelX;        // X축 가속도
    float accelY;        // Y축 가속도
    float accelZ;        // Z축 가속도
    float gyroX;         // X축 자이로스코프
    float gyroY;         // Y축 자이로스코프
    float gyroZ;         // Z축 자이로스코프
    double timestamp;    // 타임스탬프
    double elapsed_time; // 경과 시간
};

// CRC 계산 함수 (데이터 유효성 검증에 사용)
static unsigned short calculateCRC(const unsigned char* data, unsigned int length) {
    unsigned short crc = 0;
    for (unsigned int i = 0; i < length; i++) {
        crc = (unsigned char)(crc >> 8) | (crc << 8);
        crc ^= data[i];
        crc ^= (unsigned char)(crc & 0xff) >> 4;
        crc ^= crc << 12;
        crc ^= (crc & 0x00ff) << 5;
    }
    return crc;
}

// 시리얼 포트 설정 함수
static int configureSerial(const std::string& port, int baudrate) {
    serial_port = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);  // 포트 열기
    if (serial_port == -1) {  // 실패 시 에러 메시지 출력
        perror("시리얼 포트를 여는 데 실패했습니다");
        return -1;
    }

    struct termios options;
    tcgetattr(serial_port, &options); // 현재 포트 설정 가져오기
    cfsetispeed(&options, baudrate);  // 입력 보드레이트 설정
    cfsetospeed(&options, baudrate);  // 출력 보드레이트 설정
    options.c_cflag |= (CLOCAL | CREAD);  // 포트 활성화
    options.c_cflag &= ~PARENB;           // 패리티 비트 비활성화
    options.c_cflag &= ~CSTOPB;           // 스톱 비트 비활성화
    options.c_cflag &= ~CSIZE;            // 데이터 크기 설정
    options.c_cflag |= CS8;               // 8비트 데이터 설정
    options.c_iflag &= ~(IXON | IXOFF | IXANY); // 소프트웨어 흐름 제어 비활성화
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // 표준 모드 비활성화
    options.c_oflag &= ~OPOST;            // 출력 처리 비활성화
    tcsetattr(serial_port, TCSANOW, &options);  // 설정 즉시 적용

    return serial_port;
}

// IMU 초기화 함수
void initIMU(const std::string& port, int baudRate) {
    // 시리얼 포트 설정 실패 시 예외 발생
    if (configureSerial(port, baudRate) == -1) {
        throw std::runtime_error("시리얼 포트를 설정할 수 없습니다");
    }
}

// IMU 데이터 요청 함수
void sendIMURequest() {
    char command[COMMAND_SIZE];
    snprintf(command, sizeof(command), "$VNRRG,20");  // IMU 데이터 요청 명령어
    unsigned short crc = calculateCRC((unsigned char *)command + 1, strlen(command) - 1); // CRC 계산
    snprintf(command, sizeof(command), "$VNRRG,20*%04X\r\n", crc); // 명령어 포맷팅
    write(serial_port, command, strlen(command)); // 시리얼 포트에 쓰기
}

// NED 좌표계로 변환하는 함수
IMUData convertToNED(const IMUData& original) {
    IMUData ned = original; // 원본 데이터를 복사

    // 원래 IMU 좌표계 가정:
    // X: 전방, Y: 좌측, Z: 상방
    // NED 좌표계:
    // X: 북, Y: 동, Z: 하

    ned.accelY = -original.accelY;     // 동(E)은 원래 Y의 반대
    ned.accelZ = -original.accelZ;     // 하(D)은 원래 Z의 반대

    ned.gyroY = -original.gyroY;       // 동(E)은 원래 Y의 반대
    ned.gyroZ = -original.gyroZ;       // 하(D)은 원래 Z의 반대

    return ned;
}

// Eigen을 사용한 NED 변환 함수 (옵션)
IMUData convertToNED_Eigen(IMUData& original) {
    // 회전 행렬을 사용하여 변환
    Eigen::Matrix3f rotation;
    rotation << 1,  0,  0,
               0, -1,  0,
               0,  0, -1;

    // 가속도와 자이로 데이터를 벡터로 생성
    Eigen::Vector3f accel(original.accelX, original.accelY, original.accelZ);
    Eigen::Vector3f gyro(original.gyroX, original.gyroY, original.gyroZ);

    // 회전 적용
    Eigen::Vector3f accel_ned = rotation * accel;
    Eigen::Vector3f gyro_ned = rotation * gyro;

    // 변환된 데이터 할당
    original.accelX = accel_ned.x();
    original.accelY = accel_ned.y();
    original.accelZ = accel_ned.z();

    original.gyroX = gyro_ned.x();
    original.gyroY = gyro_ned.y();
    original.gyroZ = gyro_ned.z();

    return original;
}

// IMU 데이터 읽기 함수
IMUData readIMU() {
    char response[256];  // 응답 데이터를 저장할 배열
    char buffer[BUFFER_SIZE];  // IMU 데이터 저장 버퍼
    int buffer_index = 0;  // 버퍼 인덱스 초기화
    IMUData imuData = {};  // IMU 데이터 구조체 초기화

    bool data_received = false; // 데이터 수신 여부 플래그
    while (!data_received) {

        sendIMURequest(); // IMU 데이터 요청
        usleep(10000); // 10ms 대기 (데이터 반환 주기 설정)

        int bytes_read = read(serial_port, response, sizeof(response) - 1);
        if (bytes_read > 0) {
            response[bytes_read] = '\0';  // 응답 데이터 문자열 종료

            // 버퍼 오버플로우 방지
            if (buffer_index + bytes_read < BUFFER_SIZE) {
                memcpy(buffer + buffer_index, response, bytes_read); // 버퍼에 데이터 복사
                buffer_index += bytes_read;  // 인덱스 증가
            } else {
                // 버퍼 오버플로우 시 인덱스 초기화
                buffer_index = 0;
            }

            buffer[buffer_index] = '\0';  // 버퍼 종료 문자 추가
            char* line_start = buffer;
            char* line_end;
            char* last_line = NULL;

            // 줄 단위로 데이터를 분리하고 마지막 줄 찾기
            while ((line_end = strchr(line_start, '\n')) != NULL) {
                *line_end = '\0';  // 줄 종료 문자 삽입
                if (strncmp(line_start, "$VNRRG", 6) == 0) {  // $VNRRG로 시작하는 줄 찾기
                    last_line = line_start;  // 마지막 줄로 저장
                    data_received = true; // 데이터 수신 플래그 설정
                }
                line_start = line_end + 1;
            }

            // 유효한 데이터가 있으면 파싱 시작
            if (last_line) {
                char* end_of_data = strchr(last_line, '*'); // CRC 확인을 위해 '*' 문자 찾기
                if (end_of_data) {
                    *end_of_data = '\0';  // CRC 제외한 데이터로 자르기

                    // ','를 기준으로 데이터를 분리
                    std::vector<std::string> parts;
                    std::istringstream ss(last_line);
                    std::string token;

                    while (std::getline(ss, token, ',')) {
                        parts.push_back(token);
                    }

                    // 11개의 데이터가 존재할 때만 파싱 진행
                    if (parts.size() >= 11) {
                        // CRC 계산
                        unsigned short received_crc = std::stoi(end_of_data + 1, nullptr, 16); // 수신한 CRC
                        unsigned short calculated_crc = calculateCRC((unsigned char *)last_line + 1, strlen(last_line) - 1); // 계산된 CRC

                        if (received_crc == calculated_crc) {  // CRC가 맞으면
                            imuData.accelX = std::stof(parts[5]);  // X축 가속도
                            imuData.accelY = std::stof(parts[6]);  // Y축 가속도
                            imuData.accelZ = std::stof(parts[7]);  // Z축 가속도
                            imuData.gyroX = std::stof(parts[8]);   // X축 자이로스코프
                            imuData.gyroY = std::stof(parts[9]);   // Y축 자이로스코프
                            imuData.gyroZ = std::stof(parts[10]);  // Z축 자이로스코프

                            // 타임스탬프 계산
                            struct timeval current_time;
                            gettimeofday(&current_time, NULL);  // 현재 시간 가져오기
                            imuData.timestamp = (current_time.tv_sec * 1000.0) + (current_time.tv_usec / 1000.0);  // 밀리초 단위 타임스탬프 계산
                            imuData.elapsed_time = imuData.timestamp - previous_timestamp;  // 경과 시간 계산
                            previous_timestamp = imuData.timestamp;  // 이전 타임스탬프 갱신
                        } else {
                            fprintf(stderr, "CRC 불일치: 수신한 CRC: %04X, 계산된 CRC: %04X\n", received_crc, calculated_crc); // CRC 불일치 에러 메시지 출력
                        }
                    } else {
                        fprintf(stderr, "잘못된 데이터 형식\n"); // 잘못된 데이터 형식 에러 메시지 출력
                    }
                }
            }
        }
    }

    // NED 좌표계로 변환
    imuData = convertToNED(imuData);
    // 또는 Eigen을 사용하는 경우:
    // imuData = convertToNED_Eigen(imuData);

    return imuData;  // NED 변환된 IMU 데이터 반환
}
