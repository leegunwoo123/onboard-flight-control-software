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

#define BUFFER_SIZE 128    // 버퍼 크기를 128로 설정
#define COMMAND_SIZE 100   // 명령어 크기 정의
#define CRC_SIZE 4         // CRC 크기 정의

// 시그널 플래그
static int serial_port;
static long previous_timestamp = 0; // 이전 타임스탬프 저장 변수

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
    serial_port = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_port == -1) {
        perror("Failed to open serial port");
        return -1;
    }

    struct termios options;
    tcgetattr(serial_port, &options);
    cfsetispeed(&options, baudrate);
    cfsetospeed(&options, baudrate);
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag &= ~OPOST;
    tcsetattr(serial_port, TCSANOW, &options);

    return serial_port;
}

// IMU 초기화 함수
void initIMU(const std::string& port, int baudRate) {
    if (configureSerial(port, baudRate) == -1) {
        throw std::runtime_error("Unable to configure serial port");
    }
}

// IMU 데이터 요청 함수
void sendIMURequest() {
    char command[COMMAND_SIZE];
    snprintf(command, sizeof(command), "$VNRRG,20");
    unsigned short crc = calculateCRC((unsigned char *)command + 1, strlen(command) - 1);
    snprintf(command, sizeof(command), "$VNRRG,20*%04X\r\n", crc);
    write(serial_port, command, strlen(command));
}

// IMU 데이터 읽기 및 처리 함수
IMUData readIMU() {
    char buffer[BUFFER_SIZE];  // IMU 데이터 저장 버퍼 (128로 설정)
    int buffer_index = 0;
    IMUData imuData = {};

    while (true) {
        sendIMURequest();
        usleep(1000);  // 요청 간격 설정

        int bytes_read = read(serial_port, buffer + buffer_index, sizeof(buffer) - buffer_index - 1);
        if (bytes_read > 0) {
            buffer_index += bytes_read;
            buffer[buffer_index] = '\0';

            char* line_start = buffer;
            char* line_end;

            while ((line_end = strchr(line_start, '\n')) != NULL) {
                *line_end = '\0';

                if (strncmp(line_start, "$VNRRG", 6) == 0) {
                    char* end_of_data = strchr(line_start, '*');
                    if (end_of_data) {
                        *end_of_data = '\0';

                        std::vector<std::string> parts;
                        std::istringstream ss(line_start);
                        std::string token;

                        while (std::getline(ss, token, ',')) {
                            parts.push_back(token);
                        }

                        if (parts.size() >= 11) {
                            unsigned short received_crc = std::stoi(end_of_data + 1, nullptr, 16);
                            unsigned short calculated_crc = calculateCRC((unsigned char *)line_start + 1, strlen(line_start) - 1);

                            if (received_crc == calculated_crc) {
                                imuData.accelX = std::stof(parts[5]);
                                imuData.accelY = std::stof(parts[6]);
                                imuData.accelZ = std::stof(parts[7]);
                                imuData.gyroX = std::stof(parts[8]);
                                imuData.gyroY = std::stof(parts[9]);
                                imuData.gyroZ = std::stof(parts[10]);

                                struct timeval current_time;
                                gettimeofday(&current_time, NULL);
                                imuData.timestamp = (current_time.tv_sec * 1000.0) + (current_time.tv_usec / 1000.0);
                                imuData.elapsed_time = imuData.timestamp - previous_timestamp;
                                previous_timestamp = imuData.timestamp;

                                return imuData;
                            } else {
                                fprintf(stderr, "CRC mismatch: Received: %04X, Calculated: %04X\n", received_crc, calculated_crc);
                            }
                        } else {
                            fprintf(stderr, "Invalid data format\n");
                        }
                    }
                }

                // 다음 줄로 이동
                line_start = line_end + 1;
            }

            // 남은 데이터가 있으면 버퍼의 시작으로 이동
            if (line_start < buffer + buffer_index) {
                int remaining_bytes = buffer + buffer_index - line_start;
                memmove(buffer, line_start, remaining_bytes);
                buffer_index = remaining_bytes;
            } else {
                buffer_index = 0;
            }
        }
    }
}
