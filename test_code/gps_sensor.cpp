#include "gps_sensor.h"
#include <iostream>
#include <vector>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

using namespace std;

int serialPort = -1;  // 전역 변수로 시리얼 포트 관리

// 시리얼 포트 설정 함수
void initGPS(const char* port, int baudRate) {
    serialPort = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
    if (serialPort == -1) {
        perror("Unable to open serial port");
        return;
    }

    struct termios options;
    tcgetattr(serialPort, &options);
    cfsetispeed(&options, baudRate); // Baud rate 설정
    cfsetospeed(&options, baudRate);
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB; // 패리티 비트 없음
    options.c_cflag &= ~CSTOPB; // 1 스톱 비트
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8; // 8 비트 데이터 비트
    options.c_cflag &= ~CRTSCTS; // RTS/CTS 흐름 제어 없음
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // 캐노닉 모드 해제
    options.c_iflag &= ~(IXON | IXOFF | IXANY); // 소프트웨어 흐름 제어 없음
    options.c_oflag &= ~OPOST; // 원시 모드

    tcsetattr(serialPort, TCSANOW, &options);
}

// GPS 데이터 파싱 함수
GPSData parseGpsData(const vector<uint8_t>& data) {
    GPSData gpsData = {};

    if (data.size() < 100) {
        return gpsData; // 데이터가 부족하면 빈 값 반환
    }

    uint8_t msgId = data[3];
    uint16_t length = (data[5] << 8) | data[4]; // Length 필드 (2바이트)

    if (msgId == 0x07) {
        gpsData.numSV = data[29];
        gpsData.longitude = (data[33] << 24) | (data[32] << 16) | (data[31] << 8) | data[30];
        gpsData.latitude = (data[37] << 24) | (data[36] << 16) | (data[35] << 8) | data[34];
        gpsData.altitude = (data[41] << 24) | (data[40] << 16) | (data[39] << 8) | data[38];
        gpsData.gSpeed = (data[69] << 24) | (data[68] << 16) | (data[67] << 8) | data[66];
        gpsData.velocityX = (data[57] << 24) | (data[56] << 16) | (data[55] << 8) | data[54];
        gpsData.velocityY = (data[61] << 24) | (data[60] << 16) | (data[59] << 8) | data[58];
        gpsData.velocityZ = (data[65] << 24) | (data[64] << 16) | (data[63] << 8) | data[62];
    }

    return gpsData;
}

// GPS 데이터를 읽는 함수
GPSData readGPS() {
    vector<uint8_t> receivedData;
    uint8_t buffer[1024];
    GPSData gpsData = {};
    bool flag = false;  // 파싱이 성공했는지 확인하는 플래그

    while (!flag) {
        int bytesRead = read(serialPort, buffer, sizeof(buffer));
        if (bytesRead > 0) {
            receivedData.insert(receivedData.end(), buffer, buffer + bytesRead);

            // 메시지 파싱을 위한 루프
            while (true) {
                if (receivedData.size() < 5) {
                    break; // 수신된 데이터가 충분하지 않으면 대기
                }

                // 헤더 찾기
                if (receivedData[0] == 0xB5 && receivedData[1] == 0x62 && receivedData[3] == 0x07) {
                    uint16_t length = (receivedData[5] << 8) | receivedData[4]; // Length 필드 (2바이트)
                    uint16_t totalMessageLength = length + 6 + 2; // Length + Header + Checksum

                    if (receivedData.size() >= totalMessageLength) {
                        gpsData = parseGpsData(vector<uint8_t>(receivedData.begin(), receivedData.begin() + totalMessageLength));
                        flag = true;  // 올바른 값이 파싱되면 플래그를 true로 설정

                        // 처리 후 남은 데이터 관리
                        receivedData.erase(receivedData.begin(), receivedData.begin() + totalMessageLength);
                    } else {
                        break; // 전체 메시지가 아직 수신되지 않음
                    }
                } else {
                    // 헤더가 잘못된 경우 첫 번째 바이트를 삭제
                    receivedData.erase(receivedData.begin());
                }
            }
        } else {
            // 수신된 데이터가 없을 때 usleep으로 대기 (예: 100ms 대기)
            usleep(100000);  // 100,000 마이크로초 = 100ms
        }
    }

    // flag가 true일 때 gpsData를 반환
    return gpsData;
}