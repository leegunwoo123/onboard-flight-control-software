#include "rc_input.h"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <chrono>
#include <deque>
#include <thread>
#include <vector>

#define SBUS_FRAME_SIZE 35
#define START_BYTE 0x0F

static int serial_port;
static uint16_t channels[16];           // 16채널 값을 저장할 배열
static std::deque<uint8_t> data_buffer; // 최신 데이터를 저장할 버퍼

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

// RC 입력 초기화 함수
void initRC(const std::string& port, int baudRate) {
    // 올바르게 초기화되지 않았을 경우 반복적으로 시도
    while (true) {
        if (configureSerial(port, baudRate) == -1) {
            std::cerr << "Failed to initialize RC input. Retrying..." << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1)); // 1초 대기 후 재시도
            continue;
        }
        break;
    }
}

// 최신 RC 채널 값을 읽고 업데이트하는 함수
int readRCChannel(int channel) {
    if (channel < 1 || channel > 16) {
        std::cerr << "Invalid channel number: " << channel << std::endl;
        return -1;
    }

    // 시리얼 포트에서 데이터 읽어 버퍼에 추가
    uint8_t byte;
    while (true) {  // 데이터 처리가 완료될 때까지 계속 실행
        while (read(serial_port, &byte, 1) > 0) {
            data_buffer.push_back(byte);

            // 오래된 데이터를 삭제하여 버퍼 크기를 제한
            if (data_buffer.size() > SBUS_FRAME_SIZE * 10) {
                data_buffer.pop_front();
            }
        }

        // 버퍼에서 최신 프레임을 찾아 데이터 갱신
        while (data_buffer.size() >= SBUS_FRAME_SIZE) {
            // 버퍼에서 프레임 추출
            std::vector<uint8_t> frame(data_buffer.begin(), data_buffer.begin() + SBUS_FRAME_SIZE);

            // 시작 바이트 확인
            if (frame[0] != START_BYTE) {
                data_buffer.pop_front();
                continue;
            }

            // 체크섬 검증
            uint8_t xor_checksum = 0;
            for (int i = 1; i < SBUS_FRAME_SIZE - 1; ++i) {
                xor_checksum ^= frame[i];
            }

            if (xor_checksum != frame[SBUS_FRAME_SIZE - 1]) {
                data_buffer.pop_front(); // 잘못된 프레임을 버림
                continue;
            }

            // 유효한 프레임이면 채널 데이터 업데이트
            for (int i = 0; i < 16; ++i) {
                channels[i] = (frame[1 + i * 2] << 8) | frame[2 + i * 2];
            }

            // 프레임을 버퍼에서 제거
            data_buffer.erase(data_buffer.begin(), data_buffer.begin() + SBUS_FRAME_SIZE);
            break;
        }

        // 요청된 채널 값을 반환
        // usleep(100000); // 0.1초(100ms) 대기
        usleep(100); // 0.0001초 대기
        return channels[channel - 1];
    }
}
