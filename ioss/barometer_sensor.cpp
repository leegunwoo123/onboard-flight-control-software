// UART 방식으로 시도
#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>

int main() {
    int serial_fd = open("/dev/ttyAMA0", O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_fd == -1) {
        std::cerr << "UART 포트를 열 수 없습니다." << std::endl;
        return -1;
    }

    struct termios options;
    tcgetattr(serial_fd, &options);
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    tcsetattr(serial_fd, TCSANOW, &options);

    unsigned char buffer[256];
    while (true) {
        int bytes_read = read(serial_fd, &buffer, sizeof(buffer));
        if (bytes_read > 0) {
            std::cout << "Raw data (hex): ";
            for (int i = 0; i < bytes_read; i++) {
                // 받은 데이터를 16진수로 출력
                std::cout << std::hex << static_cast<int>(buffer[i]) << " ";
            }
            std::cout << std::endl;

            // 압력 값이 4바이트로 들어온다고 가정하고 이를 처리
            if (bytes_read >= 4) {
                // 첫 4바이트를 하나의 압력 값으로 해석 (float 또는 int 형식에 따라)
                uint32_t pressure_raw = (buffer[0] << 24) | (buffer[1] << 16) | (buffer[2] << 8) | buffer[3];
                // 이 값을 실제 압력 값으로 변환 (센서에 따라 변환 공식이 다를 수 있음)
                float pressure = static_cast<float>(pressure_raw) / 100.0; // 예: 압력 값을 헥토파스칼 단위로 변환
                std::cout << "Pressure = " << pressure << " hPa" << std::endl;
            }
        }
        usleep(100000);  // 1초 대기
    }

    close(serial_fd);
    return 0;
}
