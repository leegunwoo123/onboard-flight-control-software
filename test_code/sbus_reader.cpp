#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <cstdint>
#include <cstring>
#include <iomanip>

#define SERIAL_PORT "/dev/ttyAMA0"
#define BAUDRATE B115200
#define SBUS_FRAME_SIZE 35
#define START_BYTE 0x0F

int main() {
    int serial_port = open(SERIAL_PORT, O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_port == -1) {
        std::cerr << "Error opening serial port: " << SERIAL_PORT << std::endl;
        return 1;
    }

    termios tty;
    memset(&tty, 0, sizeof(tty));

    if (tcgetattr(serial_port, &tty) != 0) {
        std::cerr << "Error getting terminal attributes." << std::endl;
        close(serial_port);
        return 1;
    }

    cfsetispeed(&tty, BAUDRATE);
    cfsetospeed(&tty, BAUDRATE);

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;

    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;
    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~(ECHO | ECHOE | ISIG);

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP);
    tty.c_oflag &= ~OPOST;

    tty.c_cc[VMIN] = SBUS_FRAME_SIZE;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        std::cerr << "Error setting terminal attributes." << std::endl;
        close(serial_port);
        return 1;
    }

    uint8_t sbus_data[SBUS_FRAME_SIZE];

    while (true) {
        int num_bytes = read(serial_port, &sbus_data, SBUS_FRAME_SIZE);
        if (num_bytes == SBUS_FRAME_SIZE && sbus_data[0] == START_BYTE) {
            uint8_t xor_checksum = 0;
            for (int i = 1; i < SBUS_FRAME_SIZE - 1; ++i) {
                xor_checksum ^= sbus_data[i];
            }

            if (xor_checksum != sbus_data[SBUS_FRAME_SIZE - 1]) {
                std::cerr << "Checksum error. Discarding frame." << std::endl;
                continue;
            }

            uint16_t channels[16];
            for (int i = 0; i < 16; ++i) {
                channels[i] = (sbus_data[1 + i * 2] << 8) | sbus_data[2 + i * 2];
            }

            // Print channels 1 to 5 on the same line, updating in place
            std::cout << "\r";
            for (int i = 0; i < 5; i++) {
                std::cout << "Channel " << (i + 1) << ": " << std::setw(4) << channels[i] << " ";
            }
            std::cout << std::flush;

            // 플래그 처리(다른 용도로 필요한 경우)
            uint8_t flags = sbus_data[33];
            bool ch17 = flags & 0x80;
            bool ch18 = flags & 0x40;
            bool frame_lost = flags & 0x20;
            bool failsafe = flags & 0x10;
        }
    }

    close(serial_port);
    return 0;
}
