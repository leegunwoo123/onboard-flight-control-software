// #include <iostream>
// #include <fcntl.h>
// #include <unistd.h>
// #include <sys/ioctl.h>
// #include <linux/i2c-dev.h>
// #include <cstdint>
// #include "../ioss/rc_input.h"
// #include "motor_control.h"
// #include <termios.h> // B115200 설정

// #define PCA9685_ADDR 0x40  // PCA9685 I2C 주소
// #define MODE1 0x00         // 모드1 레지스터
// #define PRESCALE 0xFE      // 프리스케일 레지스터
// #define LED0_ON_L 0x06     // 첫 번째 채널 ON 낮은 바이트 레지스터
// #define LED0_OFF_L 0x08    // 첫 번째 채널 OFF 낮은 바이트 레지스터

// static int i2c_fd; // I2C 파일 디스크립터

// // I2C 장치 초기화 함수
// bool initI2C(const char* device) {
//     // I2C 장치 파일을 엽니다.
//     i2c_fd = open(device, O_RDWR);
//     if (i2c_fd < 0) {
//         std::cerr << "Failed to open I2C device" << std::endl;
//         return false;
//     }

//     // PCA9685의 주소를 설정합니다.
//     if (ioctl(i2c_fd, I2C_SLAVE, PCA9685_ADDR) < 0) {
//         std::cerr << "Failed to set I2C address" << std::endl;
//         close(i2c_fd);
//         return false;
//     }

//     return true;
// }

// // PCA9685 초기화 함수
// void initPCA9685(int pwm_freq) {
//     uint8_t mode1_sleep = 0x10;
//     uint8_t mode1_normal = 0x00;
//     uint8_t auto_increment = 0x80;
//     int prescale = static_cast<int>(25000000.0 / (4096 * pwm_freq) - 1);

//     // sleep 모드로 설정
//     uint8_t buffer1[2] = {MODE1, mode1_sleep};
//     write(i2c_fd, buffer1, 2);

//     // 프리스케일 설정
//     uint8_t buffer2[2] = {PRESCALE, static_cast<uint8_t>(prescale)};
//     write(i2c_fd, buffer2, 2);

//     // normal 모드로 전환 후 대기
//     uint8_t buffer3[2] = {MODE1, mode1_normal};
//     write(i2c_fd, buffer3, 2);
//     usleep(500);

//     // 자동 증가 모드 설정
//     uint8_t buffer4[2] = {MODE1, auto_increment};
//     write(i2c_fd, buffer4, 2);
// }

// // 특정 채널에 PWM 신호를 설정하는 함수
// void setPWM(int channel, int on, int off) {
//     uint8_t buffer[5];
//     buffer[0] = LED0_ON_L + 4 * channel;
//     buffer[1] = on & 0xFF;         // ON 낮은 바이트
//     buffer[2] = on >> 8;           // ON 높은 바이트
//     buffer[3] = off & 0xFF;        // OFF 낮은 바이트
//     buffer[4] = off >> 8;          // OFF 높은 바이트

//     write(i2c_fd, buffer, 5);
// }

// // PWM 값 변환 함수: 172~1811 범위를 0~4095 범위로 매핑
// int mapChannelValueToPWM(int value) {
//     return static_cast<int>((value - 172) * (4095.0 / (1811 - 172)));
// }

// // 모터 제어 함수: 0~3 채널을 제어
// void controlMotors() {
//     for (int channel = 0; channel < 4; ++channel) {
//         int rc_value = readRCChannel(channel + 1); // 0~3 채널을 읽어옴
//         if (rc_value != -1) {
//             int pwm_value = mapChannelValueToPWM(rc_value); // PWM 값으로 변환
//             setPWM(channel, 0, pwm_value);                 // 변환된 값을 채널에 설정
//             std::cout << "Channel " << channel << " PWM: " << pwm_value << std::endl;
//         } else {
//             std::cerr << "Failed to read RC channel " << channel << std::endl;
//         }
//     }
// }

// int main() {
//     // I2C 버스 초기화
//     if (!initI2C("/dev/i2c-1")) {  // I2C 버스 1 사용
//         return 1;
//     }

//     initPCA9685(50);               // PCA9685를 50Hz로 초기화 (드론 모터 주파수에 맞춤)
//     initRC("/dev/ttyAMA0", B115200); // RC 입력 초기화
    
//     while (true) {
//         controlMotors();  // 모터 제어
//         usleep(10000);    // 10ms 대기
//     }

//     close(i2c_fd);  // I2C 장치 파일 닫기
//     return 0;
// }

// #include <iostream>
// #include <fcntl.h>
// #include <unistd.h>
// #include <sys/ioctl.h>
// #include <linux/i2c-dev.h>
// #include <cstdint>
// #include "../ioss/rc_input.h"  // RC 입력 헤더 파일
// #include "motor_control.h"
// #include <termios.h>           // B115200 설정

// #define PCA9685_ADDR 0x40  // PCA9685 I2C 주소
// #define MODE1 0x00         // 모드1 레지스터
// #define PRESCALE 0xFE      // 프리스케일 레지스터
// #define LED0_ON_L 0x06     // 첫 번째 채널 ON 낮은 바이트 레지스터
// #define LED0_OFF_L 0x08    // 첫 번째 채널 OFF 낮은 바이트 레지스터

// class PCA9685 {
// public:
//     PCA9685(int address = PCA9685_ADDR) {
//         char filename[20];
//         snprintf(filename, 19, "/dev/i2c-1");
//         fd = open(filename, O_RDWR);
//         if (fd < 0) {
//             std::cerr << "Failed to open the i2c bus" << std::endl;
//             exit(1);
//         }
//         if (ioctl(fd, I2C_SLAVE, address) < 0) {
//             std::cerr << "Failed to acquire bus access and/or talk to slave" << std::endl;
//             exit(1);
//         }
//         reset();
//         setPWMFreq(50);  // Set frequency to 50Hz for motor control
//     }

//     ~PCA9685() {
//         if (fd >= 0) {
//             close(fd);
//         }
//     }

//     void setPWM(int channel, int on, int off) {
//         writeRegister(LED0_ON_L + 4 * channel, on & 0xFF);
//         writeRegister(LED0_ON_L + 4 * channel + 1, on >> 8);
//         writeRegister(LED0_OFF_L + 4 * channel, off & 0xFF);
//         writeRegister(LED0_OFF_L + 4 * channel + 1, off >> 8);
//     }

//     void setMotorSpeed(int channel, int pwm_value) {
//         // PWM 값이 모터의 유효 범위 내에 있는지 확인
//         if (pwm_value < 150 || pwm_value > 600) {
//             std::cerr << "PWM value out of range (150-600)" << std::endl;
//             return;
//         }
//         setPWM(channel, 0, pwm_value);  // 모터에 PWM 값 설정
//     }

// private:
//     int fd;

//     void reset() {
//         writeRegister(MODE1, 0x00);
//     }

//     void setPWMFreq(int freq) {
//         uint8_t prescale = static_cast<uint8_t>(25000000.0 / (4096.0 * freq) - 1.0);
//         uint8_t oldmode = readRegister(MODE1);
//         uint8_t newmode = (oldmode & 0x7F) | 0x10;
//         writeRegister(MODE1, newmode);
//         writeRegister(PRESCALE, prescale);
//         writeRegister(MODE1, oldmode);
//         usleep(5000);
//         writeRegister(MODE1, oldmode | 0xA1);
//     }

//     void writeRegister(uint8_t reg, uint8_t value) {
//         uint8_t buffer[2] = {reg, value};
//         if (write(fd, buffer, 2) != 2) {
//             std::cerr << "Failed to write to the i2c bus" << std::endl;
//         }
//     }

//     uint8_t readRegister(uint8_t reg) {
//         if (write(fd, &reg, 1) != 1) {
//             std::cerr << "Failed to write to the i2c bus" << std::endl;
//         }
//         uint8_t value;
//         if (read(fd, &value, 1) != 1) {
//             std::cerr << "Failed to read from the i2c bus" << std::endl;
//         }
//         return value;
//     }
// };

// // PWM 값 변환 함수: RC 입력 값(172~1811)을 PWM 값(205~410)으로 매핑
// int mapChannelValueToPWM(int value) {
//     return static_cast<int>(205 + (value - 172) * (205.0 / (1811 - 172)));
// }

// // 모터 제어 함수: 쓰로틀 값을 읽어 네 개의 모터에 동일한 속도로 적용
// void controlMotors(PCA9685 &pca9685) {
//     int throttle_value = readRCChannel(3); // 채널 3에서 쓰로틀 값을 읽음
//     if (throttle_value != -1) {
//         int pwm_value = mapChannelValueToPWM(throttle_value); // PWM 값으로 변환
//         for (int channel = 0; channel < 4; ++channel) {
//             pca9685.setMotorSpeed(channel, pwm_value); // 네 개의 모터에 동일한 속도 설정
//         }

//         // 같은 줄에서 출력 업데이트
//         std::cout << "\rThrottle PWM: " << pwm_value << " applied to all motors" << std::flush;
//     } else {
//         std::cerr << "\rFailed to read throttle channel" << std::flush;
//     }
// }

// int main() {
//     PCA9685 pca9685;                  // PCA9685 객체 생성 및 초기화
//     initRC("/dev/ttyAMA0", B115200);   // RC 입력 초기화
    
//     while (true) {
//         controlMotors(pca9685);        // RC 값을 기반으로 모터 제어
//         usleep(10000);                 // 10ms 대기
//     }

//     return 0;
// }



#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstdint>
#include "../ioss/rc_input.h"  // RC 입력 헤더 파일
#include "motor_control.h"
#include <termios.h>           // B115200 설정

#define PCA9685_ADDR 0x40  // PCA9685 I2C 주소
#define MODE1 0x00         // 모드1 레지스터
#define PRESCALE 0xFE      // 프리스케일 레지스터
#define LED0_ON_L 0x06     // 첫 번째 채널 ON 낮은 바이트 레지스터
#define LED0_OFF_L 0x08    // 첫 번째 채널 OFF 낮은 바이트 레지스터

class PCA9685 {
public:
    PCA9685(int address = PCA9685_ADDR) {
        char filename[20];
        snprintf(filename, 19, "/dev/i2c-1");
        fd = open(filename, O_RDWR);
        if (fd < 0) {
            std::cerr << "Failed to open the i2c bus" << std::endl;
            exit(1);
        }
        if (ioctl(fd, I2C_SLAVE, address) < 0) {
            std::cerr << "Failed to acquire bus access and/or talk to slave" << std::endl;
            exit(1);
        }
        reset();
        setPWMFreq(50);  // Set frequency to 50Hz for motor control
    }

    ~PCA9685() {
        if (fd >= 0) {
            close(fd);
        }
    }

    void setPWM(int channel, int on, int off) {
        writeRegister(LED0_ON_L + 4 * channel, on & 0xFF);
        writeRegister(LED0_ON_L + 4 * channel + 1, on >> 8);
        writeRegister(LED0_OFF_L + 4 * channel, off & 0xFF);
        writeRegister(LED0_OFF_L + 4 * channel + 1, off >> 8);
    }

    void setMotorSpeed(int channel, int pwm_value) {
        // PWM 값이 모터의 유효 범위 내에 있는지 확인
        if (pwm_value < 150 || pwm_value > 600) {
            std::cerr << "PWM value out of range (150-600)" << std::endl;
            return;
        }
        setPWM(channel, 0, pwm_value);  // 모터에 PWM 값 설정
    }

private:
    int fd;

    void reset() {
        writeRegister(MODE1, 0x00);
    }

    void setPWMFreq(int freq) {
        uint8_t prescale = static_cast<uint8_t>(25000000.0 / (4096.0 * freq) - 1.0);
        uint8_t oldmode = readRegister(MODE1);
        uint8_t newmode = (oldmode & 0x7F) | 0x10;
        writeRegister(MODE1, newmode);
        writeRegister(PRESCALE, prescale);
        writeRegister(MODE1, oldmode);
        usleep(5000);
        writeRegister(MODE1, oldmode | 0xA1);
    }

    void writeRegister(uint8_t reg, uint8_t value) {
        uint8_t buffer[2] = {reg, value};
        if (write(fd, buffer, 2) != 2) {
            std::cerr << "Failed to write to the i2c bus" << std::endl;
        }
    }

    uint8_t readRegister(uint8_t reg) {
        if (write(fd, &reg, 1) != 1) {
            std::cerr << "Failed to write to the i2c bus" << std::endl;
        }
        uint8_t value;
        if (read(fd, &value, 1) != 1) {
            std::cerr << "Failed to read from the i2c bus" << std::endl;
        }
        return value;
    }
};

// 입력 범위를 -1.0 ~ 1.0으로 정규화하는 함수
double normalizeRCInput(int value, int min_input, int center_input, int max_input) {
    if (value < center_input) {
        return (value - center_input) / (double)(center_input - min_input);
    } else {
        return (value - center_input) / (double)(max_input - center_input);
    }
}

// 쓰로틀 값을 0.0 ~ 1.0으로 정규화하는 함수
double normalizeThrottle(int value, int min_input, int max_input) {
    return (value - min_input) / (double)(max_input - min_input);
}

// 쓰로틀 값을 PWM 값으로 매핑하는 함수
int mapThrottleToPWM(double throttle) {
    int pwm_min = 205;
    int pwm_max = 410;
    return pwm_min + (int)(throttle * (pwm_max - pwm_min));
}

// 모드 입력 처리 함수
void processModeInput(int mode_input, int &mode) {
    if (mode_input >= 800 && mode_input <= 1200) {
        mode = 0;
    } else if (mode_input >= 1250 && mode_input <= 1500) {
        mode = 1;
    } else {
        // Default or unknown mode
        mode = -1;
    }
}

// 모터 제어 함수: 모든 RC 입력 값을 읽어 모터에 적용
void controlMotors(PCA9685 &pca9685) {
    // RC 입력 값 읽기
    int roll_input = readRCChannel(1);      // 에일러론(roll)
    int pitch_input = readRCChannel(2);     // 엘리베이터(pitch)
    int throttle_input = readRCChannel(3);  // 스로틀(throttle)
    int yaw_input = readRCChannel(4);       // 러더(rudder)
    int mode_input = readRCChannel(5);      // 모드 변환

    if (throttle_input != -1 && roll_input != -1 && pitch_input != -1 && yaw_input != -1 && mode_input != -1) {
        // 입력 값 매핑
        double throttle = normalizeThrottle(throttle_input, 172, 1811);  // 0.0 to 1.0
        double roll = normalizeRCInput(roll_input, 172, 991, 1811);      // -1.0 to 1.0
        double pitch = normalizeRCInput(pitch_input, 172, 991, 1811);    // -1.0 to 1.0
        double yaw = normalizeRCInput(yaw_input, 172, 991, 1811);        // -1.0 to 1.0

        // 모드 처리
        int mode = -1;
        processModeInput(mode_input, mode);

        // 모드에 따라 최대 조정 값 설정
        int max_adjustment = 50;
        if (mode == 0) {
            max_adjustment = 50;
        } else if (mode == 1) {
            max_adjustment = 25;
        } else {
            // 기본 최대 조정 값
            max_adjustment = 50;
        }

        // 모터 PWM 값 계산
        int pwm_values[4];

        // 쓰로틀로부터 기본 PWM 값 계산
        int base_pwm = mapThrottleToPWM(throttle);

        // 롤, 피치, 요우에 대한 조정 값 계산
        int roll_adjustment = static_cast<int>(roll * max_adjustment);
        int pitch_adjustment = static_cast<int>(pitch * max_adjustment);
        int yaw_adjustment = static_cast<int>(yaw * max_adjustment);

        // 모터에 대한 조정 값 적용 (쿼드콥터의 X 구성)
        // 모터 0 (앞왼쪽)
        pwm_values[0] = base_pwm + pitch_adjustment + roll_adjustment - yaw_adjustment;
        // 모터 1 (앞오른쪽)
        pwm_values[1] = base_pwm + pitch_adjustment - roll_adjustment + yaw_adjustment;
        // 모터 2 (뒤오른쪽)
        pwm_values[2] = base_pwm - pitch_adjustment - roll_adjustment - yaw_adjustment;
        // 모터 3 (뒤왼쪽)
        pwm_values[3] = base_pwm - pitch_adjustment + roll_adjustment + yaw_adjustment;

        // PWM 값이 유효 범위 내에 있는지 확인
        for (int i = 0; i < 4; ++i) {
            if (pwm_values[i] < 150) pwm_values[i] = 150;
            if (pwm_values[i] > 600) pwm_values[i] = 600;
        }

        // 모터에 PWM 값 적용
        for (int channel = 0; channel < 4; ++channel) {
            pca9685.setMotorSpeed(channel, pwm_values[channel]);
        }

        // 같은 줄에서 출력 업데이트
        std::cout << "\rThrottle PWM: " << base_pwm << " Roll adj: " << roll_adjustment
                  << " Pitch adj: " << pitch_adjustment << " Yaw adj: " << yaw_adjustment
                  << " Mode: " << mode << std::flush;

    } else {
        std::cerr << "\rFailed to read RC channels" << std::flush;
    }
}

int main() {
    PCA9685 pca9685;                  // PCA9685 객체 생성 및 초기화
    initRC("/dev/ttyAMA0", B115200);   // RC 입력 초기화

    while (true) {
        controlMotors(pca9685);        // RC 값을 기반으로 모터 제어
        usleep(10000);                 // 10ms 대기
    }

    return 0;
}
