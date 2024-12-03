#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstdint>
#include "../ioss/rc_input.h"
#include "motor_control.h"
#include <termios.h>

#define PCA9685_ADDR 0x40  // PCA9685 I2C 주소
#define MODE1 0x00         // 모드1 레지스터
#define PRESCALE 0xFE      // 프리스케일 레지스터
#define LED0_ON_L 0x06     // 첫 번째 채널 ON 낮은 바이트 레지스터
#define LED0_OFF_L 0x08    // 첫 번째 채널 OFF 낮은 바이트 레지스터

const int RC_MIN = 172;
const int RC_MAX = 1811;
const int RC_MID = 991;
const int PWM_MIN = 210;
const int PWM_MAX = 405;
const int MAX_ADJUSTMENT = 25; // 각 제어 입력의 최대 PWM 조정 값
const int I2C_RETRY_LIMIT = 3; // I2C 오류 시 재시도 횟수
const int SAFE_PWM = PWM_MIN; // 초기화 및 안전한 PWM 값
const int LOOP_DELAY_US = 10000; // 주기적인 대기 시간 (10ms)

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
        initializeMotors(); // 모든 모터를 초기 안전 PWM 값으로 설정
    }

    ~PCA9685() {
        stopAllMotors(); // 종료 시 모든 모터를 정지
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
        if (pwm_value < PWM_MIN || pwm_value > PWM_MAX) {
            std::cerr << "PWM value out of range (" << PWM_MIN << "-" << PWM_MAX << ")" << std::endl;
            return;
        }
        setPWM(channel, 0, pwm_value);
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
        int retries = 0;
        while (write(fd, buffer, 2) != 2) {
            if (++retries >= I2C_RETRY_LIMIT) {
                std::cerr << "Failed to write to the i2c bus after retries" << std::endl;
                exit(1);
            }
            usleep(1000); // 1ms 대기 후 재시도
        }
    }

    uint8_t readRegister(uint8_t reg) {
        int retries = 0;
        while (write(fd, &reg, 1) != 1) {
            if (++retries >= I2C_RETRY_LIMIT) {
                std::cerr << "Failed to write to the i2c bus after retries" << std::endl;
                exit(1);
            }
            usleep(1000);
        }
        uint8_t value;
        if (read(fd, &value, 1) != 1) {
            std::cerr << "Failed to read from the i2c bus" << std::endl;
            exit(1);
        }
        return value;
    }

    void initializeMotors() {
        for (int i = 0; i < 4; ++i) {
            setMotorSpeed(i, SAFE_PWM);
        }
    }

    void stopAllMotors() {
        for (int i = 0; i < 4; ++i) {
            setMotorSpeed(i, SAFE_PWM);
        }
        std::cout << "All motors stopped safely." << std::endl;
    }
};

// 스로틀 값을 0.0 ~ 1.0 범위로 매핑하는 함수
double mapThrottle(int value) {
    if (value <= RC_MIN) return 0.0;
    if (value >= RC_MAX) return 1.0;
    return static_cast<double>(value - RC_MIN) / (RC_MAX - RC_MIN);
}

// 제어 입력(에일러론, 엘리베이터, 러더)을 -1.0 ~ 1.0 범위로 매핑하는 함수
double mapControlInput(int value) {
    if (value < RC_MIN || value > RC_MAX) {
        std::cerr << "Control input out of range: " << value << std::endl;
        return 0.0;
    }
    if (value < RC_MID) return static_cast<double>(value - RC_MID) / (RC_MID - RC_MIN);
    if (value > RC_MID) return static_cast<double>(value - RC_MID) / (RC_MAX - RC_MID);
    return 0.0;
}

// 스로틀 PWM 계산 함수
int computeThrottlePWM(double throttle_normalized) {
    return static_cast<int>(PWM_MIN + throttle_normalized * (PWM_MAX - PWM_MIN));
}

// 에일러론, 엘리베이터, 러더 조정 값 계산 함수
int computeAdjustment(double control_normalized) {
    return static_cast<int>(control_normalized * MAX_ADJUSTMENT);
}

// 값이 특정 범위 내에 있도록 제한하는 함수
int clamp(int value,  int min_value, int max_value) {
    return value < min_value ? min_value : (value > max_value ? max_value : value);
}

int main() {
    PCA9685 pca9685;
    initRC("/dev/ttyAMA0", B115200);  // RC 입력 초기화

    while (true) {
        int throttle_value = readRCChannel(3); // 채널 3에서 스로틀 값 읽기
        int aileron_value = readRCChannel(1);  // 채널 1에서 에일러론 값 읽기
        int elevator_value = readRCChannel(2); // 채널 2에서 엘리베이터 값 읽기
        int rudder_value = readRCChannel(4);   // 채널 4에서 러더 값 읽기

        double throttle_normalized = mapThrottle(throttle_value);
        double aileron_normalized = mapControlInput(aileron_value);
        double elevator_normalized = mapControlInput(elevator_value);
        double rudder_normalized = mapControlInput(rudder_value);

        // 새 범위에 맞춘 스로틀 PWM 계산
        int throttle_PWM = computeThrottlePWM(throttle_normalized);
        int aileron_adj = computeAdjustment(aileron_normalized);
        int elevator_adj = computeAdjustment(elevator_normalized);
        int rudder_adj = computeAdjustment(rudder_normalized);

        // 각 모터별로 스로틀과 조정 값을 계산하여 PWM 설정
        int motor1_PWM = throttle_PWM - aileron_adj - elevator_adj - rudder_adj;
        int motor2_PWM = throttle_PWM + aileron_adj - elevator_adj + rudder_adj;
        int motor3_PWM = throttle_PWM - aileron_adj + elevator_adj + rudder_adj;
        int motor4_PWM = throttle_PWM + aileron_adj + elevator_adj - rudder_adj;

        // PWM 값이 최소 값을 유지하도록 조정
        int min_motor_PWM = std::min(std::min(motor1_PWM, motor2_PWM), std::min(motor3_PWM, motor4_PWM));
        if (min_motor_PWM < PWM_MIN) {
            int adjustment = PWM_MIN - min_motor_PWM;
            motor1_PWM += adjustment;
            motor2_PWM += adjustment;
            motor3_PWM += adjustment;
            motor4_PWM += adjustment;
        }

        // PWM 값이 범위 내에 있도록 제한
        motor1_PWM = clamp(motor1_PWM, PWM_MIN, PWM_MAX);
        motor2_PWM = clamp(motor2_PWM, PWM_MIN, PWM_MAX);
        motor3_PWM = clamp(motor3_PWM, PWM_MIN, PWM_MAX);
        motor4_PWM = clamp(motor4_PWM, PWM_MIN, PWM_MAX);

        // 각 모터에 계산된 PWM 값 적용
        pca9685.setMotorSpeed(0, motor1_PWM);
        pca9685.setMotorSpeed(1, motor2_PWM);
        pca9685.setMotorSpeed(2, motor3_PWM);
        pca9685.setMotorSpeed(3, motor4_PWM);

        std::cout << "\rThrottle PWM: " << throttle_PWM
                  << " Motor1: " << motor1_PWM
                  << " Motor2: " << motor2_PWM
                  << " Motor3: " << motor3_PWM
                  << " Motor4: " << motor4_PWM << std::flush;

        usleep(10000); // 10ms 대기
    }

    return 0;
}