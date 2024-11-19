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
//         if (pwm_value < 205 || pwm_value > 410) {
//             std::cerr << "PWM value out of range (205-410)" << std::endl;
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

// // RC 입력 값(172~1811)을 PWM 값(205~410)으로 매핑하는 함수
// int mapChannelValueToPWM(int value) {
//     return static_cast<int>(205 + (value - 172) * (205.0 / (1811 - 172)));
// }

// // 모터 초기화 및 캘리브레이션 해제 함수
// void initializeMotor(PCA9685 &pca9685, int channel) {
//     int max_throttle_pwm = 410;       // 최대 스로틀 PWM 값 (2000us)
//     int zero_throttle_pwm = 205;      // 제로 스로틀 PWM 값 (1000us)

//     for (int i = 0; i < 2; ++i) {  // 최대 스로틀-제로 스로틀 시퀀스를 두 번 반복
//         // 최대 스로틀 신호 보내기 (1초 대기)
//         pca9685.setMotorSpeed(channel, max_throttle_pwm);
//         std::cout << "Initializing motor on channel " << channel << " with max throttle (iteration " << i + 1 << ")..." << std::endl;
//         usleep(1000000); // 1초 동안 최대 스로틀 유지

//         // 제로 스로틀 신호 보내기 (3초 대기)
//         pca9685.setMotorSpeed(channel, zero_throttle_pwm);
//         std::cout << "Setting motor on channel " << channel << " to zero throttle (iteration " << i + 1 << ")..." << std::endl;
//         usleep(3000000); // 3초 동안 제로 스로틀 유지하여 ESC가 캘리브레이션 모드를 해제하도록 대기
//     }

//     // 캘리브레이션 해제 후 안정성을 위해 제로 스로틀 유지
//     std::cout << "Holding motor on channel " << channel << " at zero throttle to ensure calibration exit..." << std::endl;
//     pca9685.setMotorSpeed(channel, zero_throttle_pwm);
//     usleep(3000000); // 추가 3초 대기하여 ESC가 안정적인 모드로 전환되도록 보장
//     std::cout << "Motor on channel " << channel << " fully initialized and calibration mode exited." << std::endl;
// }

// int main() {
//     PCA9685 pca9685;                  // PCA9685 객체 생성 및 초기화
//     initRC("/dev/ttyAMA0", B115200);   // RC 입력 초기화

//     // 각 모터에 대해 초기화 수행
//     for (int channel = 0; channel < 4; ++channel) {
//         initializeMotor(pca9685, channel);
//     }
//     std::cout << "All motors fully initialized and calibration mode exited." << std::endl;

//     // RC 값을 기반으로 모터 제어
//     while (true) {
//         int throttle_value = readRCChannel(3); // 채널 3에서 쓰로틀 값 읽기
//         if (throttle_value != -1) {
//             int pwm_value = mapChannelValueToPWM(throttle_value); // PWM 값으로 변환
//             for (int channel = 0; channel < 4; ++channel) {
//                 pca9685.setMotorSpeed(channel, pwm_value); // 네 개의 모터에 동일한 속도 설정
//             }

//             std::cout << "\rThrottle PWM: " << pwm_value << " applied to all motors" << std::flush;
//         } else {
//             std::cerr << "\rFailed to read throttle channel" << std::flush;
//         }
//         usleep(10000); // 10ms 대기
//     }

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

// const int RC_MIN = 172;
// const int RC_MAX = 1811;
// const int RC_MID = 991;
// const int PWM_MIN = 210;
// const int PWM_MAX = 405;
// const int MAX_ADJUSTMENT = 25; // 각 제어 입력의 최대 PWM 조정 값
// const int I2C_RETRY_LIMIT = 3; // I2C 오류 시 재시도 횟수
// const int SAFE_PWM = PWM_MIN; // 초기화 및 안전한 PWM 값
// const int LOOP_DELAY_US = 10000; // 주기적인 대기 시간 (10ms)

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
//         initializeMotors(); // 모든 모터를 초기 안전 PWM 값으로 설정
//     }

//     ~PCA9685() {
//         stopAllMotors(); // 종료 시 모든 모터를 정지
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
//         if (pwm_value < PWM_MIN || pwm_value > PWM_MAX) {
//             std::cerr << "PWM value out of range (" << PWM_MIN << "-" << PWM_MAX << ")" << std::endl;
//             return;
//         }
//         setPWM(channel, 0, pwm_value);
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
//         int retries = 0;
//         while (write(fd, buffer, 2) != 2) {
//             if (++retries >= I2C_RETRY_LIMIT) {
//                 std::cerr << "Failed to write to the i2c bus after retries" << std::endl;
//                 exit(1);
//             }
//             usleep(1000); // 1ms 대기 후 재시도
//         }
//     }

//     uint8_t readRegister(uint8_t reg) {
//         int retries = 0;
//         while (write(fd, &reg, 1) != 1) {
//             if (++retries >= I2C_RETRY_LIMIT) {
//                 std::cerr << "Failed to write to the i2c bus after retries" << std::endl;
//                 exit(1);
//             }
//             usleep(1000);
//         }
//         uint8_t value;
//         if (read(fd, &value, 1) != 1) {
//             std::cerr << "Failed to read from the i2c bus" << std::endl;
//             exit(1);
//         }
//         return value;
//     }

//     void initializeMotors() {
//         for (int i = 0; i < 4; ++i) {
//             setMotorSpeed(i, SAFE_PWM);
//         }
//     }

//     void stopAllMotors() {
//         for (int i = 0; i < 4; ++i) {
//             setMotorSpeed(i, SAFE_PWM);
//         }
//         std::cout << "All motors stopped safely." << std::endl;
//     }
// };

// // 스로틀 값을 0.0 ~ 1.0 범위로 매핑하는 함수
// double mapThrottle(int value) {
//     if (value <= RC_MIN) return 0.0;
//     if (value >= RC_MAX) return 1.0;
//     return static_cast<double>(value - RC_MIN) / (RC_MAX - RC_MIN);
// }

// // 제어 입력(에일러론, 엘리베이터, 러더)을 -1.0 ~ 1.0 범위로 매핑하는 함수
// double mapControlInput(int value) {
//     if (value < RC_MIN || value > RC_MAX) {
//         std::cerr << "Control input out of range: " << value << std::endl;
//         return 0.0;
//     }
//     if (value < RC_MID) return static_cast<double>(value - RC_MID) / (RC_MID - RC_MIN);
//     if (value > RC_MID) return static_cast<double>(value - RC_MID) / (RC_MAX - RC_MID);
//     return 0.0;
// }

// // 스로틀 PWM 계산 함수
// int computeThrottlePWM(double throttle_normalized) {
//     return static_cast<int>(PWM_MIN + throttle_normalized * (PWM_MAX - PWM_MIN));
// }

// // 에일러론, 엘리베이터, 러더 조정 값 계산 함수
// int computeAdjustment(double control_normalized) {
//     return static_cast<int>(control_normalized * MAX_ADJUSTMENT);
// }

// // 값이 특정 범위 내에 있도록 제한하는 함수
// int clamp(int value, int min_value, int max_value) {
//     return value < min_value ? min_value : (value > max_value ? max_value : value);
// }

// int main() {
//     PCA9685 pca9685;
//     initRC("/dev/ttyAMA0", B115200);  // RC 입력 초기화

//     while (true) {
//         int throttle_value = readRCChannel(3); // 채널 3에서 스로틀 값 읽기
//         int aileron_value = readRCChannel(1);  // 채널 1에서 에일러론 값 읽기
//         int elevator_value = readRCChannel(2); // 채널 2에서 엘리베이터 값 읽기
//         int rudder_value = readRCChannel(4);   // 채널 4에서 러더 값 읽기

//         double throttle_normalized = mapThrottle(throttle_value);
//         double aileron_normalized = mapControlInput(aileron_value);
//         double elevator_normalized = mapControlInput(elevator_value);
//         double rudder_normalized = mapControlInput(rudder_value);

//         // 새 범위에 맞춘 스로틀 PWM 계산
//         int throttle_PWM = computeThrottlePWM(throttle_normalized);

//         int aileron_adj = computeAdjustment(aileron_normalized);
//         int elevator_adj = computeAdjustment(elevator_normalized);
//         int rudder_adj = computeAdjustment(rudder_normalized);

//         // 각 모터별로 스로틀과 조정 값을 계산하여 PWM 설정
//         int motor1_PWM = throttle_PWM - aileron_adj - elevator_adj - rudder_adj;
//         int motor2_PWM = throttle_PWM + aileron_adj - elevator_adj + rudder_adj;
//         int motor3_PWM = throttle_PWM - aileron_adj + elevator_adj + rudder_adj;
//         int motor4_PWM = throttle_PWM + aileron_adj + elevator_adj - rudder_adj;

//         // PWM 값이 최소 값을 유지하도록 조정
//         int min_motor_PWM = std::min(std::min(motor1_PWM, motor2_PWM), std::min(motor3_PWM, motor4_PWM));
//         if (min_motor_PWM < PWM_MIN) {
//             int adjustment = PWM_MIN - min_motor_PWM;
//             motor1_PWM += adjustment;
//             motor2_PWM += adjustment;
//             motor3_PWM += adjustment;
//             motor4_PWM += adjustment;
//         }

//         // PWM 값이 범위 내에 있도록 제한
//         motor1_PWM = clamp(motor1_PWM, PWM_MIN, PWM_MAX);
//         motor2_PWM = clamp(motor2_PWM, PWM_MIN, PWM_MAX);
//         motor3_PWM = clamp(motor3_PWM, PWM_MIN, PWM_MAX);
//         motor4_PWM = clamp(motor4_PWM, PWM_MIN, PWM_MAX);

//         // 각 모터에 계산된 PWM 값 적용
//         pca9685.setMotorSpeed(0, motor1_PWM);
//         pca9685.setMotorSpeed(1, motor2_PWM);
//         pca9685.setMotorSpeed(2, motor3_PWM);
//         pca9685.setMotorSpeed(3, motor4_PWM);

//         std::cout << "\rThrottle PWM: " << throttle_PWM
//                   << " Motor1: " << motor1_PWM
//                   << " Motor2: " << motor2_PWM
//                   << " Motor3: " << motor3_PWM
//                   << " Motor4: " << motor4_PWM << std::flush;

//         usleep(10000); // 10ms 대기
//     }

//     return 0;
// }


// // PWM 시작점 확인 코드
// const int PWM_MIN_TEST = 400;  // 테스트 시작점 (기존 최소 PWM 값)
// const int PWM_MAX_TEST = 430;  // 테스트 범위
// const int TEST_STEP = 5;       // PWM 값 증가 단계

// int main() {
//     PCA9685 pca9685;  // PCA9685 초기화
//     int pwm_value = PWM_MIN_TEST;

//     std::cout << "Testing motor PWM range...\n";
//     while (pwm_value <= PWM_MAX_TEST) {
//         pca9685.setMotorSpeed(0, pwm_value);  // 0번 모터에 PWM 값 설정
//         std::cout << "PWM: " << pwm_value << std::endl;
        
//         sleep(1);  // 1초 대기 후 다음 PWM 값으로 증가
//         pwm_value += TEST_STEP;
//     }

//     // 테스트 종료 후 모든 모터를 최소 PWM 값으로 설정하여 정지
//     pca9685.setMotorSpeed(0, PWM_MIN_TEST);
//     std::cout << "Test completed.\n";

//     return 0;
// }

// #include <iostream>
// #include <fcntl.h>
// #include <unistd.h>
// #include <sys/ioctl.h>
// #include <linux/i2c-dev.h>
// #include <chrono>
// #include <array>
// #include "motor_control.h"
// #include "attitude_controller.h" // controlAttitude 함수 사용

// PCA9685::PCA9685(int address) {
//     char filename[20];
//     snprintf(filename, 19, "/dev/i2c-1");
//     fd = open(filename, O_RDWR);
//     if (fd < 0) {
//         std::cerr << "Failed to open the i2c bus" << std::endl;
//         exit(1);
//     }
//     if (ioctl(fd, I2C_SLAVE, address) < 0) {
//         std::cerr << "Failed to acquire bus access and/or talk to slave" << std::endl;
//         exit(1);
//     }
//     reset();
//     setPWMFreq(50);  
//     initializeMotors(); 
// }

// PCA9685::~PCA9685() {
//     stopAllMotors(); 
//     if (fd >= 0) {
//         close(fd);
//     }
// }

// void PCA9685::setPWM(int channel, int on, int off) {
//     writeRegister(LED0_ON_L + 4 * channel, on & 0xFF);
//     writeRegister(LED0_ON_L + 4 * channel + 1, on >> 8);
//     writeRegister(LED0_OFF_L + 4 * channel, off & 0xFF);
//     writeRegister(LED0_OFF_L + 4 * channel + 1, off >> 8);
// }

// void PCA9685::setMotorSpeed(int channel, int pwm_value) {
//     int off_value = pwm_value * 4095 / 1000;
//     setPWM(channel, 0, off_value);
// }

// void PCA9685::reset() {
//     writeRegister(MODE1, 0x00);
// }

// void PCA9685::setPWMFreq(int freq) {
//     uint8_t prescale = static_cast<uint8_t>(25000000.0 / (4096.0 * freq) - 1.0);
//     uint8_t oldmode = readRegister(MODE1);
//     uint8_t newmode = (oldmode & 0x7F) | 0x10;
//     writeRegister(MODE1, newmode);
//     writeRegister(PRESCALE, prescale);
//     writeRegister(MODE1, oldmode);
//     usleep(5000);
//     writeRegister(MODE1, oldmode | 0xA1);
// }

// void PCA9685::writeRegister(uint8_t reg, uint8_t value) {
//     uint8_t buffer[2] = {reg, value};
//     if (write(fd, buffer, 2) != 2) {
//         std::cerr << "Failed to write to the i2c bus" << std::endl;
//         exit(1);
//     }
// }

// uint8_t PCA9685::readRegister(uint8_t reg) {
//     if (write(fd, &reg, 1) != 1) {
//         std::cerr << "Failed to write to the i2c bus" << std::endl;
//         exit(1);
//     }
//     uint8_t value;
//     if (read(fd, &value, 1) != 1) {
//         std::cerr << "Failed to read from the i2c bus" << std::endl;
//         exit(1);
//     }
//     return value;
// }

// void PCA9685::initializeMotors() {
//     for (int i = 0; i < 4; ++i) {
//         setMotorSpeed(i, 0);  // 초기 설정을 0으로
//     }
// }

// void PCA9685::stopAllMotors() {
//     for (int i = 0; i < 4; ++i) {
//         setMotorSpeed(i, 0);
//     }
//     std::cout << "All motors stopped safely." << std::endl;
// }

// //motor_control사용을 위한 함수
// float getDeltaTime() {
//     // static auto last_time = std::chrono::steady_clock::now();
//     // auto now = std::chrono::steady_clock::now();
//     // float dt = std::chrono::duration<float>(now - last_time).count();
//     // last_time = now;
//     // return dt;
//     return 0.1;
// }

// int main() {
//     PCA9685 pca9685;
//     std::cerr << "motor driver" << std::endl;

//     while (true) {
//         float dt = getDeltaTime();
//         std::array<float, 4> motor_pwms = controlAttitude(dt);

//         pca9685.setMotorSpeed(0, motor_pwms[0]);
//         pca9685.setMotorSpeed(1, motor_pwms[1]);
//         pca9685.setMotorSpeed(2, motor_pwms[2]);
//         pca9685.setMotorSpeed(3, motor_pwms[3]);

//         std::cout << "\rMotor1: " << motor_pwms[0]
//                   << " Motor2: " << motor_pwms[1]
//                   << " Motor3: " << motor_pwms[2]
//                   << " Motor4: " << motor_pwms[3] << std::flush;

//         usleep(10000); // 10ms delay
//     }

//     return 0;
// }

#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <chrono>
#include <array>
#include "motor_control.h"
#include "attitude_controller.h" // controlAttitude 함수 사용

PCA9685::PCA9685(int address) {
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
    setPWMFreq(50);  
    initializeMotors(); 
}

PCA9685::~PCA9685() {
    stopAllMotors(); 
    if (fd >= 0) {
        close(fd);
    }
}

void PCA9685::setPWM(int channel, int on, int off) {
    writeRegister(LED0_ON_L + 4 * channel, on & 0xFF);
    writeRegister(LED0_ON_L + 4 * channel + 1, on >> 8);
    writeRegister(LED0_OFF_L + 4 * channel, off & 0xFF);
    writeRegister(LED0_OFF_L + 4 * channel + 1, off >> 8);
}

void PCA9685::setMotorSpeed(int channel, int pwm_value) {
    int off_value = pwm_value * 4095 / 1000;
    setPWM(channel, 0, off_value);
}

void PCA9685::reset() {
    writeRegister(MODE1, 0x00);
}

void PCA9685::setPWMFreq(int freq) {
    uint8_t prescale = static_cast<uint8_t>(25000000.0 / (4096.0 * freq) - 1.0);
    uint8_t oldmode = readRegister(MODE1);
    uint8_t newmode = (oldmode & 0x7F) | 0x10;
    writeRegister(MODE1, newmode);
    writeRegister(PRESCALE, prescale);
    writeRegister(MODE1, oldmode);
    usleep(5000);
    writeRegister(MODE1, oldmode | 0xA1);
}

void PCA9685::writeRegister(uint8_t reg, uint8_t value) {
    uint8_t buffer[2] = {reg, value};
    if (write(fd, buffer, 2) != 2) {
        std::cerr << "Failed to write to the i2c bus" << std::endl;
        exit(1);
    }
}

uint8_t PCA9685::readRegister(uint8_t reg) {
    if (write(fd, &reg, 1) != 1) {
        std::cerr << "Failed to write to the i2c bus" << std::endl;
        exit(1);
    }
    uint8_t value;
    if (read(fd, &value, 1) != 1) {
        std::cerr << "Failed to read from the i2c bus" << std::endl;
        exit(1);
    }
    return value;
}

void PCA9685::initializeMotors() {
    for (int i = 0; i < 4; ++i) {
        setMotorSpeed(i, 0);  // 초기 설정을 0으로
    }
}

void PCA9685::stopAllMotors() {
    for (int i = 0; i < 4; ++i) {
        setMotorSpeed(i, 0);
    }
    std::cout << "All motors stopped safely." << std::endl;
}

//motor_control사용을 위한 함수
// float getDeltaTime() {
//     // static auto last_time = std::chrono::steady_clock::now();
//     // auto now = std::chrono::steady_clock::now();
//     // float dt = std::chrono::duration<float>(now - last_time).count();
//     // last_time = now;
//     // return dt;
//     return 0.1;
// }

// int main() {
//     PCA9685 pca9685;
//     std::cerr << "motor driver" << std::endl;

//     while (true) {
//         float dt = getDeltaTime();
//         std::array<float, 4> motor_pwms = controlAttitude(dt);

//         // controlAttitude 함수의 반환값 확인
//         std::cerr << "controlAttitude returned: Motor1: " << motor_pwms[0]
//                   << " Motor2: " << motor_pwms[1]
//                   << " Motor3: " << motor_pwms[2]
//                   << " Motor4: " << motor_pwms[3] << std::endl;

//         pca9685.setMotorSpeed(0, motor_pwms[0]);
//         pca9685.setMotorSpeed(1, motor_pwms[1]);
//         pca9685.setMotorSpeed(2, motor_pwms[2]);
//         pca9685.setMotorSpeed(3, motor_pwms[3]);

//         std::cout << "\rMotor1: " << motor_pwms[0]
//                   << " Motor2: " << motor_pwms[1]
//                   << " Motor3: " << motor_pwms[2]
//                   << " Motor4: " << motor_pwms[3] << std::flush;

//         usleep(10000); // 10ms delay
//     }

//     return 0;
// }
