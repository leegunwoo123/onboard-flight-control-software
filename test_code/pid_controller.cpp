// #include <iostream>
// #include <fcntl.h>
// #include <unistd.h>
// #include <sys/ioctl.h>
// #include <linux/i2c-dev.h>
// #include <cstdint>
// #include "rc_input.h"  // RC 입력 헤더 파일
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

#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstdint>
#include <cmath>               // 수학 함수 사용 (atan2, sqrt, M_PI)
#include <Eigen/Dense>         // Eigen 라이브러리
#include "rc_input.h"          // RC 입력 헤더 파일
#include "imu_sensor.h"        // IMU 센서 헤더 포함
#include "imu_calibration.h"
#include <termios.h>           // B115200 설정

#define PCA9685_ADDR 0x40      // PCA9685 I2C 주소
#define MODE1 0x00             // 모드1 레지스터
#define PRESCALE 0xFE          // 프리스케일 레지스터
#define LED0_ON_L 0x06         // 첫 번째 채널 ON 낮은 바이트 레지스터
#define LED0_OFF_L 0x08        // 첫 번째 채널 OFF 낮은 바이트 레지스터

const int RC_MIN = 172;
const int RC_MAX = 1811;
const int RC_MID = 991;
const int PWM_MIN = 210;
const int PWM_MAX = 405;
const int MAX_ADJUSTMENT = 25; // 각 제어 입력의 최대 PWM 조정 값
const int I2C_RETRY_LIMIT = 3; // I2C 오류 시 재시도 횟수
const int SAFE_PWM = PWM_MIN;  // 초기화 및 안전한 PWM 값
const int LOOP_DELAY_US = 10000; // 주기적인 대기 시간 (10ms)

const double MAX_ROLL_PITCH_ANGLE = 30.0; // Roll, Pitch의 최대 각도

// IMU 캘리브레이션 오프셋 값
const float offsetX = 0;
const float offsetY = 0;
const float offsetZ = 0;

IMUData imuData;

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

// PID 제어 구조체
struct PIDController {
    double kp, ki, kd;
    double prev_error;
    double integral;

    PIDController(double p, double i, double d) : kp(p), ki(i), kd(d), prev_error(0.0), integral(0.0) {}

    double calculate(double setpoint, double measurement) {
        double error = setpoint - measurement;
        integral += error * LOOP_DELAY_US / 1000000.0;  // dt로 적분 값 누적
        double derivative = (error - prev_error) / (LOOP_DELAY_US / 1000000.0);
        prev_error = error;
        return kp * error + ki * integral + kd * derivative;
    }
};

// 수평 상태의 목표 Roll과 Pitch 각도 계산 함수
void calculateTargetAngles(float &targetRoll, float &targetPitch) {
    targetRoll = atan2(offsetY, offsetZ) * 180 / M_PI;  // 목표 Roll 각도
    targetPitch = atan2(-offsetX, sqrt(offsetY * offsetY + offsetZ * offsetZ)) * 180 / M_PI;  // 목표 Pitch 각도
}

int computeThrottlePWM(double throttle_normalized) {
    return static_cast<int>(PWM_MIN + throttle_normalized * (PWM_MAX - PWM_MIN));
}

int computeAdjustment(double control_normalized) {
    return static_cast<int>(control_normalized * MAX_ADJUSTMENT);
}

int clamp(int value, int min_value, int max_value) {
    return value < min_value ? min_value : (value > max_value ? max_value : value);
}

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

// int main() {
//     PCA9685 pca9685;
//     initRC("/dev/ttyAMA0", B115200);  // RC 입력 초기화
//     initIMU("/dev/ttyUSB0", B115200);  // 이거 추가함.

//     float targetRoll = 0.0f;
//     float targetPitch = 0.0f;

//     try {
//         std::cerr << "cali3" << std::endl; // 디버그 메시지

//         // 캘리브레이션 수행
//         IMUCalibrationData calibrationData = calibrateIMU();

//         // 캘리브레이션 완료 후 데이터 출력
//         std::cout << "IMU Calibration offsets are set." << std::endl;

//         // 캘리브레이션 데이터로 오프셋 값 설정
//         const float offsetX = calibrationData.offsetX;
//         const float offsetY = calibrationData.offsetY;
//         const float offsetZ = calibrationData.offsetZ;

//         targetRoll = offsetX;
//         targetPitch = offsetY;

//         std::cout << "Target Roll: " << targetRoll << ", Target Pitch: " << targetPitch << std::endl;
//     } catch (const std::exception &e) {
//         std::cerr << "Error: " << e.what() << std::endl;
//         return -1;
//     }

//     PIDController rollPID(0.06, 0.01, 0.04);
//     PIDController pitchPID(0.06, 0.01, 0.04);

//     // // 수평 상태의 목표 Roll과 Pitch 각도 계산
//     // float targetRoll, targetPitch;
//     // calculateTargetAngles(targetRoll, targetPitch);

//     float currentRoll = 0.0;
//     float currentPitch = 0.0;

//     while (true) {
//         int throttle_value = readRCChannel(3); // 채널 3에서 스로틀 값 읽기
//         int aileron_value = readRCChannel(1);  // 채널 1에서 에일러론 값 읽기
//         int elevator_value = readRCChannel(2); // 채널 2에서 엘리베이터 값 읽기
//         int rudder_value = readRCChannel(4);   // 채널 4에서 러더 값 읽기

//         double throttle_normalized = mapThrottle(throttle_value);
//         double aileron_normalized = mapControlInput(aileron_value);
//         double elevator_normalized = mapControlInput(elevator_value);
//         double rudder_normalized = mapControlInput(rudder_value);
//         // std::cout << "1" << std::endl;
        
//         // IMU 데이터 읽기
//         IMUData imuData = readIMU();  // IMU 데이터를 읽어서 imuData에 저장
//         // std::cout << "2" << imuData.gyroZ << std::flush;

//         // 현재 자세 값을 IMU 데이터로 설정 (자이로스코프와 가속도계 융합)
//         float accelRoll = atan2(imuData.accelY, imuData.accelZ) * 180 / M_PI;
//         float accelPitch = atan2(-imuData.accelX, sqrt(imuData.accelY * imuData.accelY + imuData.accelZ * imuData.accelZ)) * 180 / M_PI;
//         // std::cout << "IMU Roll: " << accelRoll << " degrees, Pitch: " << accelPitch << " degrees" << std::endl;

//         float alpha = 0.98; // 자이로스코프와 가속도계의 가중치 비율
//         currentRoll = alpha * (currentRoll + imuData.gyroX * (LOOP_DELAY_US / 1000000.0)) + (1 - alpha) * accelRoll;
//         currentPitch = alpha * (currentPitch + imuData.gyroY * (LOOP_DELAY_US / 1000000.0)) + (1 - alpha) * accelPitch;

//         // Roll과 Pitch 각도를 ±30도 이내로 제한
//         currentRoll = std::max(-static_cast<float>(MAX_ROLL_PITCH_ANGLE), std::min(currentRoll, static_cast<float>(MAX_ROLL_PITCH_ANGLE)));
//         currentPitch = std::max(-static_cast<float>(MAX_ROLL_PITCH_ANGLE), std::min(currentPitch, static_cast<float>(MAX_ROLL_PITCH_ANGLE)));

//         // 수평 상태 목표 각도 적용
//         int roll_adj = rollPID.calculate(targetRoll, currentRoll);
//         int pitch_adj = pitchPID.calculate(targetPitch, currentPitch);

//         // 입력된 제어 신호에 PID 조정 값을 추가하여 모터 제어
//         int throttle_PWM = computeThrottlePWM(throttle_normalized);
//         int aileron_adj = computeAdjustment(aileron_normalized) + roll_adj;
//         int elevator_adj = computeAdjustment(elevator_normalized) + pitch_adj;
//         int rudder_adj = computeAdjustment(rudder_normalized);

//         int motor1_PWM = throttle_PWM - aileron_adj + elevator_adj - rudder_adj;
//         int motor2_PWM = throttle_PWM + aileron_adj - elevator_adj - rudder_adj;
//         int motor3_PWM = throttle_PWM + aileron_adj + elevator_adj + rudder_adj;
//         int motor4_PWM = throttle_PWM - aileron_adj - elevator_adj + rudder_adj;

//         int min_motor_PWM = std::min(std::min(motor1_PWM, motor2_PWM), std::min(motor3_PWM, motor4_PWM));
//         if (min_motor_PWM < PWM_MIN) {
//             int adjustment = PWM_MIN - min_motor_PWM;
//             motor1_PWM += adjustment;
//             motor2_PWM += adjustment;
//             motor3_PWM += adjustment;
//             motor4_PWM += adjustment;
//         }

//         motor1_PWM = clamp(motor1_PWM, PWM_MIN, PWM_MAX);
//         motor2_PWM = clamp(motor2_PWM, PWM_MIN, PWM_MAX);
//         motor3_PWM = clamp(motor3_PWM, PWM_MIN, PWM_MAX);
//         motor4_PWM = clamp(motor4_PWM, PWM_MIN, PWM_MAX);

//         pca9685.setMotorSpeed(0, motor1_PWM);
//         pca9685.setMotorSpeed(1, motor2_PWM);
//         pca9685.setMotorSpeed(2, motor3_PWM);
//         pca9685.setMotorSpeed(3, motor4_PWM);

//         std::cout << "\rThrottle PWM: " << throttle_PWM
//                   << " Motor1: " << motor1_PWM
//                   << " Motor2: " << motor2_PWM
//                   << " Motor3: " << motor3_PWM
//                   << " Motor4: " << motor4_PWM << std::flush;
//                 //   << " Roll: " << currentRoll << " Pitch: " << currentPitch << " Yaw: " << imuData.gyroZ << std::flush;

//         usleep(10000); // 10ms 대기
//     }

//     return 0;
// }

int main() {
    PCA9685 pca9685;
    initRC("/dev/ttyAMA0", B115200);  // RC 입력 초기화
    initIMU("/dev/ttyUSB0", B115200);  // 이거 추가함.

    float targetRoll = 0.0f;
    float targetPitch = 0.0f;

    try {
        std::cerr << "cali3" << std::endl; // 디버그 메시지

        // 캘리브레이션 수행
        IMUCalibrationData calibrationData = calibrateIMU();

        // 캘리브레이션 완료 후 데이터 출력
        std::cout << "IMU Calibration offsets are set." << std::endl;

        // 캘리브레이션 데이터로 오프셋 값 설정
        const float offsetX = calibrationData.offsetX;
        const float offsetY = calibrationData.offsetY;
        const float offsetZ = calibrationData.offsetZ;

        targetRoll = offsetX;
        targetPitch = offsetY;

        std::cout << "Target Roll: " << targetRoll << ", Target Pitch: " << targetPitch << std::endl;
    } catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }

    PIDController rollPID(0.06, 0.01, 0.04);
    PIDController pitchPID(0.06, 0.01, 0.04);

    float currentRoll = 0.0;
    float currentPitch = 0.0;

    while (true) {
        int throttle_value = readRCChannel(3); // 채널 3에서 스로틀 값 읽기
        int aileron_value = readRCChannel(1);  // 채널 1에서 에일러론 값 읽기
        int elevator_value = readRCChannel(2); // 채널 2에서 엘리베이터 값 읽기
        int rudder_value = readRCChannel(4);   // 채널 4에서 러더 값 읽기

        double throttle_normalized = mapThrottle(throttle_value);
        double aileron_normalized = mapControlInput(aileron_value);
        double elevator_normalized = mapControlInput(elevator_value);
        double rudder_normalized = mapControlInput(rudder_value);
        
        // IMU 데이터 읽기
        IMUData imuData = readIMU();  // IMU 데이터를 읽어서 imuData에 저장

        // 현재 자세 값을 IMU 데이터로 설정 (자이로스코프와 가속도계 융합)
        float accelRoll = atan2(imuData.accelY, imuData.accelZ) * 180 / M_PI;
        float accelPitch = atan2(-imuData.accelX, sqrt(imuData.accelY * imuData.accelY + imuData.accelZ * imuData.accelZ)) * 180 / M_PI;

        float alpha = 0.98; // 자이로스코프와 가속도계의 가중치 비율
        currentRoll = alpha * (currentRoll + imuData.gyroX * (LOOP_DELAY_US / 1000000.0)) + (1 - alpha) * accelRoll;
        currentPitch = alpha * (currentPitch + imuData.gyroY * (LOOP_DELAY_US / 1000000.0)) + (1 - alpha) * accelPitch;

        // Roll과 Pitch 각도를 ±30도 이내로 제한
        currentRoll = std::max(-static_cast<float>(MAX_ROLL_PITCH_ANGLE), std::min(currentRoll, static_cast<float>(MAX_ROLL_PITCH_ANGLE)));
        currentPitch = std::max(-static_cast<float>(MAX_ROLL_PITCH_ANGLE), std::min(currentPitch, static_cast<float>(MAX_ROLL_PITCH_ANGLE)));

        // 수평 상태 목표 각도 적용
        int roll_adj = rollPID.calculate(targetRoll, currentRoll);
        int pitch_adj = pitchPID.calculate(targetPitch, currentPitch);

        // 입력된 제어 신호에 PID 조정 값을 추가하여 모터 제어
        int throttle_PWM = computeThrottlePWM(throttle_normalized);
        int aileron_adj = computeAdjustment(aileron_normalized) + roll_adj;
        int elevator_adj = computeAdjustment(elevator_normalized) + pitch_adj;
        int rudder_adj = computeAdjustment(rudder_normalized);

        int motor1_PWM, motor2_PWM, motor3_PWM, motor4_PWM;

        if (throttle_PWM <= 210) {
            // 스로틀 값이 210 이하일 경우 모든 모터 정지
            motor1_PWM = PWM_MIN;
            motor2_PWM = PWM_MIN;
            motor3_PWM = PWM_MIN;
            motor4_PWM = PWM_MIN;
        } else {
            motor1_PWM = throttle_PWM - aileron_adj + elevator_adj - rudder_adj;
            motor2_PWM = throttle_PWM + aileron_adj - elevator_adj - rudder_adj;
            motor3_PWM = throttle_PWM + aileron_adj + elevator_adj + rudder_adj;
            motor4_PWM = throttle_PWM - aileron_adj - elevator_adj + rudder_adj;

            int min_motor_PWM = std::min(std::min(motor1_PWM, motor2_PWM), std::min(motor3_PWM, motor4_PWM));
            if (min_motor_PWM < PWM_MIN) {
                int adjustment = PWM_MIN - min_motor_PWM;
                motor1_PWM += adjustment;
                motor2_PWM += adjustment;
                motor3_PWM += adjustment;
                motor4_PWM += adjustment;
            }

            motor1_PWM = clamp(motor1_PWM, PWM_MIN, PWM_MAX);
            motor2_PWM = clamp(motor2_PWM, PWM_MIN, PWM_MAX);
            motor3_PWM = clamp(motor3_PWM, PWM_MIN, PWM_MAX);
            motor4_PWM = clamp(motor4_PWM, PWM_MIN, PWM_MAX);
        }

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
