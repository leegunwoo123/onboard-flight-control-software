#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstdint>
#include <cmath>               
#include <Eigen/Dense>         
#include "rc_input.h"          
#include "imu_sensor.h"        
#include "imu_calibration.h"   
#include <termios.h>           
#include <algorithm>           
#include <chrono>              

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
const int MAX_ADJUSTMENT = 5; // 각 제어 입력의 최대 PWM 조정 값
const int I2C_RETRY_LIMIT = 3; // I2C 오류 시 재시도 횟수
const int SAFE_PWM = PWM_MIN;  // 초기화 및 안전한 PWM 값
const int LOOP_DELAY_US = 26000; // 주기적인 대기 시간 (10ms)
const float MAX_ANGLE = 90.0f;           // 최대 각도 (예시)
const float TOLERANCE_ROLL = 0.05f * MAX_ANGLE;   // 롤 허용 오차 (0.45도)
const float TOLERANCE_PITCH = 0.01f * MAX_ANGLE;  // 피치 허용 오차 (0.45도)

// IMUData 
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
        setPWMFreq(50);  // 모터 제어를 위해 주파수를 50Hz로 설정
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

struct PIDController {
    float kp, ki, kd;             // PID 게인
    float prev_error;             // 이전 오차값 (미분 항 계산용)
    float integral;               // 적분값
    float integral_limit;         // 적분값 제한
    float output_limit;           // 출력값 제한
    float feedforward;            // 피드포워드 게인

    PIDController(float p, float i, float d, float ff = 0.0f, float i_limit = 10.0f, float out_limit = 400.0f)
        : kp(p), ki(i), kd(d), feedforward(ff), prev_error(0.0f), integral(0.0f),
          integral_limit(i_limit), output_limit(out_limit) {}

    void reset() {
        prev_error = 0.0f;
        integral = 0.0f;
    }

    float calculate(float setpoint, float measurement, float dt) {
        // 오차 계산
        float error = setpoint - measurement;

        // 적분 항 계산 및 제한
        integral += error * dt;
        integral = std::clamp(integral, -integral_limit, integral_limit);

        // 미분 항 계산
        float derivative = (error - prev_error) / dt;
        prev_error = error;

        // PID 출력 계산 (피드포워드 포함)
        float output = feedforward * setpoint + (kp * error) + (ki * integral) + (kd * derivative);

        // 출력 제한
        return std::clamp(output, -output_limit, output_limit);
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
        return 0.0;
    }
    if (value < RC_MID) return static_cast<double>(value - RC_MID) / (RC_MID - RC_MIN);
    if (value > RC_MID) return static_cast<double>(value - RC_MID) / (RC_MAX - RC_MID);
    return 0.0;
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

int main() {
    PCA9685 pca9685;
    initRC("/dev/ttyAMA0", B115200);  // RC 입력 초기화
    initIMU("/dev/ttyUSB0", B115200);  // IMU 초기화

    // 캘리브레이션 오프셋 변수 선언
    float accelX_calibrated = 0.0f, accelY_calibrated = 0.0f, accelZ_calibrated = 0.0f;
    float gyroX_calibrated = 0.0f, gyroY_calibrated = 0.0f, gyroZ_calibrated = 0.0f;
    float offsetX = 0.0f, offsetY = 0.0f, offsetZ = 0.0f;
    float offsetGyroX = 0.0f, offsetGyroY = 0.0f, offsetGyroZ = 0.0f;
    float roll_acc = 0.0f, pitch_acc = 0.0f;
    float dt = 0.025f;

    // 새로운 변수 선언 (기존 변수 최대한 사용)
    float rollRateSetpoint = 0.0f, pitchRateSetpoint = 0.0f;
    float gyroRoll = 0.0f, gyroPitch = 0.0f;

    try {
        // 캘리브레이션 수행
        IMUCalibrationData calibrationData = calibrateIMU();

        // 캘리브레이션 완료 후 데이터 출력
        std::cout << "IMU Calibration offsets are set." << std::endl;

        accelX_calibrated = calibrationData.accelX_calibrated;
        accelY_calibrated = calibrationData.accelY_calibrated;
        accelZ_calibrated = calibrationData.accelZ_calibrated;

        gyroX_calibrated = calibrationData.gyroX_calibrated;
        gyroY_calibrated = calibrationData.gyroY_calibrated;
        gyroZ_calibrated = calibrationData.gyroZ_calibrated;

        roll_acc = calibrationData.roll_acc;
        pitch_acc = calibrationData.pitch_acc;

        offsetX = calibrationData.offsetX;
        offsetY = calibrationData.offsetY;
        offsetZ = calibrationData.offsetZ;

        offsetGyroX = calibrationData.offsetGyroX;
        offsetGyroY = calibrationData.offsetGyroY;
        offsetGyroZ = calibrationData.offsetGyroZ;

    } catch (const std::exception &e) {
        std::cerr << "Error during calibration: " << e.what() << std::endl;
        return -1;
    }

    float targetRoll = roll_acc;  // 수평 상태를 목표로 함
    float targetPitch = pitch_acc; // 수평 상태를 목표로 함

    // 외부 루프 PIDController 초기화 (각도 제어)
    PIDController rollAnglePID(4.5f, 0.0f, 0.0f, 0.0f, 10.0f, 200.0f);
    PIDController pitchAnglePID(4.5f, 0.0f, 0.0f, 0.0f, 10.0f, 200.0f);

    // 내부 루프 PIDController 초기화 (각속도 제어)
    PIDController rollRatePID(0.15f, 0.0f, 0.05f, 0.0f, 10.0f, MAX_ADJUSTMENT);
    PIDController pitchRatePID(0.15f, 0.0f, 0.05f, 0.0f, 10.0f, MAX_ADJUSTMENT);

    // Yaw PIDController (기존 그대로 사용)
    PIDController yawPID(1.2f, 0.0f, 0.0f, 0.0f, 10.0f, MAX_ADJUSTMENT);

    // 루프 주기 시간 측정을 위한 변수
    auto previousTime = std::chrono::steady_clock::now();

    while (true) {

        int throttle_value = readRCChannel(3); // 채널 3에서 스로틀 값 읽기
        int aileron_value = readRCChannel(1);  // 채널 1에서 에일러론 값 읽기
        int elevator_value = readRCChannel(2); // 채널 2에서 엘리베이터 값 읽기
        int rudder_value = readRCChannel(4);   // 채널 4에서 러더 값 읽기

        double throttle_normalized = mapThrottle(throttle_value);
        double aileron_normalized = mapControlInput(aileron_value);
        double elevator_normalized = mapControlInput(elevator_value);
        double rudder_normalized = mapControlInput(rudder_value);

        // 목표 각도 설정 (조종기 입력에 따라)
        targetRoll = aileron_normalized * 20.0f;   // 최대 ±20도
        targetPitch = elevator_normalized * 20.0f; // 최대 ±20도

        // IMU 데이터 읽기
        IMUData imuData = readIMU();  // IMU 데이터를 읽어서 imuData에 저장

        // 가속도계를 이용한 Roll과 Pitch 계산
        float roll_acc = atan2(imuData.accelY, imuData.accelZ) * 180.0f / M_PI;
        float pitch_acc = atan2(-imuData.accelX, sqrt(imuData.accelY * imuData.accelY + imuData.accelZ * imuData.accelZ)) * 180.0f / M_PI;

        // 자이로스코프 데이터를 각속도로 사용 (deg/s)
        gyroRoll = imuData.gyroX - offsetGyroX;
        gyroPitch = imuData.gyroY - offsetGyroY;
        float gyroYaw = imuData.gyroZ - offsetGyroZ;

        // 외부 루프 PID 계산 (각도 제어) - 각도 에러로부터 목표 각속도 생성
        float rollAngleError = targetRoll - roll_acc;
        float pitchAngleError = targetPitch - pitch_acc;

        // 외부 루프에서의 PID 출력은 목표 각속도 (deg/s)
        rollRateSetpoint = rollAnglePID.calculate(targetRoll, roll_acc, dt);
        pitchRateSetpoint = pitchAnglePID.calculate(targetPitch, pitch_acc, dt);

        // 내부 루프 PID 계산 (각속도 제어)
        int roll_adj = rollRatePID.calculate(rollRateSetpoint, gyroRoll, dt);
        int pitch_adj = pitchRatePID.calculate(pitchRateSetpoint, gyroPitch, dt);

        // Yaw PID 컨트롤러 계산 (자이로스코프 데이터 사용)
        int yaw_adj = yawPID.calculate(rudder_normalized * 50.0f, gyroYaw, dt);

        // 총 조정값 계산 (기존 computeAdjustment 함수 제거)
        int throttle_PWM = computeThrottlePWM(throttle_normalized);

        int motor1_PWM, motor2_PWM, motor3_PWM, motor4_PWM;

        if (throttle_PWM <= PWM_MIN) {
            // 스로틀 값이 최소값 이하일 경우 모든 모터 정지
            motor1_PWM = PWM_MIN;
            motor2_PWM = PWM_MIN;
            motor3_PWM = PWM_MIN;
            motor4_PWM = PWM_MIN;
        } else {

            // 모터 조정값 계산
            int motor1_adj =  roll_adj /*- pitch_adj + yaw_adj*/;
            int motor2_adj = -roll_adj /*- pitch_adj + yaw_adj*/;
            int motor3_adj = -roll_adj /*+ pitch_adj - yaw_adj*/;
            int motor4_adj =  roll_adj /*+ pitch_adj - yaw_adj*/;
            
            // 조정값을 모터 PWM에 적용
            motor1_PWM = throttle_PWM + motor1_adj;
            motor2_PWM = throttle_PWM + motor2_adj;
            motor3_PWM = throttle_PWM + motor3_adj;
            motor4_PWM = throttle_PWM + motor4_adj;

            // 모터 PWM이 유효한 범위 내에 있는지 확인
            motor1_PWM = clamp(motor1_PWM, PWM_MIN, PWM_MAX);
            motor2_PWM = clamp(motor2_PWM, PWM_MIN, PWM_MAX);
            motor3_PWM = clamp(motor3_PWM, PWM_MIN, PWM_MAX);
            motor4_PWM = clamp(motor4_PWM, PWM_MIN, PWM_MAX);
        }

        // 모터에 PWM 값 설정
        pca9685.setMotorSpeed(0, motor1_PWM);
        pca9685.setMotorSpeed(1, motor2_PWM);
        pca9685.setMotorSpeed(2, motor3_PWM);
        pca9685.setMotorSpeed(3, motor4_PWM);
        std::cout << "\rROLL: " << imuData.accelX << "PITCH: " << imuData.accelY << std::flush;
        usleep(25000);
    }

    return 0;
}
