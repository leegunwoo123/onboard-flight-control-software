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
#include "imu_calibration.h"   // IMU 캘리브레이션 헤더 파일
#include <termios.h>           // B115200 설정
#include <algorithm>           // std::clamp 함수 사용
#include <chrono>              // 시간 측정을 위한 라이브러리

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
const int LOOP_DELAY_US = 2500; // 주기적인 대기 시간 (2.5ms)
const float MAX_ANGLE = 90.0f;           // 최대 각도 (예시)
const float TOLERANCE_ROLL = 0.01f * MAX_ANGLE;   // 롤 허용 오차 (0.45도)
const float TOLERANCE_PITCH = 0.01f * MAX_ANGLE;  // 피치 허용 오차 (0.45도)

// IMUData 구조체는 imu_sensor.h에서 정의되어 있다고 가정합니다.
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
        //std::cerr << "Control input out of range: " << value << std::endl;
        return 0.0;
    }
    if (value < RC_MID) return static_cast<double>(value - RC_MID) / (RC_MID - RC_MIN);
    if (value > RC_MID) return static_cast<double>(value - RC_MID) / (RC_MAX - RC_MID);
    return 0.0;
}

int computeThrottlePWM(double throttle_normalized) {
    return static_cast<int>(PWM_MIN + throttle_normalized * (PWM_MAX - PWM_MIN));
}

int clamp(int value, int min_value, int max_value) {
    return value < min_value ? min_value : (value > max_value ? max_value : value);
}

int main() {
    PCA9685 pca9685;
    initRC("/dev/ttyAMA0", B115200);  // RC 입력 초기화
    initIMU("/dev/ttyUSB0", B115200);  // IMU 초기화

    // 캘리브레이션 오프셋 변수 선언
    float offsetX = 0.0f, offsetY = 0.0f, offsetZ = 0.0f;
    float offsetGyroX = 0.0f, offsetGyroY = 0.0f, offsetGyroZ = 0.0f;
    float stlPitch = 0.0f, stlRoll = 0.0f;

    try {
        // 캘리브레이션 수행
        IMUCalibrationData calibrationData = calibrateIMU();

        // 캘리브레이션 완료 후 데이터 출력
        std::cout << "IMU Calibration offsets are set." << std::endl;

        // 캘리브레이션 데이터로 오프셋 값 설정
        offsetX = calibrationData.offsetX;
        offsetY = calibrationData.offsetY;
        offsetZ = calibrationData.offsetZ;

        offsetGyroX = calibrationData.offsetGyroX;
        offsetGyroY = calibrationData.offsetGyroY;
        offsetGyroZ = calibrationData.offsetGyroZ;

        // // 목표 각도는 0도로 설정하여 드론이 수평을 유지하도록 합니다.
        stlRoll = calibrationData.stlRoll;
        stlPitch = calibrationData.stlPitch;

    } catch (const std::exception &e) {
        std::cerr << "Error during calibration: " << e.what() << std::endl;
        return -1;
    }

    float targetRoll = stlRoll;
    float targetPitch = stlPitch;

    // PIDController 초기화 (output_limit을 MAX_ADJUSTMENT로 설정)
    // Arduino PID Gains
    float P_Gain = 5.0f;
    float P_Gain_Rate = 0.5f;

    float PID_I_Gain_Roll = 0.1f;
    float PID_I_Gain_Pitch = 0.1f;
    float D_Gain_Rate = 0.1f;

    float PID_Max_Roll = 0.5f;
    float PID_Max_Pitch = 0.5f;

    float PID_P_Gain_Yaw = 25.0f;
    float PID_I_Gain_Yaw = 0.0001f;
    float PID_D_Gain_Yaw = 0.03f;

    float PID_I_Max_Yaw = 150.0f;
    float PID_Max_Yaw = 150.0f;

    // PID Controllers 초기화
    PIDController outerRollPID(P_Gain, 0.0f, 0.0f, 0.0f, PID_Max_Roll, MAX_ADJUSTMENT);    // Outer Roll PID (Only P)
    PIDController outerPitchPID(P_Gain, 0.0f, 0.0f, 0.0f, PID_Max_Pitch, MAX_ADJUSTMENT); // Outer Pitch PID (Only P)
    PIDController yawPID(PID_P_Gain_Yaw, PID_I_Gain_Yaw, PID_D_Gain_Yaw, 0.0f, PID_I_Max_Yaw, PID_Max_Yaw); // Yaw PID

    // Inner PID Memory Variables
    float PID_I_Mem_Roll = 0.0f;
    float PID_I_Mem_Pitch = 0.0f;
    float PID_I_Mem_Yaw = 0.0f;

    float Roll_Rate_Err_Last = 0.0f;
    float Pitch_Rate_Err_Last = 0.0f;
    float PID_Last_Yaw_D_Error = 0.0f;

    float Roll_Rate_PID = 0.0f;
    float Pitch_Rate_PID = 0.0f;
    float Yaw_Rate_PID = 0.0f;

    float currentRoll = 0.0f;
    float currentPitch = 0.0f;

    while (true) {
        // 측정된 시간 차이를 계산 (여기서는 고정된 dt를 사용)
        float dt = 0.0025f; // 2.5ms 대기

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

        // 가속도계 데이터 보정
        float correctedAccelX = imuData.accelX - offsetX;
        float correctedAccelY = imuData.accelY - offsetY;
        float correctedAccelZ = imuData.accelZ - offsetZ;

        // 자이로스코프 데이터 보정
        float correctedGyroX = imuData.gyroX - offsetGyroX;
        float correctedGyroY = imuData.gyroY - offsetGyroY;
        float correctedGyroZ = imuData.gyroZ - offsetGyroZ;

        // 가속도계를 이용한 각도 계산
        float accelRoll = atan2(correctedAccelY, correctedAccelZ) * 180 / M_PI;
        float accelPitch = atan2(-correctedAccelX, sqrt(correctedAccelY * correctedAccelY + correctedAccelZ * correctedAccelZ)) * 180 / M_PI;

        // Yaw PID 컨트롤러에 보정된 자이로스코프 데이터 사용
        float yaw_err = (rudder_normalized * 1500.0f) - correctedGyroZ; // Assuming rudder_normalized maps to desired yaw rate
        Yaw_Rate_PID = yawPID.calculate(rudder_normalized * 1500.0f, correctedGyroZ, dt);

        // Pitch, Roll에 대한 각도 에러(Angle Error)
        float Pitch_Err = accelPitch - targetPitch;
        float Roll_Err = accelRoll - targetRoll;

        // Outer PID Control (Angle Control)
        float Pitch_PID = outerPitchPID.calculate(targetPitch, accelPitch, dt); // Only P control
        float Roll_PID = outerRollPID.calculate(targetRoll, accelRoll, dt);      // Only P control

        // Inner PID Control (Rate Control)
        // Pitch Rate Error
        float Pitch_Rate_Err = Pitch_PID + correctedGyroY;
        // Roll Rate Error
        float Roll_Rate_Err = Roll_PID + correctedGyroX;

        // 각속도 P 제어 (Inner P control)
        float Pitch_Rate_P = Pitch_Rate_Err * P_Gain_Rate;
        float Roll_Rate_P = Roll_Rate_Err * P_Gain_Rate;

        // 각속도 I 제어 (Inner I control)
        PID_I_Mem_Pitch += PID_I_Gain_Pitch * Pitch_Rate_Err;
        PID_I_Mem_Pitch = std::clamp(PID_I_Mem_Pitch, -PID_Max_Pitch, PID_Max_Pitch);

        PID_I_Mem_Roll += PID_I_Gain_Roll * Roll_Rate_Err;
        PID_I_Mem_Roll = std::clamp(PID_I_Mem_Roll, -PID_Max_Roll, PID_Max_Roll);

        // 각속도 D 제어 (Inner D control)
        float Pitch_Rate_D = (Pitch_Rate_Err - Pitch_Rate_Err_Last) * D_Gain_Rate;
        float Roll_Rate_D = (Roll_Rate_Err - Roll_Rate_Err_Last) * D_Gain_Rate;

        Pitch_Rate_Err_Last = Pitch_Rate_Err;
        Roll_Rate_Err_Last = Roll_Rate_Err;

        // 최종 PID 출력 계산
        Pitch_Rate_PID = Pitch_Rate_P + (PID_I_Mem_Pitch * 1.0f) + Pitch_Rate_D;
        Roll_Rate_PID = Roll_Rate_P + (PID_I_Mem_Roll * 1.0f) + Roll_Rate_D;

        // Yaw PID Control using only gyro
        float yaw_adj = Yaw_Rate_PID;

        if (fabs(accelRoll - targetRoll) < TOLERANCE_ROLL) {
            Roll_Rate_PID = 0.0f;
        }

        if (fabs(accelPitch - targetPitch) < TOLERANCE_PITCH) {
            Pitch_Rate_PID = 0.0f;
        }
        // 총 조정값 계산 (PID 출력만 사용)
        float total_yaw_adj = yaw_adj;
        float total_roll_adj = Roll_Rate_PID;
        float total_pitch_adj = Pitch_Rate_PID;

        int throttle_PWM = computeThrottlePWM(throttle_normalized);

        int motor1_PWM, motor2_PWM, motor3_PWM, motor4_PWM;

        if (throttle_PWM <= PWM_MIN) {
            // 스로틀 값이 최소값 이하일 경우 모든 모터 정지
            motor1_PWM = PWM_MIN;
            motor2_PWM = PWM_MIN;
            motor3_PWM = PWM_MIN;
            motor4_PWM = PWM_MIN;
        } else {
            // 모터 조정값 계산 (PID 출력 사용)
            int motor1_adj = static_cast<int>(total_roll_adj) - static_cast<int>(total_pitch_adj) + static_cast<int>(total_yaw_adj);
            int motor2_adj = -static_cast<int>(total_roll_adj) + static_cast<int>(total_pitch_adj) + static_cast<int>(total_yaw_adj);
            int motor3_adj = -static_cast<int>(total_roll_adj) - static_cast<int>(total_pitch_adj) - static_cast<int>(total_yaw_adj);
            int motor4_adj = static_cast<int>(total_roll_adj) + static_cast<int>(total_pitch_adj) - static_cast<int>(total_yaw_adj);

            // 조정값을 모터 PWM에 적용
            motor1_PWM = throttle_PWM + motor1_adj * 0.3;
            motor2_PWM = throttle_PWM + motor2_adj * 0.3;
            motor3_PWM = throttle_PWM + motor3_adj * 0.3;
            motor4_PWM = throttle_PWM + motor4_adj * 0.3;

            // 모터 PWM이 throttle_PWM보다 작을 수 없도록 설정
            motor1_PWM = std::max(motor1_PWM, throttle_PWM);
            motor2_PWM = std::max(motor2_PWM, throttle_PWM);
            motor3_PWM = std::max(motor3_PWM, throttle_PWM);
            motor4_PWM = std::max(motor4_PWM, throttle_PWM);

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
        usleep(LOOP_DELAY_US); // 2.5ms 대기
    }

    return 0;
}
