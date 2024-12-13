#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>          // 메모리 잠금
#include <linux/i2c-dev.h>
#include <cstdint>
#include <cmath>               // 수학 함수 사용 (atan2, sqrt, M_PI)
#include <Eigen/Dense>         // Eigen 라이브러리
#include <pthread.h>           // POSIX 스레드
#include <time.h>              // 고해상도 타이머
#include "rc_input.h"          // RC 입력 헤더 파일
#include "imu_sensor.h"        // IMU 센서 헤더 포함
#include "imu_calibration.h"   // IMU 캘리브레이션 헤더 파일
#include <termios.h>           // B115200 설정
#include <algorithm>           // std::clamp 함수 사용
#include <chrono>              // 시간 측정을 위한 라이브러리
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>


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
const int MAX_ADJUSTMENT = 10; // 각 제어 입력의 최대 PWM 조정 값
const int I2C_RETRY_LIMIT = 3; // I2C 오류 시 재시도 횟수
const int SAFE_PWM = PWM_MIN;  // 초기화 및 안전한 PWM 값
const float TOLERANCE_ROLL = 1;   // 롤 허용 오차 (0.45도)
const float TOLERANCE_PITCH = 1;  // 피치 허용 오차 (0.45도)

class PCA9685 {
public:
    PCA9685(int address = PCA9685_ADDR) {
        char filename[20];
        snprintf(filename, 19, "/dev/i2c-1");
        fd = open(filename, O_RDWR);
        if (fd < 0) {
            throw std::runtime_error("Failed to open the i2c bus");
        }
        if (ioctl(fd, I2C_SLAVE, address) < 0) {
            throw std::runtime_error("Failed to acquire bus access");
        }
        reset();
        setPWMFreq(50);  // Set frequency to 50Hz for motor control
    }

    ~PCA9685() {
        stopAllMotors();
        if (fd >= 0) {
            close(fd);
        }
    }

    void setAllMotorsSpeeds(const int pwm_values[4]) {
        uint8_t buffer[17];
        buffer[0] = LED0_ON_L;

        for (int i = 0; i < 4; i++) {
            int pwm = std::clamp(pwm_values[i], PWM_MIN, PWM_MAX);
            buffer[1 + i * 4] = 0;
            buffer[2 + i * 4] = 0;
            buffer[3 + i * 4] = pwm & 0xFF;
            buffer[4 + i * 4] = pwm >> 8;
        }

        asyncWrite(buffer, sizeof(buffer));
    }

private:
    int fd;

    void reset() {
        writeRegister(MODE1, 0x00);
    }

    void setPWMFreq(int freq) {
        double oscillator_freq = 25000000.0;
        double scale_factor = oscillator_freq / (4096.0 * freq);
        uint8_t prescale = static_cast<uint8_t>(round(scale_factor - 1.0));
        uint8_t oldmode = readRegister(MODE1);
        writeRegister(MODE1, (oldmode & 0x7F) | 0x10);
        writeRegister(PRESCALE, prescale);
        writeRegister(MODE1, oldmode);
        usleep(5000);
        writeRegister(MODE1, oldmode | 0xA1);
    }

    void asyncWrite(uint8_t* buffer, size_t length) {
        std::thread([=] {
            if (write(fd, buffer, length) != length) {
                std::cerr << "I2C write failed" << std::endl;
            }
        }).detach();
    }

    void writeRegister(uint8_t reg, uint8_t value) {
        uint8_t buffer[2] = {reg, value};
        asyncWrite(buffer, 2);
    }

    uint8_t readRegister(uint8_t reg) {
        if (write(fd, &reg, 1) != 1) {
            throw std::runtime_error("I2C read failed");
        }

        uint8_t value;
        if (read(fd, &value, 1) != 1) {
            throw std::runtime_error("I2C read failed");
        }
        return value;
    }

    void stopAllMotors() {
        int stop_pwm[4] = {SAFE_PWM, SAFE_PWM, SAFE_PWM, SAFE_PWM};
        setAllMotorsSpeeds(stop_pwm);
    }
};

struct PIDController {
    float kp, ki, kd;
    float prev_error;
    float integral;
    float integral_limit;
    float output_limit;
    float feedforward;
    float filtered_derivative; 
    float alpha;               

    // outlimit 설정 400->10로 기본 세팅
    PIDController(float p, float i, float d, float ff = 0.0f, float i_limit = 10.0f, float out_limit = 10.0f, float filter_alpha = 0.1f)
        : kp(p), ki(i), kd(d), feedforward(ff), prev_error(0.0f), integral(0.0f),
          integral_limit(i_limit), output_limit(out_limit), filtered_derivative(0.0f), alpha(filter_alpha) {}

    void reset() {
        prev_error = 0.0f;
        integral = 0.0f;
        filtered_derivative = 0.0f;
    }

    float calculate(float setpoint, float measurement, float dt) {
        if (dt <= 0.0f) {
            // dt가 0 이하일 경우, 계산을 중단
            return 0.0f;
        }

        // 오차 계산
        float error = setpoint - measurement;

        // 비례 항
        float pTerm = kp * error;

        // 적분 항 (적분 제한 적용)
        integral += error * dt;
        integral = std::clamp(integral, -integral_limit, integral_limit);
        float iTerm = ki * integral;

        // 미분 항 (필터링 적용)
        float derivative = (error - prev_error) / dt;
        filtered_derivative = alpha * derivative + (1.0f - alpha) * filtered_derivative;
        float dTerm = kd * filtered_derivative;

        // 이전 오차 업데이트
        prev_error = error;

        // PID 출력 (피드포워드 포함)
        float output = feedforward * setpoint + pTerm + iTerm + dTerm;

        // 출력 제한
        output = std::clamp(output, -output_limit, output_limit);

        return output;
    }
};

// 스로틀 값을 0.0 ~ 1.0 범위로 매핑하는 함수
double mapThrottle(int value) {
    if (value <= RC_MIN) return 0.0;
    if (value >= RC_MAX) return 1.0;
    return static_cast<double>(value - RC_MIN) / (RC_MAX - RC_MIN);
}

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

// 공유 데이터 구조
std::mutex motorMutex;
std::condition_variable motorCv;
std::queue<std::array<int, 4>> motorCommandQueue;

// 모터 업데이트 스레드
void *motorUpdateThread(void *arg) {
    PCA9685 pca9685;

    while (true) {
        std::unique_lock<std::mutex> lock(motorMutex);
        motorCv.wait(lock, [] { return !motorCommandQueue.empty(); });

        // 큐에서 명령 가져오기
        auto motorCommand = motorCommandQueue.front();
        motorCommandQueue.pop();
        lock.unlock();

        // 모터 속도 설정
        pca9685.setAllMotorsSpeeds(motorCommand.data());
    }

    return nullptr;
}

// IMU 데이터와 mutex 정의
IMUData imuData;
std::mutex imuQueueMutex;
std::condition_variable imuDataCv;

// IMU 스레드 함수
void *imuThread(void *arg) {
    initIMU("/dev/ttyUSB0", B115200);
    IMUCalibrationData calibrationData = calibrateIMU();
    float offsetGyroZ = calibrationData.offsetGyroZ;

    while (true) {
        IMUData localIMUData = readIMU();
        localIMUData.gyroZ -= offsetGyroZ; // 보정된 자이로 Z값

        // IMU 데이터를 큐에 추가
        {
            std::lock_guard<std::mutex> lock(imuQueueMutex);
            imuDataQueue.push(localIMUData);
        }
        imuDataCv.notify_one(); // 데이터 추가 알림

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return nullptr;
}

// PID 계산 스레드 (수정된 controlLoop 함수)
void *controlLoop(void *arg) {
    PIDController rollPID(1.5f, 0.0f, 1.0f);
    PIDController pitchPID(2.0f, 0.5f, 0.2f);
    PIDController yawPID(1.2f, 0.5f, 0.5f);

    float roll_com = 0, pitch_com = 0;
    auto previousTime = std::chrono::steady_clock::now();

    while (true) {
        IMUData imuData;

        // 큐에서 IMU 데이터 읽기
        {
            std::unique_lock<std::mutex> lock(imuQueueMutex);
            imuDataCv.wait(lock, [] { return !imuDataQueue.empty(); }); // 큐에 데이터가 있을 때까지 대기

            imuData = imuDataQueue.front();
            imuDataQueue.pop();
        }

        auto currentTime = std::chrono::steady_clock::now();
        std::chrono::duration<float> elapsed = currentTime - previousTime;
        float dt = elapsed.count();
        previousTime = currentTime;
        std::cout << dt << " ms" << std::endl;

        // IMU 데이터 사용
        float roll_angle = imuData.roll_angle;
        float pitch_angle = imuData.pitch_angle;
        float yaw_angle = imuData.yaw_angle;

        int throttle_value = readRCChannel(3);
        double throttle_normalized = mapThrottle(throttle_value);

        // PID 계산
        int throttle_PWM = computeThrottlePWM(throttle_normalized);
        int motor1_PWM, motor2_PWM, motor3_PWM, motor4_PWM;

        if (throttle_PWM <= PWM_MIN) {
            motor1_PWM = motor2_PWM = motor3_PWM = motor4_PWM = PWM_MIN;
        } else {
            int roll_adj = rollPID.calculate(roll_com, roll_angle, dt);
            int pitch_adj = pitchPID.calculate(pitch_com, pitch_angle, dt);

            int aileron_adj = computeAdjustment(mapControlInput(readRCChannel(1)));
            int elevator_adj = computeAdjustment(mapControlInput(readRCChannel(2)));

            motor1_PWM = clamp(throttle_PWM - aileron_adj + elevator_adj, PWM_MIN, PWM_MAX);
            motor2_PWM = clamp(throttle_PWM + aileron_adj - elevator_adj, PWM_MIN, PWM_MAX);
            motor3_PWM = clamp(throttle_PWM + aileron_adj + elevator_adj, PWM_MIN, PWM_MAX);
            motor4_PWM = clamp(throttle_PWM - aileron_adj - elevator_adj, PWM_MIN, PWM_MAX);
        }

        // 모터 명령 큐에 추가
        {
            std::lock_guard<std::mutex> lock(motorMutex);
            motorCommandQueue.push({motor1_PWM, motor2_PWM, motor3_PWM, motor4_PWM});
        }
        motorCv.notify_one();

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    return nullptr;
}


int main() {
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
        std::cerr << "Failed to lock memory!" << std::endl;
        return -1;
    }

    pthread_t imuThreadHandle, controlThreadHandle, motorThreadHandle;

    // IMU 스레드 생성
    if (pthread_create(&imuThreadHandle, nullptr, imuThread, nullptr) != 0) {
        std::cerr << "Failed to create IMU thread!" << std::endl;
        return -1;
    }

    // PID 계산 스레드 생성
    if (pthread_create(&controlThreadHandle, nullptr, controlLoop, nullptr) != 0) {
        std::cerr << "Failed to create Control thread!" << std::endl;
        return -1;
    }

    // 모터 업데이트 스레드 생성
    if (pthread_create(&motorThreadHandle, nullptr, motorUpdateThread, nullptr) != 0) {
        std::cerr << "Failed to create Motor Update thread!" << std::endl;
        return -1;
    }

    // 모든 스레드 종료 대기
    pthread_join(imuThreadHandle, nullptr);
    pthread_join(controlThreadHandle, nullptr);
    pthread_join(motorThreadHandle, nullptr);

    return 0;
}