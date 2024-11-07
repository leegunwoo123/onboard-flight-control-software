#ifndef MOTERCONTROL_H
#define MOTERCONTROL_H

#include <cstdint>

#define PCA9685_ADDR 0x40  // PCA9685 I2C 주소
#define MODE1 0x00         // 모드1 레지스터
#define PRESCALE 0xFE      // 프리스케일 레지스터
#define LED0_ON_L 0x06     // 첫 번째 채널 ON 낮은 바이트 레지스터
#define LED0_OFF_L 0x08    // 첫 번째 채널 OFF 낮은 바이트 레지스터

// PCA9685 초기화 함수: 주파수를 설정하여 PCA9685를 초기화합니다.
void initPCA9685(int pwm_freq);

// 특정 채널에 PWM 신호를 설정하는 함수
void setPWM(int channel, int on, int off);

// RC 채널 값을 PWM 값으로 변환하는 함수
int mapChannelValueToPWM(int value);

// 0~3번 채널의 모터를 제어하는 함수
void controlMotors();

#endif // MOTERCONTROL_H