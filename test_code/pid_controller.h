#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <cstdint>

#define PCA9685_ADDR 0x40
#define MODE1 0x00
#define PRESCALE 0xFE
#define LED0_ON_L 0x06
#define LED0_OFF_L 0x08

class PCA9685 {
public:
    PCA9685(int address = PCA9685_ADDR);
    ~PCA9685();
    void setMotorSpeed(int channel, int pwm_value);

private:
    int fd;
    void reset();
    void setPWMFreq(int freq);
    void setPWM(int channel, int on, int off);
    void writeRegister(uint8_t reg, uint8_t value);
    uint8_t readRegister(uint8_t reg);
    void initializeMotors();
    void stopAllMotors();
};

#endif // MOTOR_CONTROL_H
