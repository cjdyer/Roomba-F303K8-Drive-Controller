#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class Motor
{
public:
    Motor(const uint8_t& _input_1_pin, const uint8_t& _input_2_pin, const uint8_t& _pwm_pin, const uint8_t& _enable_pin);

    void drive(const int16_t _speed);
    void brake();

private:
    const uint8_t input_1_pin_;
    const uint8_t input_2_pin_;
    const uint8_t pwm_pin_; 
    const uint8_t enable_pin_;
};

#endif