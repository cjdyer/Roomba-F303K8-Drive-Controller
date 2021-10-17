#include "Motor.h"

Motor::Motor(const uint8_t& _input_1_pin, const uint8_t& _input_2_pin, const uint8_t& _pwm_pin, const uint8_t& _enable_pin) : 
input_1_pin_(_input_1_pin), input_2_pin_(_input_2_pin), pwm_pin_(_pwm_pin), enable_pin_(_enable_pin)
{
    pinMode(input_1_pin_, OUTPUT);
    pinMode(input_2_pin_, OUTPUT);
    pinMode(pwm_pin_, OUTPUT);
    pinMode(enable_pin_, OUTPUT);
    digitalWrite(enable_pin_, HIGH);
}

void Motor::drive(const int16_t _speed)
{
    bool direction = (_speed > 0);
    digitalWrite(input_1_pin_, direction * HIGH);
    digitalWrite(input_2_pin_, !direction * HIGH);
    analogWrite(pwm_pin_, (_speed * direction) - (_speed * !direction));
}

void Motor::brake()
{
   digitalWrite(input_1_pin_, HIGH);
   digitalWrite(input_2_pin_, HIGH);
   analogWrite(pwm_pin_, 0);
}