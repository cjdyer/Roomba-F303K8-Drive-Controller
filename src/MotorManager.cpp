#include "MotorManager.h"

MotorManager::MotorManager(Motor* _motor, Encoder* _encoder, PID* _pid)
{
    motor_ = _motor;
    encoder_ = _encoder;
    pid_ = _pid;
}

void MotorManager::printEncoder()
{
    Serial.print(encoder_->getPosition());
}

void MotorManager::drive(int16_t _speed)
{
    motor_->drive(_speed, _forwards);
}

void MotorManager::brake()
{
    motor_->brake();
}
