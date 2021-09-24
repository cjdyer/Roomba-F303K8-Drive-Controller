#include "MotorManager.h"

MotorManager::MotorManager(Motor* _motor, Encoder* _encoder, PID* _pid)
{
    motor_ = _motor;
    encoder_ = _encoder;
    pid_ = _pid;
}

int32_t& MotorManager::getEncoder()
{
    return encoder_->getPosition();
}

void MotorManager::drive(int16_t _speed)
{
    motor_->drive(_speed);
}

void MotorManager::brake()
{
    motor_->brake();
}
