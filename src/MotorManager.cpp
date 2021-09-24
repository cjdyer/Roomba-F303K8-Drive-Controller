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

void MotorManager::driveTo(int16_t _target)
{
    pid_->reset();
    pid_->setTarget(_target);
}

void MotorManager::run()
{
    int32_t& encoder_value = encoder_->getPosition();
    double pid_value = pid_->calculate(encoder_value);
    motor_->drive(pid_value);
}