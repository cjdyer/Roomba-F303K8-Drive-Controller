#include "MotorManager.h"


MotorManager::MotorManager(const bool _left)
{
    if (_left)
    {
        encoder_ = new Encoder(encoderLA, encoderLB);
        motor_ = new Motor(BIN1, BIN2, PWMB, STBY);
        pid_ = new PID(2, 0.0005, 0);

        void (*pointer_to_callback)() = encoderCallback(this);
        attachInterrupt(encoderLA, pointer_to_callback, CHANGE);
        attachInterrupt(encoderLB, pointer_to_callback, CHANGE);
    }
    else
    {
        encoder_ = new Encoder(encoderRB, encoderRA);
        motor_ = new Motor(AIN1, AIN2, PWMA, STBY);;
        pid_ = new PID(2, 0.0005, 0);

        void (*pointer_to_callback)() = encoderCallback(this);
        attachInterrupt(encoderRA, pointer_to_callback, CHANGE);
        attachInterrupt(encoderRB, pointer_to_callback, CHANGE);
    }
}

void MotorManager::driveTo(int16_t _target)
{
    pid_->reset();
    pid_->setTarget(_target);
    active_ = true;
}

void MotorManager::run()
{
    if (active_)
    {
        int32_t encoder_value = encoder_->getPosition();
        double pid_value = pid_->calculate(encoder_value);
        motor_->drive(pid_value);

        if(pid_->done())
        {
            active_ = false;
            motor_->drive(0);
        }
    }
}

void (*encoderCallback(MotorManager *_motorManager))() { _motorManager->encoder_->encoderTick(); }