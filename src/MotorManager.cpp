#include "MotorManager.h"
#include "pins.h"

MotorManager::MotorManager()
{
    void (*left_callback)() = encoderCallback(left_drive_.encoder_);
    void (*right_callback)() = encoderCallback(right_drive_.encoder_);

    left_drive_.encoder_ = new Encoder(encoderLA, encoderLB);
    right_drive_.encoder_ = new Encoder(encoderRB, encoderRA);

    left_drive_.motor_ = new Motor(BIN1, BIN2, PWMB, STBY);
    right_drive_.motor_ = new Motor(AIN1, AIN2, PWMA, STBY);

    left_drive_.pid_ = new PID(2, 0.0005, 0);
    right_drive_.pid_ = new PID(2, 0.0005, 0);

    attachInterrupt(encoderLA, left_callback, CHANGE);
    attachInterrupt(encoderLB, left_callback, CHANGE);
    attachInterrupt(encoderRA, right_callback, CHANGE);
    attachInterrupt(encoderRB, right_callback, CHANGE);
}

void MotorManager::driveTo(int16_t _x, int16_t _y)
{

    // pid_->reset();
    // pid_->setTarget(_target);
    // active_ = true;
}

void MotorManager::run()
{
    // if (active_)
    // {
    //     int32_t encoder_value = encoder_->getPosition();
    //     double pid_value = pid_->calculate(encoder_value);
    //     motor_->drive(pid_value);

    //     if(pid_->done())
    //     {
    //         active_ = false;
    //         motor_->drive(0);
    //     }
    // }
}

void (*encoderCallback(Encoder *_encoder))() { _encoder->encoderTick(); }