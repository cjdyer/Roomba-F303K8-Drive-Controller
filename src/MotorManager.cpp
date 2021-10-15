#include "MotorManager.h"
#include "pins.h"

MotorManager::MotorManager()
{
    void (*left_callback)() = encoderCallback(leftDrive_.encoder_);
    void (*right_callback)() = encoderCallback(rightDrive_.encoder_);

    leftDrive_.encoder_ = new Encoder(encoderLA, encoderLB);
    rightDrive_.encoder_ = new Encoder(encoderRB, encoderRA);

    leftDrive_.motor_ = new Motor(BIN1, BIN2, PWMB, STBY);
    rightDrive_.motor_ = new Motor(AIN1, AIN2, PWMA, STBY);

    leftDrive_.pid_ = new PID(2, 0.0005, 0);
    rightDrive_.pid_ = new PID(2, 0.0005, 0);

    attachInterrupt(encoderLA, left_callback, CHANGE);
    attachInterrupt(encoderLB, left_callback, CHANGE);
    attachInterrupt(encoderRA, right_callback, CHANGE);
    attachInterrupt(encoderRB, right_callback, CHANGE);
}

void MotorManager::driveTo(int16_t _x, int16_t _y)
{
    // pid_->reset();
    // pid_->setTarget(target);
    // active_ = true;
}

void MotorManager::run()
{
    if (active_)
    {
        int32_t left_encoder_value = leftDrive_.encoder_->getPosition();
        int32_t right_encoder_value = rightDrive_.encoder_->getPosition();

        int16_t left_pid_value = leftDrive_.pid_->calculate(left_encoder_value);
        int16_t right_pid_value = rightDrive_.pid_->calculate(right_encoder_value);

        leftDrive_.motor_->drive(left_pid_value);
        rightDrive_.motor_->drive(right_pid_value);

        if(leftDrive_.pid_->done() && rightDrive_.pid_->done())
        {
            leftDrive_.motor_->drive(0);
            rightDrive_.motor_->drive(0);
            active_ = false;
        }
    }
}

void (*encoderCallback(Encoder *_encoder))() { _encoder->encoderTick(); }