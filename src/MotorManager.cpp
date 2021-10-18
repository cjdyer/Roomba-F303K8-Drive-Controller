#include "MotorManager.h"
#include "pins.h"

Encoder* leftEncoder = new Encoder(ENCODER_LA, ENCODER_LB);
Encoder* rightEncoder = new Encoder(ENCODER_RA, ENCODER_RB);

void leftEncoderCallback() { leftEncoder->encoderTick(); }
void rightEncoderCallback() { rightEncoder->encoderTick(); }

MotorManager::MotorManager()
{
    leftDrive_.encoder_ = leftEncoder;
    rightDrive_.encoder_ = rightEncoder;

    leftDrive_.motor_ = new Motor(AIN2, AIN1, PWMA, STBY);
    rightDrive_.motor_ = new Motor(BIN2, BIN1, PWMB, STBY);

    leftDrive_.pid_ = new PID(2, 0.00025, 0.12);
    rightDrive_.pid_ = new PID(2, 0.00025, 0.12);

    attachInterrupt(ENCODER_LA, leftEncoderCallback, CHANGE);
    attachInterrupt(ENCODER_LB, leftEncoderCallback, CHANGE);
    attachInterrupt(ENCODER_RA, rightEncoderCallback, CHANGE);
    attachInterrupt(ENCODER_RB, rightEncoderCallback, CHANGE);
}

void MotorManager::driveTo(const int16_t _distance)
{
    leftDrive_.pid_->reset();
    leftDrive_.pid_->setTarget(_distance);
    
    rightDrive_.pid_->reset();
    rightDrive_.pid_->setTarget(_distance);
    
    active_ = true;
}

void MotorManager::rotateTo(const int16_t _angle)
{
    leftDrive_.pid_->reset();
    leftDrive_.pid_->setTarget(_angle);

    rightDrive_.pid_->reset();
    rightDrive_.pid_->setTarget(-_angle);
    active_ = true;
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
            leftDrive_.motor_->brake();
            rightDrive_.motor_->brake();
            active_ = false;
        }
    }
}