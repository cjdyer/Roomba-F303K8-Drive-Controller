#include "MotorManager.h"
#include "pins.h"

void (*encoderCallback(Encoder *_encoder))() { _encoder->encoderTick(); }

MotorManager::MotorManager()
{
    leftDrive_.encoder_ = new Encoder(encoderLA, encoderLB);
    rightDrive_.encoder_ = new Encoder(encoderRB, encoderRA);

    leftDrive_.motor_ = new Motor(BIN1, BIN2, PWMB, STBY);
    rightDrive_.motor_ = new Motor(AIN1, AIN2, PWMA, STBY);

    leftDrive_.pid_ = new PID(2, 0.0005, 0);
    rightDrive_.pid_ = new PID(2, 0.0005, 0);
}

void MotorManager::attachInterrupts()
{
    void (*left_callback)() = encoderCallback(leftDrive_.encoder_);
    void (*right_callback)() = encoderCallback(rightDrive_.encoder_);

    attachInterrupt(encoderLA, left_callback, CHANGE);
    attachInterrupt(encoderLB, left_callback, CHANGE);
    attachInterrupt(encoderRA, right_callback, CHANGE);
    attachInterrupt(encoderRB, right_callback, CHANGE);
}

void MotorManager::driveTo(int16_t _x, int16_t _y)
{
    leftDrive_.pid_->reset();
    rightDrive_.pid_->reset();
    leftDrive_.pid_->setTarget(_x);
    rightDrive_.pid_->setTarget(_x);
    active_ = true;
}

void MotorManager::run(const uint32_t _dt, const float _gyroscope)
{
    if (active_)
    {
        int32_t left_encoder_value = leftDrive_.encoder_->getPosition();
        int32_t right_encoder_value = rightDrive_.encoder_->getPosition();

        calculatePosition(left_encoder_value, right_encoder_value, _gyroscope, _dt);

        // Code goes here.....

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

void MotorManager::getPosition(float &_x, float &_y)
{
    _x = global_x_;
    _y = global_y_;
}

void MotorManager::calculatePosition(const int32_t _left_encoder,const int32_t _right_encoder, const float _gyroscope, const uint32_t _dt)
{
    static int32_t previous_left_encoder = 0, previous_right_encoder = 0;

    int32_t change_x = _left_encoder - previous_left_encoder;
    int32_t change_y = _right_encoder - previous_right_encoder;

    previous_left_encoder = _left_encoder;
    previous_right_encoder = _right_encoder;

    float linear_velocity = (change_x + change_y) / 2;

    float linear_velocity_x = linear_velocity * sin(_gyroscope);
    float linear_velocity_y = linear_velocity * cos(_gyroscope);

    global_x_ += linear_velocity_x * _dt;
    global_y_ += linear_velocity_y * _dt;
}
