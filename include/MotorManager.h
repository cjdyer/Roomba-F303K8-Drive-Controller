#ifndef MOTOR_MANAGER_H
#define MOTOR_MANAGER_H

#include <Arduino.h>

#include "Encoder.h"
#include "Motor.h"
#include "PID.h"

class MotorManager
{
public:
    MotorManager();

    void driveTo(const int16_t _x, const int16_t _y);
    void run(const uint32_t _dt, const float _gyroscope);
    void setActive(const bool _active_state);
    void getPosition(float &_x, float &_y);
    void attachInterrupts();

private:
    struct Drive {
        Encoder* encoder_;
        Motor* motor_;
        PID* pid_;
    } leftDrive_, rightDrive_;

    void calculatePosition(const int32_t _left_encoder,const int32_t _right_encoder, const float _gyroscope, const uint32_t _dt);

private:
    int32_t target_x_, target_y_;
    float global_x_, global_y_;
    bool active_;
};

void (*encoderCallback(Encoder *_encoder))(void);

#endif