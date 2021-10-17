#ifndef MOTOR_MANAGER_H
#define MOTOR_MANAGER_H

#include <Arduino.h>

#include "Encoder.h"
#include "Motor.h"
#include "PID.h"

class MotorManager
{
public:
    MotorManager(Encoder* _left_encoder, Encoder* _right_encoder);

    void driveTo(const int16_t _distance);
    void rotateTo(const int16_t _angle);
    void run();
    void setActive(const bool _active_state);
    
    void leftEncoderCallback();
    void rightEncoderCallback();

private:
    struct Drive {
        Encoder* encoder_;
        Motor* motor_;
        PID* pid_;
    } leftDrive_, rightDrive_;

private:
    bool active_;
};

void (*encoderCallback(Encoder *_encoder))(void);

#endif