#ifndef MOTOR_MANAGER_H
#define MOTOR_MANAGER_H

#include <Arduino.h>
#include "Motor.h"
#include "PID.h"
#include "Encoder.h"

class MotorManager
{
public:
    MotorManager(Motor* _motor, Encoder* _encoder, PID* _pid);

    int32_t& getEncoder();
    void drive(int16_t _speed);
    void brake();

private:
    Motor* motor_;
    Encoder* encoder_;
    PID* pid_;
};

#endif