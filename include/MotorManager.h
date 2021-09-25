#ifndef MOTOR_MANAGER_H
#define MOTOR_MANAGER_H

#include <Arduino.h>
#include "Motor.h"
#include "PID.h"
#include "Encoder.h"
#include "pins.h"

class MotorManager
{
public:
    MotorManager(const bool _left);

    void driveTo(int16_t _target);
    void run();

public:
    Encoder* encoder_;
    Motor* motor_;
    PID* pid_;

private:
    bool active_;
};

void (*encoderCallback(MotorManager *_motorManager))();

#endif