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

    void driveTo(int16_t _x, int16_t _y);
    void run();

public:
    struct Drive {
        Encoder* encoder_;
        Motor* motor_;
        PID* pid_;
    } leftDrive_, rightDrive_;

private:
    bool active_;
};

void (*encoderCallback(Encoder *_encoder))();

#endif