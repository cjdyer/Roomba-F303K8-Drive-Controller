#ifndef MOTOR_MANAGER_H
#define MOTOR_MANAGER_H

#include <Arduino.h>
#include "Motor.h"
#include "PID.h"
#include "Encoder.h"

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
    } left_drive_, right_drive_;

private:
    bool active_;
};

void (*encoderCallback(Encoder *_encoder))();

#endif