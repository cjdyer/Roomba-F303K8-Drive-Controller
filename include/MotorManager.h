#ifndef MOTOR_MANAGER_H
#define MOTOR_MANAGER_H

#include "Arduino.h"

#include "Encoder.h"
#include "Motor.h"
#include "PID.h"
#include "pins.h"

class MotorManager
{
public:
    MotorManager();

    void driveTo(const int16_t _distance);
    void rotateTo(const int16_t _angle);
    void run();

private:
    struct Drive {
        Encoder* encoder_;
        Motor* motor_;
        PID* pid_;
    } leftDrive_, rightDrive_;

private:
    bool active_;
};

#endif