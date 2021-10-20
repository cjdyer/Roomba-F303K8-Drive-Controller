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
    void run();

private:
    void rotateTo(const int16_t _angle);
    void driveTo(const int16_t _distance);


private:
    bool active_;
    
    struct Drive {
        Encoder* encoder_;
        Motor* motor_;
        PID* pid_;
    } leftDrive_, rightDrive_;

};

#endif