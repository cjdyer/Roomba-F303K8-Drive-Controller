// Standard Includes
#include <Arduino.h>
#include "pins.h"
#include "Encoder.h"
#include "Motor.h"
#include "PID.h"
#include "MotorManager.h"

Encoder leftEncoder(encoderLA, encoderLB); // Left Tachometer Object (Will be integrated into the motor class)
Encoder rightEncoder(encoderRB, encoderRA); // Right Tachometer Object (Will be integrated into the motor class)

void leftEncoder_callback(void) { leftEncoder.encoderTick();  }
void rightEncoder_callback(void) { rightEncoder.encoderTick();  }

Motor leftMotor(BIN1, BIN2, PWMB, STBY);
Motor rightMotor(AIN1, AIN2, PWMA, STBY);

PID leftPID(0,0,0,0);
PID rightPID(0,0,0,0);

MotorManager leftMotorManager(&leftMotor, &leftEncoder, &leftPID);
MotorManager rightMotorManager(&rightMotor, &rightEncoder, &rightPID);

void setup()
{
	Serial.begin(115200);

	while (!Serial) { }

    attachInterrupt(encoderLA, leftEncoder_callback, CHANGE);
    attachInterrupt(encoderLB, leftEncoder_callback, CHANGE);
    attachInterrupt(encoderRA, rightEncoder_callback, CHANGE);
    attachInterrupt(encoderRB, rightEncoder_callback, CHANGE);
}

void loop()
{
    // leftMotorManager.printEncoder();
    // Serial.print("\t");
    // rightMotorManager.printEncoder();
    // Serial.println();

    leftMotorManager.drive(255);
    for (int i = 0; i < 500; i++)
    {
        delay(10);
    }
    leftMotorManager.brake();
    for (int i = 0; i < 50; i++)
    {
        delay(10);
    }
    leftMotorManager.drive(-255);
    for (int i = 0; i < 500; i++)
    {
        delay(10);
    }
    leftMotorManager.brake();
    for (int i = 0; i < 50; i++)
    {
        delay(10);
    }
    
}