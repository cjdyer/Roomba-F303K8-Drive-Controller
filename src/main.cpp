// Standard Includes
#include <Arduino.h>
#include "pins.h"
#include "Encoder.h"
#include "Motor.h"
#include "PID.h"
#include "MotorManager.h"

Encoder leftEncoder(encoderLA, encoderLB);
Encoder rightEncoder(encoderRB, encoderRA); 

void leftEncoder_callback(void) { leftEncoder.encoderTick();  }
void rightEncoder_callback(void) { rightEncoder.encoderTick();  } // This has to be defined here as the encoders exist in this scope.

Motor leftMotor(BIN1, BIN2, PWMB, STBY);
Motor rightMotor(AIN1, AIN2, PWMA, STBY);

PID leftPID(2,0.0005, 0);
PID rightPID(2,0.0005, 0);

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
    leftPID.SetTarget(1000);
    rightPID.SetTarget(1000);
    Serial.print("LEFT Encoder Value : ");
    Serial.print(leftMotorManager.getEncoder());
    Serial.print("\tOutput Value : ");
    Serial.print(leftPID.Calculate(leftMotorManager.getEncoder()));
    Serial.print("\t RIGHT Encoder Value : ");
    Serial.print(rightMotorManager.getEncoder());
    Serial.print("\tOutput Value : ");
    Serial.println(rightPID.Calculate(rightMotorManager.getEncoder()));

    leftMotorManager.drive(-leftPID.Calculate(leftMotorManager.getEncoder()));
    rightMotorManager.drive(-rightPID.Calculate(rightMotorManager.getEncoder()));
}