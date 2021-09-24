// Standard Includes
#include <Arduino.h>
#include "pins.h"
#include "Encoder.h"
#include "Motor.h"
#include "PID.h"
#include "MotorManager.h"

Encoder LeftEncoder(encoderLA, encoderLB);
Encoder RightEncoder(encoderRB, encoderRA); 

void leftEncoderCallback(void) { LeftEncoder.encoderTick();  }
void rightEncoderCallback(void) { RightEncoder.encoderTick();  } // This has to be defined here as the encoders exist in this scope.

Motor LeftMotor(BIN1, BIN2, PWMB, STBY);
Motor RightMotor(AIN1, AIN2, PWMA, STBY);

PID LeftPID(2,0.0005, 0);
PID RightPID(2,0.0005, 0);

MotorManager LeftMotorManager(&LeftMotor, &LeftEncoder, &LeftPID);
MotorManager RightMotorManager(&RightMotor, &RightEncoder, &RightPID);

void setup()
{
	Serial.begin(115200);

	while (!Serial) { }

    attachInterrupt(encoderLA, leftEncoderCallback, CHANGE);
    attachInterrupt(encoderLB, leftEncoderCallback, CHANGE);
    attachInterrupt(encoderRA, rightEncoderCallback, CHANGE);
    attachInterrupt(encoderRB, rightEncoderCallback, CHANGE);
}

void loop()
{

}