// Standard Includes
#include <Arduino.h>
#include "pins.h"
#include "Encoder.h"
#include "Motor.h"
#include "PID.h"
#include "MotorManager.h"

MotorManager LeftMotorManager(true); // hefty overhead....
MotorManager RightMotorManager(false); 

void setup()
{
	Serial.begin(115200);

	while (!Serial) { }
}

void loop()
{
    delayMicroseconds(10);
}