#include "pins.h"
#include "MotorManager.h"
#include "SerialManager.h"

MotorManager motorManager;
SerialManager serialManager;

void setup()
{
    Serial.begin(115200);
    while (!Serial);
    motorManager.driveTo(1000);
}

void loop()
{
    // Run motors/PIDs
    
    motorManager.run();
}