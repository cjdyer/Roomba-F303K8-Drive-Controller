#include "MotorManager.h"

MotorManager motorManager;

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
    delay(5);
}