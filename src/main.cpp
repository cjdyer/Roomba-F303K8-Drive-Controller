#include "MotorManager.h"

MotorManager motorManager;

void setup()
{
    Serial.begin(115200);
    while (!Serial);
}

void loop()
{
    // Run motors/PIDs
    motorManager.run();
}