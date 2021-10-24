#include "MotorManager.h"

MotorManager motorManager;

void setup()
{
    Serial.begin(115200);
    while (!Serial);
    Serial.println("Serial Init");
}

void loop()
{
    // Run motors/PIDs
    motorManager.run();
}