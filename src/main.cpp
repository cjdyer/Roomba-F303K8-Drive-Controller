#include "pins.h"
#include "MotorManager.h"
#include "Encoder.h"

Encoder leftEncoder(encoderLA, encoderLB);
Encoder rightEncoder(encoderRA, encoderRB);
MotorManager motorManager(&leftEncoder, &rightEncoder);

void leftEncoderCallback() { leftEncoder.encoderTick(); }
void rightEncoderCallback() { rightEncoder.encoderTick(); }

void setup()
{
    Serial.begin(115200);
    while (!Serial);

    attachInterrupt(ENCODER_LA, leftEncoderCallback, CHANGE);
    attachInterrupt(ENCODER_LB, leftEncoderCallback, CHANGE);
    attachInterrupt(ENCODER_RA, rightEncoderCallback, CHANGE);
    attachInterrupt(ENCODER_RB, rightEncoderCallback, CHANGE);
}

void loop()
{
    // Run motors/PIDs
    motorManager.run();
}