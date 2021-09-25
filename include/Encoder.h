#ifndef TACHO_WHEEL_H
#define TACHO_WHEEL_H

#include <Arduino.h>

class Encoder
{
public:
    Encoder(const uint8_t &_pinA, const uint8_t &_pinB);

    int32_t getPosition();
    void encoderTick();

public:
    const uint8_t pinA_;
    const uint8_t pinB_;

private:
    const int8_t QEM_[4][4] = {{ 0,-1, 1, 2 },
                               { 1, 0, 2,-1 },
                               {-1, 2, 0, 1 },
                               { 2, 1,-1, 0 }}; 
    uint8_t last_state_;
    int32_t position_;
};

#endif