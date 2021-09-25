#include "Encoder.h"

Encoder::Encoder(const uint8_t &_pinA, const uint8_t &_pinB) : pinA_(_pinA), pinB_(_pinB), position_(0), last_state_(0)
{
	pinMode(pinA_, INPUT);      // sets pin A as input
	pinMode(pinB_, INPUT);      // sets pin B as input
}

void Encoder::encoderTick()
{
    uint8_t current_state = (digitalRead(pinA_) << 1) | digitalRead(pinB_);
    position_ += QEM_[last_state_][current_state];
    // Calculate direction here if needed
    last_state_ = current_state;
}

int32_t Encoder::getPosition()
{
    return position_;
}