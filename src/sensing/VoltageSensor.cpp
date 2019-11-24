#include "VoltageSensor.hpp"

using namespace sensing;

VoltageSensor::VoltageSensor(uint8_t pin, float resolution) :
    pin_(pin),
    state_(false),
    analog_value_(0.0),
    resolution_(resolution)
{
    pinMode(pin_, INPUT);
}

bool VoltageSensor::GetState()
{
    state_ = digitalRead(pin_);

    return state_;
}

float VoltageSensor::GetAnalogValue()
{
    analog_value_ = analogRead(pin_);
    analog_value_ *= resolution_;

    return analog_value_;
}
