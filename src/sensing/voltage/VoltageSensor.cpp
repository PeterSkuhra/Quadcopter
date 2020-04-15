#include "VoltageSensor.hpp"


sensing::voltage::VoltageSensor::VoltageSensor(uint8_t pin, float resolution) :
    pin_(pin),
    resolution_(resolution)
{
    pinMode(pin_, INPUT);
}

bool sensing::voltage::VoltageSensor::GetDigitalValue()
{
    digital_value_ = digitalRead(pin_);

    return digital_value_;
}

float sensing::voltage::VoltageSensor::GetAnalogValue()
{
    analog_value_ = analogRead(pin_);
    analog_value_ *= resolution_;

    return analog_value_;
}

void sensing::voltage::VoltageSensor::SetResolution(float resolution)
{
    resolution_ = resolution;
}

float sensing::voltage::VoltageSensor::GetResolution() const
{
    return resolution_;
}
