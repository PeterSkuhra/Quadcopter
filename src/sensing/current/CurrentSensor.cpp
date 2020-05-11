#include "CurrentSensor.hpp"

#define ANALOG_RES          1024.0      // 2^10 = 1024
#define VOLTAGE_REFERENCE   5000.0      // mV
#define VOLTAGE_OFFSET      2500.0      // mV


sensing::current::CurrentSensor::CurrentSensor(const uint8_t pin,
                                               float resolution) :
    pin_(pin),
    resolution_(resolution)
{
    pinMode(pin_, INPUT);
}

bool sensing::current::CurrentSensor::GetDigitalValue()
{
    return digitalRead(pin_);
}

float sensing::current::CurrentSensor::GetAnalogValue()
{
    uint16_t raw_value = analogRead(pin_);
    float voltage_value = (raw_value / ANALOG_RES) * VOLTAGE_REFERENCE;
    current_value_ = (voltage_value - VOLTAGE_OFFSET) / resolution_;

    return current_value_;
}

void sensing::current::CurrentSensor::SetResolution(float resolution)
{
    resolution_ = resolution;
}

float sensing::current::CurrentSensor::GetResolution() const
{
    return resolution_;
}
