#ifndef VOLTAGE_SENSOR_HPP
#define VOLTAGE_SENSOR_HPP

#include <Arduino.h>

#include "ISensor.hpp"

namespace sensing
{

#define DEFAULT_RESOLUTION      1

class VoltageSensor : public ISensor
{
public:

    VoltageSensor(uint8_t pin, float = DEFAULT_RESOLUTION);

    bool GetState() override;

    float GetAnalogValue() override;


private:

    uint8_t pin_;

    bool state_;

    float analog_value_;

    float resolution_;
};

}

#endif
