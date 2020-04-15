#ifndef ISENSOR_HPP
#define ISENSOR_HPP

#include <Arduino.h>

namespace sensing
{
namespace voltage
{

class ISensor
{
public:

    virtual ~ISensor() = default;

    virtual bool GetDigitalValue() = 0;

    virtual float GetAnalogValue() = 0;

    virtual void SetResolution(float resolution) = 0;

    virtual float GetResolution() const = 0;

};

}
}

#endif
