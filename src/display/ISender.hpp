#ifndef ISENDER_HPP
#define ISENDER_HPP

#include <Arduino.h>

namespace display
{

class ISender
{
public:
    virtual ~ISender() = default;

    virtual void WriteBatteryVoltage(uint8_t voltage) = 0;

    virtual void WriteRPM(uint16_t rpm) = 0;

    virtual void WriteTempetrature(int8_t temperature) = 0;
};

}

#endif
