#ifndef IESC_HPP
#define IESC_HPP

#include <Arduino.h>


namespace esc
{

class IESC
{
public:

    virtual ~IESC() = default;

    virtual void SetSpeed(uint16_t speed) = 0;

    virtual void SetSpeed(uint16_t speed, float battery_voltage) = 0;

    virtual uint16_t GetSpeed() const = 0;

    virtual uint16_t GetRPM(float battery_voltage) = 0;

};

}

#endif
