#ifndef IRECEIVER_HPP
#define IRECEIVER_HPP

#include <Arduino.h>

#include "ICalibratable.hpp"


namespace command
{

class IReceiver : public ICalibratable
{
public:
    virtual ~IReceiver() = default;

    virtual uint16_t ReadChannel(uint8_t channel_number) const = 0;
};

}

#endif
