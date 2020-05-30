#ifndef IRECEIVER_HPP
#define IRECEIVER_HPP

#include <Arduino.h>

#include "ICalibratable.hpp"


namespace command
{

/******************************************************************************
 *  An interface for any receiver that can process channels
 *  and return their values.
 *
 *  Implements ICalibratable interface.
 *****************************************************************************/
class IReceiver : public ICalibratable
{
 public:
     
     /**
      *  Default virtual destructor.
      */
     virtual ~IReceiver() = default;

     /**
      *  Returns value od specific channel.
      *
      *  @param channel_number   number of channel to read
      *
      *  @return value of specific channel
      */
     virtual uint16_t ReadChannel(uint8_t channel_number) const = 0;
};

}

#endif
