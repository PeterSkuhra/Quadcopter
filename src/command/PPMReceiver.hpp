#ifndef PPM_RECEIVER_HPP
#define PPM_RECEIVER_HPP

#include <Arduino.h>

#include "IReceiver.hpp"
#include "PPMPinListener.hpp"


namespace command
{

class PPMReceiver : public IReceiver
{
public:
    PPMReceiver(uint8_t pins[], uint8_t size);

    ~PPMReceiver();

    uint16_t ReadChannel(uint8_t channel_number) const override;


private:
    const uint8_t* channel_pins_;
    const uint8_t channel_count_;

    PPMPinListener** ppm_pin_listeners_;
};

}

#endif
