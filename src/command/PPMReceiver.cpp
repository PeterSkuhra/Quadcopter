#include "PPMReceiver.hpp"

using namespace command;

#define ARRAY_SIZE(x)   (sizeof((x)) / sizeof((x)[0]))


PPMReceiver::PPMReceiver(uint8_t pins[]) :
    channel_count_(ARRAY_SIZE(pins))
{
    noInterrupts();
    memcpy(channel_pins_, pins, sizeof(pins));
    interrupts();

    ppm_pin_listeners_ = new PPMPinListener*[channel_count_];

    for (int i = 0; i < channel_count_; ++i) {
        ppm_pin_listeners_[i] = new PPMPinListener(channel_pins_[i]);
    }
}

PPMReceiver::~PPMReceiver()
{
    for (int i = 0; i < channel_count_; ++i) {
        delete [] ppm_pin_listeners_[i];
    }
    delete [] ppm_pin_listeners_;
}

uint16_t PPMReceiver::ReadChannel(uint8_t channel_number) const
{
    return ppm_pin_listeners_[channel_number - 1]->ReadChannel();
}
