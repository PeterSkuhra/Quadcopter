#include "PWMReceiver.hpp"

using namespace command;


PWMReceiver::PWMReceiver(const std::vector<uint8_t> &pins) :
    channel_count_(pins.size())
{
    pwm_pin_listeners_ = new PWMPinListener*[channel_count_];

    for (int i = 0; i < channel_count_; ++i) {
        pwm_pin_listeners_[i] = new PWMPinListener(pins[i]);
    }
}

PWMReceiver::~PWMReceiver()
{
    for (int i = 0; i < channel_count_; ++i) {
        delete [] pwm_pin_listeners_[i];
    }
    delete [] pwm_pin_listeners_;
}

uint16_t PWMReceiver::ReadChannel(uint8_t channel_number) const
{
    if ((channel_number <= channel_count_) && !(channel_number <= 0)) {
        return pwm_pin_listeners_[channel_number - 1]->ReadChannel();
    }
}
