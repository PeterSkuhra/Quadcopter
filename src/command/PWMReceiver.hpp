#ifndef PWM_RECEIVER_HPP
#define PWM_RECEIVER_HPP

#include <Arduino.h>
#include <ArduinoSTL.h>

#include "IReceiver.hpp"
#include "PWMPinListener.hpp"


namespace command
{

class PWMReceiver : public IReceiver
{
public:
    PWMReceiver(const std::vector<uint8_t> &pins);

    ~PWMReceiver();

    uint16_t ReadChannel(uint8_t channel_number) const override;


private:
    const uint8_t channel_count_;

    PWMPinListener** pwm_pin_listeners_;
};

}

#endif
