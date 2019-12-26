#include "PWM.hpp"

PWM* PWM::GetInstance(uint8_t pin)
{
    const auto it = instances_.find(pin);

    if (it != instances_.end())
        return (PWM*)(it->second);

    PWM* instance = new PWM(pin);
    instances_[pin] = instance;
    return instance;
}
