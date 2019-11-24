#ifndef PPM_PIN_LISTENER_HPP
#define PPM_PIN_LISTENER_HPP

#include <Arduino.h>

namespace command
{

class PPMPinListener
{
public:
    
    PPMPinListener(uint8_t pin);

    ~PPMPinListener();

    uint16_t ReadChannel() const;

    void HandleInterrupt();


private:

    inline void ProcessPPM();

private:

    uint8_t pin_;

    uint16_t value_;

    bool update_started_;

    struct timer {
        uint16_t start;
        uint16_t current;
    } time_;

};

}

#endif
