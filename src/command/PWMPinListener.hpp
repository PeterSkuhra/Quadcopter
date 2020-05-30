#ifndef PWM_PIN_LISTENER_HPP
#define PWM_PIN_LISTENER_HPP

#include <Arduino.h>


namespace command
{

/******************************************************************************
 *  The class represents the listener on a specific pin
 *  for processing the PWM signal and measuring its length.
 *
 *  This class uses an external interrupt to capture a signal change.
 *****************************************************************************/
class PWMPinListener
{
public:

    /**
     *  Constructor.
     *
     *  @param pin  input pin for listening PWM signal
     */
    PWMPinListener(uint8_t pin);

    /**
     *  Destructor.
     */
    ~PWMPinListener();

    /**
     *  Returns the pulse length of the PWM signal.
     *
     *  @return the pulse length of the PWM signal
     */
    uint16_t ReadChannel() const;

    /**
     *  Special method for handle external interrupt.
     *
     *  Inside the method, the pulse length of the PWM signal is measured.
     *  This method automatically calls itself as soon as an interrupt occurs.
     *
     *  Do not call this method! Must be public for ISR.
     */
    inline void HandleInterrupt();


private:

    /**
     *  pin for listening PWM signal
     */
    const uint8_t pin_;

    /**
     *  Pulse length of PWM signal.
     */
    uint16_t value_;

    /**
     *  Information for started update.
     *  True if PWM signal changed state from false to true.
     */
    bool update_started_;

    /**
     *  Start time of pulse PWM signal.
     */
    uint32_t time_start_;
};

}

#endif
