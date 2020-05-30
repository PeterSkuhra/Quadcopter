#ifndef ESC30A_HPP
#define ESC30A_HPP

#include <Arduino.h>

#include "IESC.hpp"

namespace esc
{

#define MIN_PULSE           1000    // µs
#define MAX_PULSE           2000    // µs
#define RANGE_PULSE         (MAX_PULSE - MIN_PULSE)

/******************************************************************************
 *  This class represents the ESC (Electronic speed controller).
 *  The brushless motor can be controlled with instance of this class.
 *
 *  You can set speed in standard range in RC [1000, 2000].
 *  You can also check the current number of RPMs according
 *  to the battery voltage and the motor's KV parameter.
 *****************************************************************************/
class ESC30A : public IESC
{
 public:

     /**
      *  Constructor.
      *
      *  @param pin  Motor pin for control speed
      */
     ESC30A(uint8_t pin);

    /**
     *  Destructor.
     */
    ~ESC30A();

    /**
     *  Sets speed of motor.
     *
     *  @param speed    speed of motor in range [1000, 2000]
     */
    void SetSpeed(uint16_t speed) override;

    /**
     *  Sets speed of motor with compensation from battery voltage value.
     *
     *  @param speed            speed of motor in range [1000, 2000]
     *  @param battery_voltage  current battery voltage [V]
     */
    void SetSpeed(uint16_t speed, float battery_voltage) override;

    /**
     *  Returns current speed of motor in range [1000, 2000].
     *
     *  @return current speed of motor
     */
    uint16_t GetSpeed() const override;

    /**
     *  Returns actuall RPM of motor.
     *
     *  @param battery_voltage  current voltage of battery
     *
     *  @return actuall RPM of motor
     */
    uint16_t GetRPM(float battery_voltage) override;


private:

    /**
     *  Enum type of 16-bit timers.
     */
    enum Timers16Bit
    {
        TIMER1,
        TIMER3,
        TIMER4,
        TIMER5,
        NO_TIMER
    };

    /**
     *  Init registers of 16-bit timer for FastPWM mode.
     *
     *  @param timer    16-bit timer to init FastPWM mode.
     */
    static void InitRegisters(Timers16Bit timer);

    /**
     *  Returns the appropriate timer for the selected pin.
     *
     *  @param pin              pin number
     *
     *  @return Timers16Bit     enum type of timer
     */
    static Timers16Bit GetTimer(uint8_t pin);

    /**
     *  Generate PWM signal on selected pin with defined pulse.
     *
     *  @param pin      pin number
     *  @param value    pulse of PWM signal in microseconds
     */
    static void WritePWM12bit(uint8_t pin, uint16_t value);

 private:

    /**
     *  Pin number of motor control.
     */
    const uint8_t pin_;

    /**
     *  Current speed of motor in range [1000, 2000].
     */
    uint16_t speed_;

    /**
     *  Current RPM of motor.
     */
    uint16_t rpm_;
};

}

#endif
