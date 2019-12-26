#ifndef BRUSHLESS_MOTOR_HPP
#define BRUSHLESS_MOTOR_HPP

#include <Arduino.h>

#include "IMotor.hpp"
#include "PWM.hpp"

namespace motor
{

#define DEFAULT_FREQUENCY   50  // Hz

/******************************************************************************
 *  Class represents motor which is controlled
 *  with ESC (Electronic speed controller).
 *
 *  The multiton pattern is aplied.
 *
 *  Class uses 16-bit Timers (Arduino Mega):
 *  Timer 1 controls pins 11, 12, 13
 *  Timer 3 controls pins 2, 3, 5
 *  Timer 4 controls pins 6, 7, 8
 *  Timer 5 controls pins 38, 39, 40
 *
 *  If you will use some pin from any Timer, you shouldn't use other pins from
 *  this group for other functions.
 *  Any pin configuration also affects others pins from group.
 *
 *****************************************************************************/
class BrushlessMotor : public IMotor
{
 public:
     /**
      *  Constructor.
      *
      *  @param pin  Motor pin for control speed
      */
     BrushlessMotor(uint8_t pin);

    /**
     *  Destructor.
     */
    ~BrushlessMotor();

    /**
     *  Sets speed of motor.
     *
     *  @param speed    speed of motor in range [1000, 2000]
     */
    void SetSpeed(uint16_t speed) override;

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
    uint16_t GetRPM(uint8_t battery_voltage) override;


 private:

    /**
     *  Pin number of motor control.
     */
    uint8_t pin_;

    uint16_t frequency_;

    /**
     *  Current speed of motor in range [1000, 2000]
     */
    uint16_t speed_;

    /**
     *  Current RPM of motor.
     */
    uint16_t rpm_;

    /**
     *  PWM instance for generate PWM signal on defined pin.
     */
    PWM* pwm_;
};

}

#endif
