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

    //static IMotor* GetMotor(uint8_t pin);

    /**
    *  Constructor.
    *
    *  @param pin  Motor pin for control speed
    */
    BrushlessMotor(uint8_t pin, uint16_t = DEFAULT_FREQUENCY);

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

    void SetFrequency(uint32_t frequency);

    void SetPeriod(uint32_t period);


 private:

    void Init(uint8_t pin, uint16_t frequency);


 private:

     PWM* pwm_;
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
};

}

#endif
