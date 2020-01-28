#ifndef BRUSHLESS_MOTOR_HPP
#define BRUSHLESS_MOTOR_HPP

#include <Arduino.h>

#include "IMotor.hpp"
#include "PWM.hpp"

namespace motor
{

/******************************************************************************
 *  Class represents motor which is controlled
 *  with ESC (Electronic speed controller).
 *
 *  You can set speed in standard range in RC [1000, 2000].
 *  You can also check the current number of RPMs according
 *  to the battery voltage and the motor's KV parameter.
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
