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
