#ifndef IESC_HPP
#define IESC_HPP

#include <Arduino.h>


namespace esc
{

/******************************************************************************
 *  Interface for any ESC (Electronic speed controller).
 *
 *  Contains methods for universal ESC. You can set and get speed or RPM.
 *  For better speed regulation is available methods with battery voltage
 *  parameter and compensates RPM according to the voltage.
 *****************************************************************************/
class IESC
{
 public:

     /**
      * Default destructor.
      */
     virtual ~IESC() = default;

     /**
      * Sets speed of ESC.
      *
      * @param speed    speed of ESC
      */
     virtual void SetSpeed(uint16_t speed) = 0;

     /**
      * Sets speed of ESC and compensates RPM according to the battery_voltage.
      *
      * @param speed            speed of ESC
      * @param battery_voltage  battery voltage in Volts
      */
     virtual void SetSpeed(uint16_t speed, float battery_voltage) = 0;

     /**
      * Returns speed of ESC.
      *
      * @param speed of ESC
      */
     virtual uint16_t GetSpeed() const = 0;

     /**
      * Returns the RPM of the motor according to the battery voltage.
      *
      * @param battery_voltage battery voltage in Volts
      *
      * @return the RPM of the motor
      */
     virtual uint16_t GetRPM(float battery_voltage) = 0;

};

}

#endif
