#ifndef ISENSOR_HPP
#define ISENSOR_HPP

#include <Arduino.h>

namespace sensing
{
namespace voltage
{

/******************************************************************************
 *  Interface for any sensor capable of returning a digital or analog value.
 *****************************************************************************/
class ISensor
{
 public:

     /**
      * Default destructor.
      */
     virtual ~ISensor() = default;

     /**
      * Returns digital value from sensor.
      *
      * @return digital value from sensor
      */
     virtual bool GetDigitalValue() = 0;

     /**
      * Returns analog value from sensor.
      *
      * @return analog value from sensor
      */
     virtual float GetAnalogValue() = 0;

     /**
      * Sets resolution of sensor.
      * It is important for right calculations of analog value.
      *
      * @param resolution   resolution of analog sensor
      */
     virtual void SetResolution(float resolution) = 0;

     /**
      * Returns the set resolution value.
      *
      * @return the set resolution value
      */
     virtual float GetResolution() const = 0;
};

}
}

#endif
