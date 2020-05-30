#ifndef ITHERMOMETER_HPP
#define ITHERMOMETER_HPP

namespace sensing
{
namespace temperature
{

/******************************************************************************
 *  Interface for general thermometer. You can read temperature from any device.
 *****************************************************************************/
class IThermometer
{
 public:

     /**
      * Default desctructor.
      */
     virtual ~IThermometer() = default;

     /**
      * Updates data from thermometer.
      */
     virtual void Update() = 0;

     /**
      * Returns temperature value.
      *
      * @return temperature value
      */
     virtual float GetTemperature() const = 0;
};

}
}

#endif
