#ifndef VOLTAGESENSOR_HPP
#define VOLTAGESENSOR_HPP

#include <Arduino.h>

#include "ISensor.hpp"


namespace sensing
{
namespace voltage
{

/**
 *  Default resolution:
 *
 *  2^10 = 1024
 *  5V / 1024 = 0.004882813
 *
 *  Arduino float reference:
 *  https://www.arduino.cc/en/pmwiki.php?n=Reference/Float
 *
 *  Floats in Arduino have only 6-7 decimal digits of precision.
 *  Double type is same as float.
 */
#define DEFAULT_RESOLUTION      (0.004883)


/**
 *  Class represents voltage sensor for Arduino.
 *  Implemets methods from ISensor interface.
 *
 *  You can meassure digital value (true/false) or analog value in Volts.
 *  During meassuring analog value is very important right sets
 *  resolution of voltage sensor.
 *  Arduino usualy has 10 bit resolution of analog pins.
 *  So resulting resolution for any sensor is:
 *      max_value_of_sensor / 2^10
 */
class VoltageSensor : public ISensor
{
 public:
     
     /**
      * Constructor.
      *
      * @param pin          Voltage sensor pin
      * @param resolution   Resolution of voltage sensor
      * For example: If maximum value of voltage sensor is 30V, then
      * resolution is:  30V / (2^10) = 0.029296875
      */
     VoltageSensor(uint8_t pin, float resolution = DEFAULT_RESOLUTION);

     /**
      * Returns digital value of sensor.
      *
      * @return bool true if voltage on pin is greater than 3.0V
      *              false if voltage on pin is less than 1.5V
      */
     bool GetDigitalValue() override;

     /**
      * Returns analog value of sensor on Volts.
      *
      * @return float   analog value on pin in Volts.
      */
     float GetAnalogValue() override;

     /**
      * Sets sensor resolution.
      *
      * @param resolution   sensor resolution
      */
     void SetResolution(float resolution) override;

     /**
      * Returns current resolution of sensor.
      *
      * @return float   current resolution of sensor
      */
     float GetResolution() const override;


 private:

     /**
      * Signal pin of voltage sensor.
      */
     const uint8_t pin_;

     /**
      * Resolution of voltage sensor.
      */
     float resolution_;

     /**
      * Digital value of sensor (true/false)
      */
     bool digital_value_;

     /**
      * Analog value of sensor in Volts.
      */
     float analog_value_;
};

}
}

#endif
