#ifndef IIMU_HPP
#define IIMU_HPP

#include <Arduino.h>

#include "ICalibratable.hpp"

namespace sensing
{
namespace imu
{

#define DEAFULT_SAMPLES     1000

/******************************************************************************
 *  IMU interface. Provides methods for communicating with the general IMU.
 *****************************************************************************/
class IIMU : public ICalibratable
{
 public:

     /**
      *  Default destructor.
      */
     virtual ~IIMU() = default;

     /**
      * Initializes and begins communication with IMU.
      */
     virtual void Begin() = 0;

     /**
      * Update loop for reading data.
      */
     virtual void Update() = 0;

     /**
      * Returns the X-axis acceleration.
      *
      * @return the X-axis acceleration
      */
     virtual float GetXAcceleration() const = 0;

     /**
      * Returns the Y-axis acceleration.
      *
      * @return the Y-axis acceleration
      */
     virtual float GetYAcceleration() const = 0;

     /**
      * Returns the Z-axis acceleration.
      *
      * @return the Z-axis acceleration
      */
     virtual float GetZAcceleration() const = 0;

     /**
      * Returns the angular velocity of the X axis.
      *
      * @return the angular velocity of the X axis
      */
     virtual float GetXAngularRate() const = 0;

     /**
      * Returns the angular velocity of the Y axis.
      *
      * @return the angular velocity of the Y axis
      */
     virtual float GetYAngularRate() const = 0;

     /**
      * Returns the angular velocity of the Z axis.
      *
      * @return the angular velocity of the Z axis
      */
     virtual float GetZAngularRate() const = 0;

     /**
      * Returns the angular velocity of the roll axis (X axis).
      *
      * @return the angular velocity of the roll axis (X axis)
      */
     virtual float GetRollAngularRate() const = 0;

     /**
      * Returns the angular velocity of the pitch axis (Y axis).
      *
      * @return the angular velocity of the pitch axis (Y axis)
      */
     virtual float GetPitchAngularRate() const = 0;

     /**
      * Returns the angular velocity of the yaw axis (Z axis).
      *
      * @return the angular velocity of the yaw axis (Z axis)
      */
     virtual float GetYawAngularRate() const = 0;

     /**
      * Returns the angle of the X axis.
      *
      * @return the angle of the X axis
      */
     virtual float GetXAngle() const = 0;

     /**
      * Returns the angle of the Y axis.
      *
      * @return the angle of the Y axis
      */
     virtual float GetYAngle() const = 0;

     /**
      * Returns the angle of the Z axis.
      *
      * @return the angle of the Z axis
      */
     virtual float GetZAngle() const = 0;

     /**
      * Returns the angle of the roll axis (X axis).
      *
      * @return the angle of the roll axis (X axis)
      */
     virtual float GetRollAngle() const = 0;

     /**
      * Returns the angle of the pitch axis (Y axis).
      *
      * @return the angle of the pitch axis (Y axis)
      */
     virtual float GetPitchAngle() const = 0;

     /**
      * Returns the angle of the yaw axis (Z axis).
      *
      * @return the angle of the yaw axis (Z axis)
      */
     virtual float GetYawAngle() const = 0;
};

}
}

#endif
