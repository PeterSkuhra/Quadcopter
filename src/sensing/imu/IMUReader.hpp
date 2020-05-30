#ifndef IMU_READER_HPP
#define IMU_READER_HPP

#include <Arduino.h>

#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps_V6_12.h>

#include <SBWire.h>

#include "IIMU.hpp"
#include "../temperature/IThermometer.hpp"

namespace sensing
{

namespace imu
{

/******************************************************************************
 *  Wrapper class for reading data from specific IMU: MPU-6050
 *  Implements interfaces IIMU and IThermometer.
 *
 *  You can read acceleration and angular rates from all axes (X, Y, Z).
 *  Also provides communication with DMP (Digital motion processor) and reads
 *  FIFO buffer with important data for angle calculations.
 *
 *  Uses external interrupt for capture "data ready" signal from DMP in MPU.
 *****************************************************************************/
class IMUReader : public IIMU, public temperature::IThermometer
{
 public:

     /**
      * Constructor.
      *
      * @param interrupt_pin    pin for external interrupt from IMU
      */
     IMUReader(const uint8_t interrupt_pin);

     /**
      * Constructor.
      *
      * @param interrupt_pin    pin for external interrupt from IMU
      * @param x_invert         true, if x invert
      * @param y_invert         true, if y invert
      * @param z_invert         true, if z invert
      */
     IMUReader(const uint8_t interrupt_pin,
              bool x_invert,
              bool y_invert,
              bool z_invert);

     /**
      * Destructor.
      */
     ~IMUReader();

     /**
      * Initializes communication with IMU (MPU-6050)
      */
     void Begin() override;

     /**
      * Calibrates MPU-6050
      *
      * @return true if the calibration was successful
      */
     bool Calibrate() override;

     /**
      * Returns true if IMU is calibrated.
      *
      * @return true if IMU is calibrated
      */
     bool IsCalibrated() const override;

     /**
      * Updates all data from MPU-6050.
      */
     void Update() override;

     /**
      * Returns the X-axis acceleration.
      *
      * @return the X-axis acceleration
      */
     inline float GetXAcceleration() const override;

     /**
      * Returns the Y-axis acceleration.
      *
      * @return the Y-axis acceleration
      */
     inline float GetYAcceleration() const override;

     /**
      * Returns the Z-axis acceleration.
      *
      * @return the Z-axis acceleration
      */
     inline float GetZAcceleration() const override;

     /**
      * Returns the angular velocity of the X axis.
      *
      * @return the angular velocity of the X axis in °/s
      */
     inline float GetXAngularRate() const override;

     /**
      * Returns the angular velocity of the Y axis.
      *
      * @return the angular velocity of the Y axis in °/s
      */
     inline float GetYAngularRate() const override;

     /**
      * Returns the angular velocity of the Z axis.
      *
      * @return the angular velocity of the Z axis in °/s
      */
     inline float GetZAngularRate() const override;

     /**
      * Returns the angular velocity of the roll axis (X axis).
      *
      * @return the angular velocity of the roll axis (X axis) in °/s
      */
     inline float GetRollAngularRate() const override;

     /**
      * Returns the angular velocity of the pitch axis (Y axis).
      *
      * @return the angular velocity of the pitch axis (Y axis) in °/s
      */
     inline float GetPitchAngularRate() const override;

     /**
      * Returns the angular velocity of the yaw axis (Z axis).
      *
      * @return the angular velocity of the yaw axis (Z axis) in °/s
      */
     inline float GetYawAngularRate() const override;

     /**
      * Returns the angle of the X axis.
      *
      * @return the angle of the X axis in degrees
      */
     float GetXAngle() const override;

     /**
      * Returns the angle of the Y axis.
      *
      * @return the angle of the Y axis in degrees
      */
     float GetYAngle() const override;

     /**
      * Returns the angle of the Z axis.
      *
      * @return the angle of the Z axis in degrees
      */
     float GetZAngle() const override;

     /**
      * Returns the angle of the roll axis (X axis).
      *
      * @return the angle of the roll axis (X axis) in degrees
      */
     float GetRollAngle() const override;

     /**
      * Returns the angle of the pitch axis (Y axis).
      *
      * @return the angle of the pitch axis (Y axis) in degrees
      */
     float GetPitchAngle() const override;

     /**
      * Returns the angle of the yaw axis (Z axis).
      *
      * @return the angle of the yaw axis (Z axis) in degrees
      */
     float GetYawAngle() const override;

     /**
      * Returns the temperature in degrees Celsius.
      *
      * @return the temperature in degrees Celsius
      */
     inline float GetTemperature() const override;


 private:

     /**
      * Pin for external interrupt for capturing "data ready" siganl from DMP.
      */
     const uint8_t kInterruptPin_;

     /**
      * Instance MPU-6050
      */
     MPU6050* mpu_;

     /**
      * DMP (digital motion processor) is ready
      */
     bool dmp_ready_;

     /**
      * Status of MPU interrupt.
      */
     uint8_t mpu_interrupt_status_;

     /**
      * MPU Device status.
      */
     uint8_t device_status_;

     /**
      * Size of packet for FIFO.
      */
     uint16_t packet_size_;

     /**
      * Count of data in FIFO.
      */
     uint16_t fifo_count_;

     /**
      * FIFO buffer for data.
      */
     uint8_t fifo_buffer_[64];

     /**
      * MPU is calibrated.
      */
     bool is_calibrated_;

     /**
      * Instance of quaternion.
      */
     Quaternion quaternion_;

     /**
      * Instance of gravity vector.
      */
     VectorFloat gravity_;

     /**
      * Array of all angles (Yaw, Pitch, Roll)
      */
     float ypr_[3];

     /**
      * Inverts axes.
      */
     bool axes_invert_[3];
};

}
}

#endif
