#ifndef IMU_READER_HPP
#define IMU_READER_HPP

#include <Arduino.h>

#include <I2Cdev.h>
//#include <MPU6050_6Axis_MotionApps20.h>       // stare_mpu
#include <MPU6050_6Axis_MotionApps_V6_12.h>

// #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include <SBWire.h>
// #endif

#include "IIMU.hpp"
#include "../temperature/IThermometer.hpp"

namespace sensing
{

namespace imu
{

class IMUReader : public IIMU, public temperature::IThermometer
{
public:
    IMUReader(const uint8_t interrupt_pin);

    IMUReader(const uint8_t interrupt_pin,
              bool x_invert,
              bool y_invert,
              bool z_invert);

    ~IMUReader();

    void Begin() override;

    bool Calibrate() override;

    bool IsCalibrated() const override;

    void Update() override;

    inline float GetXAcceleration() const override;
    inline float GetYAcceleration() const override;
    inline float GetZAcceleration() const override;

    inline float GetXAngularRate() const override;
    inline float GetYAngularRate() const override;
    inline float GetZAngularRate() const override;

    inline float GetRollAngularRate() const override;
    inline float GetPitchAngularRate() const override;
    inline float GetYawAngularRate() const override;

    float GetXAngle() const override;
    float GetYAngle() const override;
    float GetZAngle() const override;

    float GetRollAngle() const override;
    float GetPitchAngle() const override;
    float GetYawAngle() const override;

    inline float GetTemperature() const override;

private:
    const uint8_t kInterruptPin_;

    MPU6050* mpu_;


    bool dmp_ready_;

    uint8_t mpu_interrupt_status_;

    uint8_t device_status_;

    uint16_t packet_size_;

    uint16_t fifo_count_;

    uint8_t fifo_buffer_[64];

    bool is_calibrated_;


    Quaternion quaternion_;
    VectorFloat gravity_;

    float ypr_[3];
    bool axes_invert_[3];
};

}
}

#endif
