#include "MPU6050.hpp"

#include <Wire.h>

using namespace sensing;
using namespace imu;

#define PWR_MGMT_1      0x6B

#define CONFIG          0x1A
#define ACCEL_CONFIG    0x1C
#define GYRO_CONFIG     0x1B

#define ACCEL_XOUT_H    0x3B    // First register of accel data
#define GYRO_XOUT_H     0x43    // First register of gyro data
#define TEMP_OUT_H      0x41    // First register of temperature data

#define ACCEL_REGISTERS_COUNT   6
#define GYRO_REGISTERS_COUNT    6
#define TEMP_REGISTERS_COUNT    2

#define ACCEL_LSB_SENSITIVITY_2G    16384.0
#define ACCEL_LSB_SENSITIVITY_4G    8192.0
#define ACCEL_LSB_SENSITIVITY_8G    4096.0
#define ACCEL_LSB_SENSITIVITY_16G   2048.0

#define GYRO_LSB_SENSITIVITY_250    131.0
#define GYRO_LSB_SENSITIVITY_500    65.5
#define GYRO_LSB_SENSITIVITY_1000   32.8
#define GYRO_LSB_SENSITIVITY_2000   16.4

// std::map<uint8_t, MPU6050*> MPU6050::instances_;

// IIMU* MPU6050::GetInstance(const uint8_t i2c_address)
// {
//     if (instances_.find(i2c_address) == instances_.end()) {
//         instances_.insert({i2c_address, new MPU6050(i2c_address)});
//     }
//
//     return instances_[i2c_address];
// }

MPU6050::MPU6050(const uint8_t i2c_address) :
    i2c_address_(i2c_address),
    is_calibrated_(false),
    time_{0.0, 0.0, 0.0}
{
    Wire.begin();
}

void MPU6050::Begin()
{
    WriteToRegister(PWR_MGMT_1, 0x00);

    SetAccelFullScaleRange(FS_RANGE_2G);
    SetGyroFullScaleRange(FS_RANGE_250);
    SetDigitalLowPassFilter(DLPF_CFG_3);

    // is_calibrated_ = Calibrate();
}

bool MPU6050::Calibrate()
{
    return Calibrate(DEAFULT_SAMPLES);
}

bool MPU6050::Calibrate(uint16_t samples)
{
    return (CalibrateAccel(samples) && CalibrateGyro(samples));
}

bool MPU6050::IsCalibrated() const
{
    return is_calibrated_;
}

void MPU6050::Update()
{
    UpdateAccel();
    UpdateGyro();
    UpdateTemperature();
}

void MPU6050::SetAccelFullScaleRange(AccelFullScaleRange range)
{
    WriteToRegister(ACCEL_CONFIG, range);

    switch (range) {
        case FS_RANGE_2G:
            accel_full_scale_range_ = 2;
            accel_lsb_sensitivity_ = ACCEL_LSB_SENSITIVITY_2G;
            break;

        case FS_RANGE_4G:
            accel_full_scale_range_ = 4;
            accel_lsb_sensitivity_ = ACCEL_LSB_SENSITIVITY_4G;
            break;

        case FS_RANGE_8G:
            accel_full_scale_range_ = 8;
            accel_lsb_sensitivity_ = ACCEL_LSB_SENSITIVITY_8G;
            break;

        case FS_RANGE_16G:
            accel_full_scale_range_ = 16;
            accel_lsb_sensitivity_ = ACCEL_LSB_SENSITIVITY_16G;
            break;
    }
}

void MPU6050::SetGyroFullScaleRange(GyroFullScaleRange range)
{
    WriteToRegister(GYRO_CONFIG, range);

    switch (range) {
        case FS_RANGE_2G:
            gyro_full_scale_range_ = 250;
            gyro_lsb_sensitivity_ = GYRO_LSB_SENSITIVITY_250;
            break;

        case FS_RANGE_4G:
            gyro_full_scale_range_ = 500;
            gyro_lsb_sensitivity_ = GYRO_LSB_SENSITIVITY_500;
            break;

        case FS_RANGE_8G:
            gyro_full_scale_range_ = 1000;
            gyro_lsb_sensitivity_ = GYRO_LSB_SENSITIVITY_1000;
            break;

        case FS_RANGE_16G:
            gyro_full_scale_range_ = 2000;
            gyro_lsb_sensitivity_ = GYRO_LSB_SENSITIVITY_2000;
            break;
    }
}

void MPU6050::SetDigitalLowPassFilter(DigitalLowPassFilterConfig dlpf_config)
{
    WriteToRegister(CONFIG, dlpf_config);
}

uint8_t MPU6050::GetAccelFullScaleRange() const
{
    return accel_full_scale_range_;
}

uint8_t MPU6050::GetGyroFullScaleRange() const
{
    return gyro_full_scale_range_;
}

float MPU6050::GetAccelLSBSensitivity() const
{
    return accel_lsb_sensitivity_;
}

float MPU6050::GetGyroLSBSenssitivity() const
{
    return gyro_lsb_sensitivity_;
}

Vector<float> MPU6050::GetRollPitchYaw() const
{
    return roll_pitch_yaw_;
}

float MPU6050::GetRoll() const
{
    return roll_pitch_yaw_.x;
}

float MPU6050::GetPitch() const
{
    return roll_pitch_yaw_.y;
}

float MPU6050::GetYaw() const
{
    return roll_pitch_yaw_.z;
}

Vector<float> MPU6050::GetAccOffset() const
{
    return acc_offset_;
}

Vector<float> MPU6050::GetGyroOffset() const
{
    return gyro_offset_;
}

float MPU6050::GetTemperature() const
{
    return temperature_;
}

inline void MPU6050::WriteToRegister(const uint8_t register_address,
                                     const uint8_t data,
                                     const bool end)
{
    Wire.beginTransmission(i2c_address_);
    Wire.write(register_address);
    Wire.write(data);
    Wire.endTransmission(end);
}

inline void MPU6050::RequestFromRegister(const uint8_t register_address,
                                         const uint8_t bytes_count)
{
    Wire.beginTransmission(i2c_address_);
    Wire.write(register_address);       // First byte for request
    Wire.endTransmission(false);
    Wire.requestFrom(i2c_address_, bytes_count);    // Request bytes count
}

bool MPU6050::CalibrateAccel(uint16_t samples)
{
    for (uint16_t i = 0; i < samples; ++i) {
        RequestFromRegister(ACCEL_XOUT_H, ACCEL_REGISTERS_COUNT);

        acc_raw_.x = (Wire.read() << 8 | Wire.read()) / accel_lsb_sensitivity_;
        acc_raw_.y = (Wire.read() << 8 | Wire.read()) / accel_lsb_sensitivity_;
        acc_raw_.z = (Wire.read() << 8 | Wire.read()) / accel_lsb_sensitivity_;

        acc_offset_.x += (atan(
            (acc_raw_.y) / sqrt(pow((acc_raw_.x), 2) + pow((acc_raw_.z), 2)))
            * 180 / PI);
        acc_offset_.y += (atan(-1 * (acc_raw_.x) / sqrt(
            pow((acc_raw_.y), 2) + pow((acc_raw_.z), 2))) * 180 / PI);
    }

    acc_offset_.x /= samples;
    acc_offset_.y /= samples;
    //acc_offset_.z /= samples;

    return true;
}

bool MPU6050::CalibrateGyro(uint16_t samples)
{
    for (uint16_t i = 0; i < samples; ++i) {
        RequestFromRegister(GYRO_XOUT_H, GYRO_REGISTERS_COUNT);

        gyro_raw_.x = (Wire.read() << 8 | Wire.read());
        gyro_raw_.y = (Wire.read() << 8 | Wire.read());
        gyro_raw_.z = (Wire.read() << 8 | Wire.read());

        gyro_offset_.x += (gyro_raw_.x / gyro_lsb_sensitivity_);
        gyro_offset_.y += (gyro_raw_.y / gyro_lsb_sensitivity_);
        gyro_offset_.z += (gyro_raw_.z / gyro_lsb_sensitivity_);
    }

    gyro_offset_.x /= samples;
    gyro_offset_.y /= samples;
    gyro_offset_.z /= samples;

    return true;
}

void MPU6050::UpdateAccel()
{
    RequestFromRegister(ACCEL_XOUT_H, ACCEL_REGISTERS_COUNT);

    acc_raw_.x = (Wire.read() << 8 | Wire.read()) / accel_lsb_sensitivity_;
    acc_raw_.y = (Wire.read() << 8 | Wire.read()) / accel_lsb_sensitivity_;
    acc_raw_.z = (Wire.read() << 8 | Wire.read()) / accel_lsb_sensitivity_;

    acc_angle_.x =
        (atan((acc_raw_.y) / sqrt(pow((acc_raw_.x), 2) + pow((acc_raw_.z), 2)))
            *
                180 / PI) +
            (acc_offset_.x * (-1));

    acc_angle_.y =
        (atan(-1 * (acc_raw_.x)
                  / sqrt(pow((acc_raw_.y), 2) + pow((acc_raw_.z), 2))) *
            180 / PI) +
            (acc_offset_.y * (-1));
}

void MPU6050::UpdateGyro()
{
    time_.previous_ = time_.current_;
    time_.current_ = micros();
    time_.elapsed_ = (time_.current_ - time_.previous_) / 1000000;

    RequestFromRegister(GYRO_XOUT_H, GYRO_REGISTERS_COUNT);

    gyro_raw_.x = (Wire.read() << 8 | Wire.read()) / gyro_lsb_sensitivity_;
    gyro_raw_.y = (Wire.read() << 8 | Wire.read()) / gyro_lsb_sensitivity_;
    gyro_raw_.z = (Wire.read() << 8 | Wire.read()) / gyro_lsb_sensitivity_;

    gyro_raw_.x += (gyro_offset_.x * (-1));
    gyro_raw_.y += (gyro_offset_.y * (-1));
    gyro_raw_.z += (gyro_offset_.z * (-1));

    //  deg/s * s = deg
    gyro_angle_.x += (gyro_raw_.x * time_.elapsed_);
    gyro_angle_.y += (gyro_raw_.y * time_.elapsed_);
    roll_pitch_yaw_.z += (gyro_raw_.z * time_.elapsed_);    // yaw

    // Complementary filter
    roll_pitch_yaw_.x = (0.96 * gyro_angle_.x) + (0.04 * acc_angle_.x);
    roll_pitch_yaw_.y = (0.96 * gyro_angle_.y) + (0.04 * acc_angle_.y);
}

void MPU6050::UpdateTemperature()
{
    RequestFromRegister(TEMP_OUT_H, TEMP_REGISTERS_COUNT);

    // Temperature in degrees C = TEMP_OUT / 340 + 36.53    (see in datasheet)
    temperature_ = (Wire.read() << 8 | Wire.read()) / 340 + 36.53;
}
