#ifndef MPU6050_HPP
#define MPU6050_HPP

#include <Arduino.h>

#include "IIMU.hpp"
#include "../temperature/IThermometer.hpp"
#include "../../ICalibratable.hpp"
#include "Vector.hpp"

namespace sensing
{
namespace imu
{

#define DEAFULT_MPU6050_I2C_ADDRESS     0x68

enum AccelFullScaleRange
{
    FS_RANGE_2G = 0x00,
    FS_RANGE_4G = 0x08,
    FS_RANGE_8G = 0x10,
    FS_RANGE_16G = 0x18,
};

enum GyroFullScaleRange
{
    FS_RANGE_250 = 0x00,
    FS_RANGE_500 = 0x08,
    FS_RANGE_1000 = 0x10,
    FS_RANGE_2000 = 0x18,
};

enum DigitalLowPassFilterConfig
{
    DLPF_CFG_0 = 0x00,
    DLPF_CFG_1 = 0x01,
    DLPF_CFG_2 = 0x02,
    DLPF_CFG_3 = 0x03,
    DLPF_CFG_4 = 0x04,
    DLPF_CFG_5 = 0x05,
    // DLPF_CFG_6 = 0x06  // RESERVE
    // DLPF_CFG_7 = 0x07  // RESERVE
};

class MPU6050 : public IIMU,
                public temperature::IThermometer,
                public ICalibratable
{
 public:

    MPU6050(const uint8_t = DEAFULT_MPU6050_I2C_ADDRESS);

    void Begin() override;

    bool Calibrate() override;
    bool Calibrate(uint16_t samples);
    bool IsCalibrated() const override;

    void Update() override;

    void SetAccelFullScaleRange(AccelFullScaleRange = FS_RANGE_2G);
    void SetGyroFullScaleRange(GyroFullScaleRange = FS_RANGE_250);

    void SetDigitalLowPassFilter(DigitalLowPassFilterConfig = DLPF_CFG_3);

    uint8_t GetAccelFullScaleRange() const;
    uint8_t GetGyroFullScaleRange() const;

    float GetAccelLSBSensitivity() const;
    float GetGyroLSBSenssitivity() const;

    Vector<float> GetRollPitchYaw() const;
    float GetRoll() const override;
    float GetPitch() const override;
    float GetYaw() const override;

    Vector<float> GetAccOffset() const;
    Vector<float> GetGyroOffset() const;

    float GetTemperature() const override;

 private:

    inline void WriteToRegister(const uint8_t register_address,
                                const uint8_t data,
                                const bool = true);
    inline bool RequestFromRegister(const uint8_t register_address,
                                    const uint8_t bytes_count);

    bool CalibrateAccel(uint16_t = DEAFULT_SAMPLES);
    bool CalibrateGyro(uint16_t = DEAFULT_SAMPLES);

    void UpdateAccel();
    void UpdateGyro();
    void UpdateTemperature();

 private:

    const uint8_t i2c_address_;

    bool is_calibrated_;

    uint8_t accel_full_scale_range_;
    uint16_t gyro_full_scale_range_;
    float accel_lsb_sensitivity_;
    float gyro_lsb_sensitivity_;

    Vector<float> acc_raw_;
    Vector<float> gyro_raw_;

    Vector<float> acc_offset_;
    Vector<float> gyro_offset_;

    Vector<float> acc_angle_;
    Vector<float> gyro_angle_;

    //Vector<float> xyz_;
    Vector<float> roll_pitch_yaw_;

    float temperature_;

    struct timer
    {
        float current_;
        float previous_;
        float elapsed_;
    } time_;
};

}
}

#endif
