#ifndef BMP180_HPP
#define BMP180_HPP

#include <Arduino.h>

#include "IBarometer.hpp"
#include "../temperature/IThermometer.hpp"

namespace sensing
{
namespace pressure
{

#define DEFAULT_BMP180_I2C_ADDRESS     0x77    // ????

class BMP180 : public IBarometer, public temperature::IThermometer
{
public:

    BMP180(uint8_t = DEFAULT_BMP180_I2C_ADDRESS);

    void Begin() override;

    void Update() override;

    float GetPressure() const override;

    float GetAltitude() const override;

    float GetTemperature() const override;


private:

    uint8_t i2c_address_;

    float pressure_;
    float altitude_;
    float temperature_;
};

}
}

#endif
