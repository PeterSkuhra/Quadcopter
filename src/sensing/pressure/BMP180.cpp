#include "BMP180.hpp"

#include <Wire.h>

using namespace sensing;
using namespace pressure;
using namespace temperature;

BMP180::BMP180(uint8_t i2c_address) :
    i2c_address_(i2c_address),
    pressure_(0.0),
    altitude_(0.0),
    temperature_(0.0)
{
    Wire.begin();
}

void BMP180::Begin()
{
    // TODO
}

void BMP180::Update()
{
    // TODO
}

float BMP180::GetPressure() const
{
    return pressure_;
}

float BMP180::GetAltitude() const
{
    return altitude_;
}

float BMP180::GetTemperature() const
{
    return temperature_;
}
