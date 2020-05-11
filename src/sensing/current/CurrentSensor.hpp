#ifndef CURRENT_SENSOR_HPP
#define CURRENT_SENSOR_HPP

#include "../voltage/ISensor.hpp"

namespace sensing
{
namespace current
{

#define CURRENT_5A      (0.185)
#define CURRENT_20A     (0.100)
#define CURRENT_30A     (0.066)


class CurrentSensor : public voltage::ISensor
{
public:

    CurrentSensor(const uint8_t pin, float resolution = CURRENT_30A);

    bool GetDigitalValue() override;

    // returns current in mA !!!
    float GetAnalogValue() override;

    void SetResolution(float resolution) override;

    float GetResolution() const override;

private:
    const uint8_t pin_;

    float resolution_;

    float current_value_;
};

}
}

#endif
