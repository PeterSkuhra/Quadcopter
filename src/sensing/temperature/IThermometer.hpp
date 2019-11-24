#ifndef ITHERMOMETER_HPP
#define ITHERMOMETER_HPP

namespace sensing
{
namespace temperature
{

class IThermometer
{
public:

    virtual ~IThermometer() = default;

    virtual void Update() = 0;

    virtual float GetTemperature() const = 0;
};

}
}

#endif
