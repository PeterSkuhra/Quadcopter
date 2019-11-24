#ifndef IBAROMETER_HPP
#define IBAROMETER_HPP

namespace sensing
{
namespace pressure
{

class IBarometer
{
public:

    virtual ~IBarometer() = default;

    virtual void Begin() = 0;

    virtual void Update() = 0;

    virtual float GetPressure() const = 0;

    virtual float GetAltitude() const = 0;
};

}
}

#endif
