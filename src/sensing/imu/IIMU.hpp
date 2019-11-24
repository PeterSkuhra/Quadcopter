#ifndef IIMU_HPP
#define IIMU_HPP

#include <Arduino.h>


namespace sensing
{
namespace imu
{

#define DEAFULT_SAMPLES     1000

class IIMU
{
public:

    virtual ~IIMU() = default;

    virtual void Begin() = 0;

    virtual void Update() = 0;

    virtual float GetRoll() const = 0;
    virtual float GetPitch() const = 0;
    virtual float GetYaw() const = 0;
};

}
}

#endif
