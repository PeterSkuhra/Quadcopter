#ifndef IIMU_HPP
#define IIMU_HPP

#include <Arduino.h>

#include "ICalibratable.hpp"

namespace sensing
{
namespace imu
{

#define DEAFULT_SAMPLES     1000

class IIMU : public ICalibratable
{
public:

    virtual ~IIMU() = default;

    virtual void Begin() = 0;

    virtual void Update() = 0;

    // Acceleration
    virtual float GetXAcceleration() const = 0;
    virtual float GetYAcceleration() const = 0;
    virtual float GetZAcceleration() const = 0;

    // Angular rate
    virtual float GetXAngularRate() const = 0;
    virtual float GetYAngularRate() const = 0;
    virtual float GetZAngularRate() const = 0;

    virtual float GetRollAngularRate() const = 0;
    virtual float GetPitchAngularRate() const = 0;
    virtual float GetYawAngularRate() const = 0;

    // Angle
    virtual float GetXAngle() const = 0;
    virtual float GetYAngle() const = 0;
    virtual float GetZAngle() const = 0;

    virtual float GetRollAngle() const = 0;
    virtual float GetPitchAngle() const = 0;
    virtual float GetYawAngle() const = 0;
};

}
}

#endif
