#ifndef ISENSOR_HPP
#define ISENSOR_HPP

namespace sensing
{

class ISensor
{
public:

    virtual ~ISensor() = default;

    virtual bool GetState() = 0;

    virtual float GetAnalogValue() = 0;

};

}

#endif
