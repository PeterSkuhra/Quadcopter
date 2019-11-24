#ifndef IMOTOR_HPP
#define IMOTOR_HPP

namespace motor
{

class IMotor
{
public:

    virtual ~IMotor() = default;

    virtual void SetSpeed(uint16_t speed) = 0;

    virtual uint16_t GetSpeed() const = 0;

    virtual uint16_t GetRPM(uint8_t battery_voltage) = 0;

};

}

#endif
