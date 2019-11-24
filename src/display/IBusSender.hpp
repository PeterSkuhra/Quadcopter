#ifndef IBUS_SENDER_HPP
#define IBUS_SENDER_HPP

#include <IBusBM.h>

#include "ISender.hpp"


namespace display
{

class IBusSender : public ISender
{
public:

    IBusSender(HardwareSerial& serial);

    void WriteBatteryVoltage(uint8_t voltage) override;

    void WriteRPM(uint16_t rpm) override;

    void WriteTempetrature(int8_t temperature) override;


private:

    struct Sensor {
        uint8_t number;
        int16_t value;
    };

    inline void WriteValue(Sensor& sensor, int16_t value);


private:

    IBusBM ibus_;

    Sensor battery_voltage_;
    Sensor rpm_;
    Sensor temperature_;
};

}

#endif
