#ifndef PWM_HPP
#define PWM_HPP

#include <Arduino.h>
#include <ArduinoSTL.h>
#include <map>

const uint8_t group_16bit_timers[4][3] =
    {
        {11, 12, 13},
        {2, 3, 5},
        {6, 7, 8},
        {38, 39, 40}
    };

const uint16_t prescaler[5] = {1, 8, 64, 256, 1024};

class PWM
{
 public:
     static PWM* GetInstance(uint8_t pin);

     // void SetFrequency(uint8_t group, uint32_t frequency);
     //
     // void SetPeriod(uint32_t period);
     //
     // void SetPulse(uint32_t pulse_us);

 private:

     static std::map<uint8_t, PWM*> instances_;

     PWM(uint8_t pin);


};

#endif
