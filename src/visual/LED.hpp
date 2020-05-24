#ifndef LED_HPP
#define LED_HPP

#include <Arduino.h>

namespace visual
{

class LED
{
public:

    LED(uint8_t pin);

    LED(uint8_t pin, uint16_t on_time_ms, uint16_t off_time_ms);

    void SetOn();

    void SetOff();

    void StartBlink();

    void StopBlink();

    void Update();

    void SetBlinkInterval(uint16_t on_time_ms, uint16_t off_time_ms);

    uint16_t GetOnTime() const;

    uint16_t GetOffTime() const;


private:

    const uint8_t pin_;

    bool led_state_;

    bool blink_started_;

    uint16_t on_time_ms_;

    uint16_t off_time_ms_;

    uint32_t previous_time_;
};

}

#endif
