#include "LED.hpp"

visual::led::LED::LED(uint8_t pin) :
    pin_(pin),
    led_state_(false),
    on_time_ms_(500),
    off_time_ms_(500)
{
    pinMode(pin_, OUTPUT);
}

visual::led::LED::LED(uint8_t pin, uint16_t on_time_ms, uint16_t off_time_ms) :
    pin_(pin),
    led_state_(false),
    on_time_ms_(on_time_ms),
    off_time_ms_(off_time_ms)
{
    pinMode(pin_, OUTPUT);
}

void visual::led::LED::SetOn()
{
    led_state_ = true;
    blink_started_ = false;
    digitalWrite(pin_, led_state_);
}

void visual::led::LED::SetOff()
{
    led_state_ = false;
    blink_started_ = false;
    digitalWrite(pin_, led_state_);
}

void visual::led::LED::StartBlink()
{
    blink_started_ = true;
}

void visual::led::LED::StopBlink()
{
    blink_started_ = false;
    led_state_ = false;
    digitalWrite(pin_, led_state_);
}

void visual::led::LED::Update()
{
    if (blink_started_) {
        uint32_t current_time = millis();
        uint32_t elapsed_time = current_time - previous_time_;

        if (led_state_ && (elapsed_time >= on_time_ms_)) {
            led_state_ = false;
            previous_time_ = current_time;
        }
        else if (!led_state_ && (elapsed_time >= off_time_ms_)) {
            led_state_ = true;
            previous_time_ = current_time;
        }

        digitalWrite(pin_, led_state_);
    }
}

void visual::led::LED::SetBlinkInterval(uint16_t on_time_ms, uint16_t off_time_ms)
{
    on_time_ms_ = on_time_ms;
    off_time_ms_ = off_time_ms;
}

uint16_t visual::led::LED::GetOnTime() const
{
    return on_time_ms_;
}

uint16_t visual::led::LED::GetOffTime() const
{
    return off_time_ms_;
}
