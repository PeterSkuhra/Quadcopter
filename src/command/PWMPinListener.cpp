#include "PWMPinListener.hpp"

#include <ArduinoSTL.h>

#define LIBCALL_ENABLEINTERRUPT
#include <EnableInterrupt.h>


static std::vector<command::PWMPinListener*> pwm_listener_instances;

static void InterruptHandler(void)
{
    uint16_t current_time = micros();

    for (uint8_t i = 0; i < pwm_listener_instances.size(); ++i) {
        pwm_listener_instances.at(i)->HandleInterrupt(current_time);
    }
}


command::PWMPinListener::PWMPinListener(uint8_t pin) :
    pin_(pin),
    value_(0),
    update_started_(false)
{
    pinMode(pin_, INPUT);

    noInterrupts();
    enableInterrupt(pin_, InterruptHandler, CHANGE);
    interrupts();

    pwm_listener_instances.push_back(this);
}

command::PWMPinListener::~PWMPinListener()
{
    disableInterrupt(pin_);

    for (uint8_t i = 0; i < pwm_listener_instances.size(); ++i) {
        if (pwm_listener_instances.at(i) == this) {
            pwm_listener_instances.erase(pwm_listener_instances.begin() + i);
        }
    }
}

uint16_t command::PWMPinListener::ReadChannel() const
{
    return value_;
}

void command::PWMPinListener::HandleInterrupt(uint16_t current_time)
{
    if (digitalRead(pin_)) {
        if (!update_started_) {
            update_started_ = true;
            time_.start = micros();
        }
    }
    else {
        if (update_started_) {
            update_started_ = false;
            value_ = current_time - time_.start;
        }
    }
}
