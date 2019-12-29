#include "PWMPinListener.hpp"

#include <ArduinoSTL.h>
#include <EnableInterrupt.h>

using namespace command;


static std::vector<PWMPinListener*> pwm_listener_instances;

static void InterruptHandler(void)
{
    for (uint8_t i = 0; i < pwm_listener_instances.size(); ++i) {
        pwm_listener_instances.at(i)->HandleInterrupt();
    }
}


PWMPinListener::PWMPinListener(uint8_t pin) :
    pin_(pin),
    value_(0),
    update_started_(false)
{
    pinMode(pin_, INPUT);

    pwm_listener_instances.push_back(this);
    enableInterrupt(pin_, InterruptHandler, CHANGE);
}

PWMPinListener::~PWMPinListener()
{
    disableInterrupt(pin_);

    for (uint8_t i = 0; i < pwm_listener_instances.size(); ++i) {
        if (pwm_listener_instances.at(i) == this) {
            pwm_listener_instances.erase(i);
        }
    }
}

uint16_t PWMPinListener::ReadChannel() const
{
    return value_;
}

void PWMPinListener::HandleInterrupt()
{
    ProcessPWM();
}

void PWMPinListener::ProcessPWM()
{
    time_.current = micros();       // Moznost na optimalizaciu!!
    // vytvorit metodu SetCurrentTime(time)
    // a zavolat ju iba raz v preruseni InterruptHandler
    // ale mozno iba trepem :D

    if (digitalRead(pin_) == HIGH) {
        if (!update_started_) {
            update_started_ = true;
            time_.start = micros();
        }
    }

    else {
        if (update_started_) {
            update_started_ = false;
            value_ = time_.current - time_.start;
        }
    }
}
