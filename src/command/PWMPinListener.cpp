#include "PWMPinListener.hpp"

#include <ArduinoSTL.h>

#define LIBCALL_ENABLEINTERRUPT
#include <EnableInterrupt.h>


//=============================================================================

static void HandleInterrupt1();
static void HandleInterrupt2();
static void HandleInterrupt3();
static void HandleInterrupt4();
static void HandleInterrupt5();
static void HandleInterrupt6();
static void HandleInterrupt7();
static void HandleInterrupt8();
static void HandleInterrupt9();
static void HandleInterrupt10();

typedef void (*isr_p)(void);

static const std::vector<isr_p> isr_functions{
    &HandleInterrupt1,
    &HandleInterrupt2,
    &HandleInterrupt3,
    &HandleInterrupt4,
    &HandleInterrupt5,
    &HandleInterrupt6,
    &HandleInterrupt7,
    &HandleInterrupt8,
    &HandleInterrupt9,
    &HandleInterrupt10
};

static std::vector<command::PWMPinListener*> instances;
//=============================================================================

command::PWMPinListener::PWMPinListener(uint8_t pin) :
    pin_(pin),
    value_(0),
    update_started_(false)
{
    pinMode(pin_, INPUT);

    noInterrupts();
    enableInterrupt(pin_, isr_functions[instances.size()], CHANGE);
    interrupts();

    instances.push_back(this);
}

command::PWMPinListener::~PWMPinListener()
{
    disableInterrupt(pin_);

    for (uint8_t i = 0; i < instances.size(); ++i) {
        if (instances.at(i) == this) {
            instances.erase(instances.begin() + i);
        }
    }
}

uint16_t command::PWMPinListener::ReadChannel() const
{
    return value_;
}

void command::PWMPinListener::HandleInterrupt()
{
    if (digitalRead(pin_)) {
        if (!update_started_) {
            update_started_ = true;
            time_start_ = micros();
        }
    }
    else {
        if (update_started_) {
            update_started_ = false;
            value_ = micros() - time_start_;
        }
    }
}


//=============================================================================

static void HandleInterrupt1()
{
    if (instances.size() > 0) {
        instances[0]->HandleInterrupt();
    }
}

static void HandleInterrupt2()
{
    if (instances.size() > 1) {
        instances[1]->HandleInterrupt();
    }
}

static void HandleInterrupt3()
{
    if (instances.size() > 2) {
        instances[2]->HandleInterrupt();
    }
}

static void HandleInterrupt4()
{
    if (instances.size() > 3) {
        instances[3]->HandleInterrupt();
    }
}

static void HandleInterrupt5()
{
    if (instances.size() > 4) {
        instances[4]->HandleInterrupt();
    }
}

static void HandleInterrupt6()
{
    if (instances.size() > 5) {
        instances[5]->HandleInterrupt();
    }
}

static void HandleInterrupt7()
{
    if (instances.size() > 6) {
        instances[6]->HandleInterrupt();
    }
}

static void HandleInterrupt8()
{
    if (instances.size() > 7) {
        instances[7]->HandleInterrupt();
    }
}

static void HandleInterrupt9()
{
    if (instances.size() > 8) {
        instances[8]->HandleInterrupt();
    }
}

static void HandleInterrupt10()
{
    if (instances.size() > 9) {
        instances[9]->HandleInterrupt();
    }
}
