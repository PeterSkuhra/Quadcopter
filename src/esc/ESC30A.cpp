#include "ESC30A.hpp"
#include <math.h>

namespace esc
{

#define MOTOR_KV            920     // rpm/V
#define PWM_PERIOD_US       2048    // 2^11
#define ESC_RESPONSE_RATE   488     // 1 / 2048e-6 ~= 488Hz

#define REFERENCE_VOLTAGE   11.1    // 3S =>    3 * 3.7V = 11.1V
#define MAX_RPM             (MOTOR_KV * REFERENCE_VOLTAGE)

}

esc::ESC30A::ESC30A(uint8_t pin) :
    pin_(pin),
    speed_(0),
    rpm_(0)
{
    InitRegisters(GetTimer(pin_));
    pinMode(pin_, OUTPUT);
    SetSpeed(MIN_PULSE);
}

esc::ESC30A::~ESC30A()
{
    SetSpeed(0);
}

void esc::ESC30A::SetSpeed(uint16_t speed)
{
    if (speed == 0) {
        WritePWM12bit(pin_, 0);
        return;
    }

    speed_ = constrain(speed, MIN_PULSE, MAX_PULSE);

    WritePWM12bit(pin_, speed_ * 2);
}

// TEST!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
void esc::ESC30A::SetSpeed(uint16_t speed, float battery_voltage)
{
    speed_ = constrain(speed, MIN_PULSE, MAX_PULSE);

    const float speed_percent_coefficient =
        (speed_ - RANGE_PULSE) / RANGE_PULSE;

    const float wanted_rpm =
        (MOTOR_KV * REFERENCE_VOLTAGE) * speed_percent_coefficient;

    const float real_rpm =
        (MOTOR_KV * battery_voltage) * speed_percent_coefficient;

    const float diff_rpm = wanted_rpm - real_rpm;

    const int16_t compensated_speed = static_cast<int16_t>(
        round((diff_rpm / MAX_RPM) * RANGE_PULSE));

    speed_ += compensated_speed;
    speed_ = constrain(speed_, MIN_PULSE, MAX_PULSE);
    WritePWM12bit(pin_, speed_ * 2);
}

uint16_t esc::ESC30A::GetSpeed() const
{
    return speed_;
}

uint16_t esc::ESC30A::GetRPM(float battery_voltage)
{
    const float rpm_percent_coefficient = (speed_ - RANGE_PULSE) / RANGE_PULSE;

    rpm_ = MOTOR_KV * battery_voltage * rpm_percent_coefficient;

    return rpm_;
}

void esc::ESC30A::InitRegisters(Timers16Bit timer)
{
    noInterrupts();
    switch(timer) {
        case TIMER1:
            TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(COM1C1) | _BV(WGM11);
            TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11);
            ICR1 = 4096;
            break;

        case TIMER3:
            TCCR3A = _BV(COM3A1) | _BV(COM3B1) | _BV(COM3C1) | _BV(WGM31);
            TCCR3B = _BV(WGM33) | _BV(WGM32) | _BV(CS31);
            ICR3 = 4096;
            break;

        case TIMER4:
            TCCR4A = _BV(COM4A1) | _BV(COM4B1) | _BV(COM4C1) | _BV(WGM41);
            TCCR4B = _BV(WGM43) | _BV(WGM42) | _BV(CS41);
            ICR4 = 4096;
            break;

        case TIMER5:
            TCCR5A = _BV(COM5A1) | _BV(COM5B1) | _BV(COM5C1) | _BV(WGM51);
            TCCR5B = _BV(WGM53) | _BV(WGM52) | _BV(CS51);
            ICR5 = 4096;
            break;

        default:
            break;
    }
    interrupts();
}

esc::ESC30A::Timers16Bit esc::ESC30A::GetTimer(uint8_t pin)
{
    switch (pin) {
        case 11:
        case 12:
        case 13:
            return TIMER1;

        case 5:
        case 2:
        case 3:
            return TIMER3;

        case 6:
        case 7:
        case 8:
            return TIMER4;

        case 46:
        case 45:
        case 44:
            return TIMER5;

        default:
            return NO_TIMER;
    }
}

void esc::ESC30A::WritePWM12bit(uint8_t pin, uint16_t value)
{
    switch (pin) {
        case 11:
            OCR1A = value;
            break;
        case 12:
            OCR1B = value;
            break;
        case 13:
            OCR1C = value;
            break;

        case 5:
            OCR3A = value;
            break;
        case 2:
            OCR3B = value;
            break;
        case 3:
            OCR3C = value;
            break;

        case 6:
            OCR4A = value;
            break;
        case 7:
            OCR4B = value;
            break;
        case 8:
            OCR4C = value;
            break;

        case 46:
            OCR5A = value;
            break;
        case 45:
            OCR5B = value;
            break;
        case 44:
            OCR5C = value;
            break;

        default:
            break;
    }
}
