#include "BrushlessMotor.hpp"

#include <TimerOne.h>

#define MOTOR_KV            920     // rpm/V
#define ESC_RESPONSE_RATE   490     // Hz
#define PWM_PERIOD_US       2040    // 1 / 490 Hz ~= 2040 µs

#define MIN_PULSE           1000    // µs
#define MAX_PULSE           2000    // µs

#define MIN_PWM             0
#define MAX_PWM             1023

motor::BrushlessMotor::BrushlessMotor(uint8_t pin) :
    pin_(pin),
    speed_(0),
    rpm_(0)
{
    Timer1.initialize(PWM_PERIOD_US);
    SetSpeed(0);
}

motor::BrushlessMotor::~BrushlessMotor()
{
    SetSpeed(0);        // TODO: otestovať zastavovanie motora!!!
                        // dorobit ciselny rozsah zastavenia motora
    Timer1.~TimerOne();
}

void motor::BrushlessMotor::SetSpeed(uint16_t speed)
{
    speed = constrain(speed, MIN_PULSE, MAX_PULSE);
    speed_ = map(speed, MIN_PULSE, MAX_PULSE, MIN_PWM, MAX_PWM);

    Timer1.pwm(pin_, speed_);
}

uint16_t motor::BrushlessMotor::GetSpeed() const
{
    return map(speed_, MIN_PWM, MAX_PWM, MIN_PULSE, MAX_PULSE);
}

uint16_t motor::BrushlessMotor::GetRPM(uint8_t battery_voltage)
{
    const uint16_t speed_range = MAX_PULSE - MIN_PULSE;
    const float rpm_percent_coefficient = (speed_ - speed_range) / speed_range;

    rpm_ = MOTOR_KV * battery_voltage * rpm_percent_coefficient;

    return rpm_;
}
