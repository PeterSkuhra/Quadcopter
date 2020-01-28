#include "BrushlessMotor.hpp"

#define MOTOR_KV            920     // rpm/V
#define PWM_PERIOD_US       2048    // 2^11
#define ESC_RESPONSE_RATE   488     // 1 / 2048e-6 ~= 488Hz


#define MIN_PULSE           1000    // µs
#define MAX_PULSE           2000    // µs


motor::BrushlessMotor::BrushlessMotor(uint8_t pin) :
    pin_(pin),
    speed_(0),
    rpm_(0)
{
    pwm_ = PWM::GetInstance(pin);
    pwm_->SetPeriodMicroseconds(PWM_PERIOD_US);
}

motor::BrushlessMotor::~BrushlessMotor()
{
    SetSpeed(0);
    delete pwm_;
}

void motor::BrushlessMotor::SetSpeed(uint16_t speed)
{
    speed_ = constrain(speed, MIN_PULSE, MAX_PULSE);

    pwm_->WritePulseMicroseconds(speed_);
}

uint16_t motor::BrushlessMotor::GetSpeed() const
{
    return speed_;
}

uint16_t motor::BrushlessMotor::GetRPM(uint8_t battery_voltage)
{
    const uint16_t speed_range = MAX_PULSE - MIN_PULSE;
    const float rpm_percent_coefficient = (speed_ - speed_range) / speed_range;

    rpm_ = MOTOR_KV * battery_voltage * rpm_percent_coefficient;

    return rpm_;
}
