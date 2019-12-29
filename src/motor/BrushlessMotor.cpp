#include "BrushlessMotor.hpp"

#define MOTOR_KV            920     // rpm/V
#define PWM_PERIOD_US       2048    // 2^11
#define ESC_RESPONSE_RATE   488     // 1 / 2048e-6 ~= 488Hz


#define MIN_PULSE           1000    // µs
#define MAX_PULSE           2000    // µs

#define MIN_PWM             0
#define MAX_PWM             1023


motor::BrushlessMotor::BrushlessMotor(uint8_t pin) :
    pin_(pin),
    period_(PWM_PERIOD_US / 1000000),
    speed_(0),
    rpm_(0)
{
    pwm_ = PWM::GetInstance(pin);

    SetSpeed(0);
}

motor::BrushlessMotor::~BrushlessMotor()
{
    SetSpeed(0);        // TODO: otestovať zastavovanie motora!!!
                        // dorobit ciselny rozsah zastavenia motora

}

void motor::BrushlessMotor::SetSpeed(uint16_t speed)
{
    speed = constrain(speed, MIN_PULSE, MAX_PULSE);
    speed_ = map(speed, MIN_PULSE, MAX_PULSE, MIN_PWM, MAX_PWM);

    // TODO

    pwm_->WritePulseMicroseconds(speed_);
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

// void motor::BrushlessMotor::Init(uint8_t pin, uint16_t frequency)
// {
//     switch (pin) {
//         case 11:
//             break;
//         case 12:
//             break;
//         case 13:
//             break;
//
//         case 5:
//             break;
//         case 2:
//             break;
//         case 3:
//             break;
//
//         case 6:
//             break;
//         case 7:
//             break;
//         case 8:
//             break;
//
//         case 46:
//             break;
//         case 45:
//             break;
//         case 44:
//             break;
//
//         default:
//             break;
//     }
// }
