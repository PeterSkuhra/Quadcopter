#include "PWM.hpp"

#define MAX_TIMER_VALUE     65535   // (2^16) - 1

const uint8_t PWM::kTimersPins_[TIMERS_16BIT_COUNT][PWM_PIN_PER_TIMER] =
{
    // A, B, C
    {11, 12, 13},   //Timer1 pins
    {5, 2, 3},      //Timer3 pins
    {6, 7, 8},      //Timer4 pins
    {46, 45, 44}    //Timer5 pins
};

const uint16_t PWM::kPrescaler_[PRESCALER_COUNT] =
{
    1,
    8,
    64,
    256,
    1024
};

std::map<uint8_t, PWM*> PWM::instances_;

bool PWM::initialized_(false);


PWM* PWM::GetInstance(uint8_t pin)
{
    const auto &it(instances_.find(pin));

    if (it != instances_.end()) {
        return (PWM*)(it->second);
    }

    PWM* instance = new PWM(pin);
    instances_[pin] = instance;
    return instance;
}

PWM::~PWM()
{
    instances_.erase(pin_);

    // TODO: dorobit detach pinu na registre
    // zrusit nastavenu konfigurciu casovacov a ich pinov,
    // aby sa dali pouzit inde.
}

bool PWM::SetFrequency(float frequency)
{
    // TODO     f = 1 / T
    // Zistit ci je mozne nastavit zvolenu frekvenciu.
    // Ak ano, tak ju nastavit a vratit 1, inak vratit 0
    return false;
}

float PWM::GetFrequency() const
{
    return frequency_hz_;
}

bool PWM::SetPeriod(uint16_t period_us)
{
    // TODO: skontrolovat, ci je mozne nastavit zvolenu periodu.
    // Ak ano, tak nastavit a vratit 1, inak vratit 0

    uint16_t input_capture;

    for (uint8_t i = 0; i < PRESCALER_COUNT; ++i) {
        input_capture = (period_us * F_CPU) / kPrescaler_[i];
    // skontrolovat ci je to cele cislo a zaroven mensie ako max. casovaca
            //if (input_capture % )
    }

    period_us_ = period_us;
    max_pulse_us_ = period_us_;



    return false;
}

uint32_t PWM::GetPeriod() const
{
    return period_us_;
}

uint32_t PWM::GetMaxPulseMicroseconds() const
{
    return max_pulse_us_;
}

void PWM::WritePulseMicroseconds(uint32_t us)
{
    pulse_us_ = constrain(us, 0, period_us_);
    pulse_percent_ = (pulse_us_ * 100) / period_us_;

    switch (pin_) {
        case 11:
            OCR1A = pulse_us_;
            break;
        case 12:
            OCR1B = pulse_us_;
            break;
        case 13:
            OCR1C = pulse_us_;
            break;

        case 5:
            OCR3A = pulse_us_;
            break;
        case 2:
            OCR3B = pulse_us_;
            break;
        case 3:
            OCR3C = pulse_us_;
            break;

        case 6:
            OCR3A = pulse_us_;
            break;
        case 7:
            OCR3B = pulse_us_;
            break;
        case 8:
            OCR3C = pulse_us_;
            break;

        case 46:
            OCR5A = pulse_us_;
            break;
        case 45:
            OCR5B = pulse_us_;
            break;
        case 44:
            OCR5C = pulse_us_;
            break;
    }
}

uint32_t PWM::GetCurrentPulseMicroseconds() const
{
    return pulse_us_;
}

void PWM::WritePulsePercent(uint8_t percent)
{
    pulse_percent_ = constrain(percent, 0, 100);
    pulse_us_ = (pulse_percent_ * period_us_) / 100;

    WritePulseMicroseconds(pulse_us_);
}

uint8_t PWM::GetCurrentPulsePercent() const
{
    return pulse_percent_;
}

void PWM::ResetRegisters()
{
    //  Timer 1 control registers
    TCCR1A = 0;
    TCCR1B = 0;

    //  Timer 3 control registers
    TCCR3A = 0;
    TCCR3B = 0;

    //  Timer 4 control registers
    TCCR4A = 0;
    TCCR4B = 0;

    //  Timer 5 control registers
    TCCR5A = 0;
    TCCR5B = 0;
}

void PWM::InitFastPWMMode(Timers16Bit timer)
{
    switch (timer) {
        case TIMER1:
            TCCR1A |= _BV(WGM11);
            TCCR1B |= _BV(WGM12) | _BV(WGM13);
            break;

        case TIMER3:
            TCCR3A |= _BV(WGM31);
            TCCR3B |= _BV(WGM32) | _BV(WGM33);
            break;

        case TIMER4:
            TCCR4A |= _BV(WGM41);
            TCCR4B |= _BV(WGM42) | _BV(WGM43);
            break;

        case TIMER5:
            TCCR5A |= _BV(WGM51);
            TCCR5B |= _BV(WGM52) | _BV(WGM53);
            break;

        default:
            return;
    }
}

void PWM::SetPrescaler(Timers16Bit timer, Prescalers prescaler)
{
    uint8_t prescaler_bits;

    switch (prescaler) {
        case NO_PRESCALER:
            prescaler_bits = 0b001;
            break;

        case PRESCALER_8:
            prescaler_bits = 0b010;
            break;

        case PRESCALER_64:
            prescaler_bits = 0b011;
            break;

        case PRESCALER_256:
            prescaler_bits = 0b100;
            break;

        case PRESCALER_1024:
            prescaler_bits = 0b101;
            break;

        default:
            prescaler_bits = 0b000;
            break;
    }

    switch (timer) {
        case TIMER1:
            TCCR1B |= prescaler_bits;
            break;

        case TIMER3:
            TCCR3B |= prescaler_bits;
            break;

        case TIMER4:
            TCCR4B |= prescaler_bits;
            break;

        case TIMER5:
            TCCR5B |= prescaler_bits;
            break;

        default:
            return;
    }
}

Timers16Bit PWM::WhichTimer(uint8_t pin)
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

void PWM::SetInputCapture(uint8_t pin, uint16_t value)
{
    Timers16Bit timer = WhichTimer(pin);

    switch (timer) {
        case TIMER1:
            ICR1 = value;
            break;

        case TIMER3:
            ICR3 = value;
            break;

        case TIMER4:
            ICR4 = value;
            break;

        case TIMER5:
            ICR5 = value;
            break;

        case NO_TIMER:
        default:
            return;

    }
}

PWM::PWM(uint8_t pin) :
    frequency_hz_(1),
    period_us_(1000000),
    max_pulse_us_(1000000)
{
    for (auto& rows : kTimersPins_) {
        for (auto& column : rows) {
            if (column == pin) {
                if (!initialized_) {
                    ResetRegisters();
                }

                InitOutput(pin);
                InitFastPWMMode(WhichTimer(pin));
            }
        }
    }
}

void PWM::InitOutput(uint8_t pin)
{
    pin_ = pin;
    pinMode(pin_, OUTPUT);

    switch (pin_) {
        case 11:
            TCCR1A |= _BV(COM1A1);
            break;

        case 12:
            TCCR1A |= _BV(COM1B1);
            break;

        case 13:
            TCCR1A |= _BV(COM1C1);
            break;


        case 5:
            TCCR3A |= _BV(COM3A1);
            break;

        case 2:
            TCCR3A |= _BV(COM3B1);
            break;

        case 3:
            TCCR3A |= _BV(COM3C1);
            break;


        case 6:
            TCCR4A |= _BV(COM4A1);
            break;

        case 7:
            TCCR4A |= _BV(COM4B1);
            break;

        case 8:
            TCCR4A |= _BV(COM4C1);
            break;


        case 46:
            TCCR5A |= _BV(COM5A1);
            break;

        case 45:
            TCCR5A |= _BV(COM5B1);
            break;

        case 44:
            TCCR5A |= _BV(COM5C1);
            break;
    }
}
