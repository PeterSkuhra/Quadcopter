#ifndef PWM_HPP
#define PWM_HPP

#include <Arduino.h>
#include <ArduinoSTL.h>
#include <map>

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)

#define TIMERS_16BIT_COUNT  4
#define PWM_PIN_PER_TIMER   3
#define PRESCALER_COUNT     5

enum Timers16Bit
{
    TIMER1 = 1,
    TIMER3 = 3,
    TIMER4 = 4,
    TIMER5 = 5,
    NO_TIMER = 255
};

class PWM
{
public:
    static PWM* GetInstance(uint8_t pin);


    ~PWM();

    bool SetFrequency(float frequency);

    float GetFrequency() const;

    bool SetPeriod(uint16_t period_us);

    uint32_t GetPeriod() const;

    uint32_t GetMaxPulseMicroseconds() const;

    void WritePulseMicroseconds(uint32_t us);

    uint32_t GetCurrentPulseMicroseconds() const;

    void WritePulsePercent(uint8_t percent);

    uint8_t GetCurrentPulsePercent() const;


private:
    static void ResetRegisters();

    static void InitFastPWMMode(Timers16Bit timer);

    static void SetPrescaler(Timers16Bit timer, uint16_t prescaler);

    static Timers16Bit WhichTimer(uint8_t pin);

    static void SetInputCapture(uint8_t pin, uint16_t value);


    PWM(uint8_t pin);

    void InitOutput(uint8_t pin);



private:
    static const uint8_t timers_pins_[TIMERS_16BIT_COUNT][PWM_PIN_PER_TIMER];

    static const uint16_t prescaler[PRESCALER_COUNT];

    static std::map<uint8_t, PWM*> instances_;

    static bool initialized_;


    uint8_t pin_;

    float frequency_hz_;

    uint32_t period_us_;

    uint32_t max_pulse_us_;

    uint32_t pulse_us_;

    uint8_t pulse_percent_;
};

#endif

#endif
