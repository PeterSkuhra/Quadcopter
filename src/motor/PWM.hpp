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

/******************************************************************************
 *  Class for generate PWM signal on Arduino MEGA in FastPWM mode.
 *  You can configure frequency or period of PWM signal,
 *  sets pulse in microseconds or in percent of duty.
 *
 *  Fast PWM mode:
 *  At the beginning of the period, the output value on pin
 *  is equal to logical true. When a timer value equal to the comparison
 *  register is reached, the output is turned off.
 *
 *  The multiton pattern is aplied.
 *
 *  Class uses 16-bit Timers (Arduino Mega):
 *  Timer 1 controls pins 11, 12, 13
 *  Timer 3 controls pins 2, 3, 5
 *  Timer 4 controls pins 6, 7, 8
 *  Timer 5 controls pins 38, 39, 40
 *
 *  If you will use some pin from any Timer, you shouldn't use other pins from
 *  this group for other functions.
 *  Any pin configuration also affects others pins from group.
 *
 *****************************************************************************/
class PWM
{
public:
    /**
     *  Returns pointer of PWM instance.
     *
     *  @param pin      pin number
     *
     *  @return PWM*    pointer of PWM instance
     */
    static PWM* GetInstance(uint8_t pin);

    /**
     *  Destructor.
     */
    ~PWM();

    /**
     *  Sets frequency of PWM signal if it's possible.
     *
     *  @param frequency    chosen frequency in Hz
     *
     *  @return bool        true if it's possible to set chosen frequency
     */
    bool SetFrequencyHz(uint16_t frequency);

    /**
     *  Returns current     frequency in Hz.
     *
     *  @return uint16_t    current frequency in Hz
     */
    uint16_t GetFrequencyHz() const;

    /**
     *  Sets period of PWM signal in microseconds.
     *
     *  @param period_us    period of PWM signal in microseconds
     *
     *  @return bool        true if it's possible to sets chosen period
     */
    bool SetPeriodMicroseconds(uint16_t period_us);

    /**
     *  Returns current value of period of PWM signal.
     *
     *  @return uint32_t current value of period
     */
    uint32_t GetPeriodMicroseconds() const;

    /**
     *  Sets pulse of PWM signal to chosen value in microseconds.
     *
     *  @param us   microseconds of pulse
     */
    void WritePulseMicroseconds(uint32_t us);

    /**
     *  Returns current pulse of PWM signal in microseconds.
     *
     *  @return uint32_t current pulse of PWM signal in microseconds
     */
    uint32_t GetCurrentPulseMicroseconds() const;

    /**
     *  Sets pulse of PWM signal to chosen value in percent of period.
     *
     *  @param percent  pulse value in percent of period
     */
    void WritePulsePercent(uint8_t percent);

    /**
     *  Returns current pulse of PWM signal in percent.
     *
     *  @return uint8_t current pulse of PWM signal in percent
     */
    uint8_t GetCurrentPulsePercent() const;


private:
    /**
     *  static method
     *
     *  Resets control registers for Timers (1, 3, 4, 5).
     */
    static void ResetRegisters();

    /**
     *  static method
     *
     *  Initializes control registers for FastPWM mode.
     */
    static void InitFastPWMMode(Timers16Bit timer);

    /**
     *  static method
     *
     *  Sets chosen prescaler for the appropriate timer.
     *
     *  @return bool    true if chosen prescaler is valid
     */
    static bool SetPrescaler(Timers16Bit timer, uint16_t prescaler);

    /**
     *  static method
     *
     *  Returns the appropriate timer for the selected pin.
     *
     *  @param pin      pin number
     *
     *  @return Timers16Bit     enum type Timer for selected pin
     */
    static Timers16Bit WhichTimer(uint8_t pin);

    /**
     *  static method
     *
     *  Sets ICR register for defined timer to chosen value.
     *
     *  @return void
     */
    static void SetInputCapture(Timers16Bit timer, uint16_t value);

    /**
     *  Disable all settings of timer for generate PWM signal.
     *
     *  @param timer    timer for disable settings
     */
    static void DisablePWM(Timers16Bit timer);

    /**
     *  Constructor.
     *
     *  @param pin  pin number to generate PWM signal
     */
    PWM(uint8_t pin);

    /**
     *  Initializes pin as output and configures it for generate PWM.
     *
     *  @param pin  pin number
     */
    void InitOutput(uint8_t pin);


private:
    /**
     *  2D array of timers pins.
     */
    static const uint8_t kTimersPins_[TIMERS_16BIT_COUNT][PWM_PIN_PER_TIMER];

    /**
     *  Array of available prescalers.
     */
    static const uint16_t kPrescaler_[PRESCALER_COUNT];

    /**
     *  Map of instances of PWM class - multiton pattern
     */
    static std::map<uint8_t, PWM*> instances_;

    /**
     *  Bit for information of initialization status.
     */
    static bool initialized_;

    /**
     *  Pin number for generate PWM signal.
     */
    uint8_t pin_;

    /**
     *  Current prescaler.
     */
    uint16_t prescaler_;

    /**
     *  Current frequency of PWM signal.
     */
    uint16_t frequency_hz_;

    /**
     *  Current period of PWM signal in microseconds.
     */
    uint32_t period_us_;

    /**
     *  Current generated pulse of PWM signal in microseconds.
     */
    uint32_t pulse_us_;

    /**
     *  Current generated pulse of PWM signal in percent.
     */
    uint8_t pulse_percent_;
};

#endif

#endif
