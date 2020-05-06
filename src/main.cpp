#include <avr/wdt.h>
#include <Arduino.h>
#include <EnableInterrupt.h>

#include "control/IController.hpp"
#include "control/FlightController.hpp"


#define LED_PIN     23
bool led_state = false;

static int get_free_memory()
{
    extern char     __bss_end;
    extern char     *__brkval;
    int             free_memory;

    if ((int)__brkval == 0) {
        free_memory = ((int)&free_memory) - ((int)&__bss_end);
    }
    else {
        free_memory = ((int)&free_memory) - ((int)__brkval);
    }

    return free_memory;
}

static void soft_restart()
{
    asm volatile("jmp 0");
}

static void debug_setup()
{
    pinMode(LED_PIN, OUTPUT);
    pinMode(22, OUTPUT);
    digitalWrite(22, LOW);
}

static void debug_loop()
{
    led_state = !led_state;
    digitalWrite(LED_PIN, led_state);
    // PORTB ^= (1 << PB7);

    // Serial.print(F("Free RAM: "));
    // Serial.println(get_free_memory());
}

void enableWatchdog()
{
    noInterrupts();
    WDTCSR = 0x00;

    WDTCSR = (1 << WDCE) |
             (1 << WDE) |
             (0 << WDP0) |
             (1 << WDP1) |  // 64ms
             (0 << WDP2) |
             (0 << WDP3);

     WDTCSR = (1 << WDCE) |
              (0 << WDE) |
              (1 << WDIE);      // ISR enable

    interrupts();
}

void disableWatchdog()
{
  cli();
  wdt_reset();
  MCUSR &= ~(1<<WDRF);
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  WDTCSR = 0x00;
  sei();
}

ISR(WDT_vect)
{
    //WDTCSR |= (1 << WDIE);
    digitalWrite(22, HIGH);
    while(true);
}

control::IController* flight_controller;

void setup()
{
    Serial.begin(115200);

    flight_controller = new control::FlightController();
    flight_controller->Init();

    debug_setup();

    // wdt_enable(WDTO_30MS);
    // enableWatchdog();
    wdt_reset();
}

void loop()
{
    flight_controller->Control();

    debug_loop();
    // wdt_reset();
}
