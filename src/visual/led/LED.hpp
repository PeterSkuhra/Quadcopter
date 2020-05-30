#ifndef LED_HPP
#define LED_HPP

#include <Arduino.h>

namespace visual
{

namespace led
{

/******************************************************************************
 *  Class for control LED. You can set on and off. Also you can set blinking
 *  with different on and off times.
 *****************************************************************************/
class LED
{
 public:

     /**
      * Constructor.
      *
      * @param pin  LED pin number
      */
     LED(uint8_t pin);

     /**
      * Constructor.
      *
      * @param pin          LED pin number
      * @param on_time_ms   ON time in ms during blinking.
      * @param off_time_ms  OFF time in ms during blinking.
      */
     LED(uint8_t pin, uint16_t on_time_ms, uint16_t off_time_ms);

     /**
      * Sets LED on.
      */
     void SetOn();

     /**
      * Sets LED off.
      */
     void SetOff();

     /**
      * Starts blinking.
      */
     void StartBlink();

     /**
      * Stops blinking.
      */
     void StopBlink();

     /**
      * Updates LED state during blinking.
      */
     void Update();

     /**
      * Sets ON and OFF time interval during blinking.
      *
      * @param on_time_ms   ON time in ms during blinking.
      * @param off_time_ms  OFF time in ms during blinking.
      */
     void SetBlinkInterval(uint16_t on_time_ms, uint16_t off_time_ms);

     /**
      * Returns ON time in ms.
      *
      * @return ON time in ms
      */
     uint16_t GetOnTime() const;

     /**
      * Returns OFF time in ms.
      *
      * @return OFF time in ms
      */
     uint16_t GetOffTime() const;


 private:

     /**
      * LED pin number.
      */
     const uint8_t pin_;

     /**
      * LED state (true/false = ON/OFF)
      */
     bool led_state_;

     /**
      * Blinking started.
      */
     bool blink_started_;

     /**
      * ON time in ms for blinking.
      */
     uint16_t on_time_ms_;

     /**
      * OFF time in ms for blinking.
      */
     uint16_t off_time_ms_;

     /**
      * Previous time of change LED state.
      */
     uint32_t previous_time_;
};

}
}

#endif
