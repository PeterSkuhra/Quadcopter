#ifndef OLED_128X32_HPP
#define OLED_128X32_HPP

#include "IDisplay.hpp"

#include <SBWire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <ArduinoSTL.h>
#include <deque>

namespace visual
{

namespace display
{

/**
 *  Default I2C addres of OLED display (128x32)
 */
#define DEFAULT_ADDRESS     0x3C

/**
 *  OLED Screen width
 */
#define SCREEN_WIDTH        128

/**
 *  OLED screen height
 */
#define SCREEN_HEIGHT       32


/******************************************************************************
 *  Wrapper class for OLED display with resolution 128x32.
 *  Implements IDisplay interface.
 *
 *  Provides printing text to display.
 *  Uses the fifo buffer to shows the last four strings.
 *****************************************************************************/
class OLED_128x32 : public IDisplay
{
 public:

     /**
      * Constructor.
      *
      * @param init_text        text displayed during initialization
      * @param i2i2c_address    I2C address of OLED display (optional)
      */
     OLED_128x32(const char* init_text,
                const uint8_t i2c_address = DEFAULT_ADDRESS);

     /**
      * Initializes OLED display.
      */
     void Begin() override;

     /**
      * Clear the while display.
      */
     void Clear() override;

     /**
      * Prints the entered text on the display.
      * Keeps last four string printed on display.
      *
      * @param text     text to printig
      */
     void Print(const char* text) override;


 private:

     /**
      * I2C address of OLED display.
      */
     const uint8_t address_;

     /**
      * Text displayed during initialization.
      */
     const char* init_text_;

     /**
      * Buffer for keeping last four strings.
      */
     std::deque<char*> text_buffer_;

     /**
      * Instance OLED display.
      */
     Adafruit_SSD1306* oled_display_;
};

}
}

#endif
