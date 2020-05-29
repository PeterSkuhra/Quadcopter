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

#define DEFAULT_ADDRESS     0x3C

#define SCREEN_WIDTH        128
#define SCREEN_HEIGHT       32


class OLED_128x32 : public IDisplay
{
public:

    OLED_128x32(const char* init_text,
                const uint8_t i2c_address = DEFAULT_ADDRESS);

    void Begin() override;

    void Clear() override;

    void Print(const char* text) override;


private:

    const uint8_t address_;

    const char* init_text_;

    std::deque<char*> text_buffer_;

    Adafruit_SSD1306* oled_display_;
};

}
}

#endif
