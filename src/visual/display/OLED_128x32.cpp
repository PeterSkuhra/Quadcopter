#include "OLED_128x32.hpp"

#define TEXT_SIZE   1
#define TEXT_COLOR  SSD1306_WHITE


visual::display::OLED_128x32::OLED_128x32(const char* init_text,
                                          const uint8_t i2c_address) :
    address_(i2c_address),
    init_text_(init_text)
{
    oled_display_ = new Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

    text_buffer_.clear();
}

void visual::display::OLED_128x32::Begin()
{
    oled_display_->begin(SSD1306_SWITCHCAPVCC, address_);
    oled_display_->clearDisplay();

    oled_display_->setTextSize(TEXT_SIZE);
    oled_display_->setTextColor(TEXT_COLOR);


    oled_display_->cp437(true);


    oled_display_->print(init_text_);
    oled_display_->display();
    delay(2000);
    oled_display_->clearDisplay();
}

void visual::display::OLED_128x32::Clear()
{
    text_buffer_.clear();
    oled_display_->clearDisplay();
    oled_display_->setCursor(0, 0);
    oled_display_->display();
}

void visual::display::OLED_128x32::Print(const char* text)
{
    text_buffer_.push_front(text);

    while (text_buffer_.size() > 4) {
        text_buffer_.pop_back();
    }

    oled_display_->clearDisplay();
    oled_display_->setCursor(0, 0);
    oled_display_->display();


    for (uint8_t i = 0; i < text_buffer_.size(); ++i) {
        int j = (text_buffer_.size() - 1) - i;
        oled_display_->println(text_buffer_.at(j));
    }

    oled_display_->display();
}
