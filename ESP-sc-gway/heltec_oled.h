#pragma once

#include <Arduino.h>
#include <Wire.h>

enum HeltecTextAlignment {
    TEXT_ALIGN_LEFT,
    TEXT_ALIGN_CENTER,
    TEXT_ALIGN_RIGHT
};

struct HeltecFont {
    uint8_t baseWidth;
    uint8_t baseHeight;
    uint8_t scale;
};

extern const HeltecFont Heltec_Arial16;
extern const HeltecFont Heltec_Arial24;

class HeltecSSD1306 {
public:
    HeltecSSD1306(uint8_t address, int sdaPin, int sclPin, int rstPin = -1);

    bool init();
    bool begin() { return init(); }

    void flipScreenVertically();

    void setFont(const HeltecFont *font);
    void setTextAlignment(HeltecTextAlignment alignment);

    void clear();
    void clearDisplay() { clear(); }
    void display();

    void drawString(int16_t x, int16_t y, const String &text);

    void displayOn();
    void displayOff();

private:
    void drawChar(int16_t x, int16_t y, char c);
    void drawPixel(int16_t x, int16_t y, bool on = true);
    void writeCommand(uint8_t command) const;

    uint8_t address_;
    int sda_;
    int scl_;
    int rst_;

    bool initialized_ = false;
    bool flipVertical_ = false;

    uint8_t fontScale_ = 2; // default ~16px
    HeltecTextAlignment alignment_ = TEXT_ALIGN_LEFT;

    static constexpr uint8_t width_ = 128;
    static constexpr uint8_t height_ = 64;
    static constexpr size_t bufferSize_ = (width_ * height_) / 8;
    uint8_t buffer_[bufferSize_];
};
