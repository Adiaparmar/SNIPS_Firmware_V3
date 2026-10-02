#include "OledHAL.h"
#ifdef HAS_OLED

#include <Wire.h>
#include "snips_config.h"

OledHAL::OledHAL() {
    // deliberately does nothing hardware-related —
    // real init happens in begin(), once I2C/board is ready
}

bool OledHAL::begin(uint8_t i2cAddr) {
#ifdef ESP32
    Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN);
#else
    Wire.begin();
#endif
    _oled.begin(&Adafruit128x64, i2cAddr);
    _oled.setFont(Adafruit5x7);
    _oled.clear();
    return true;
}

// ============================================================
//  PRIVATE HELPERS
// ============================================================
void OledHAL::_printHex2(uint8_t value) {
    if (value < 0x10) {
        _oled.print('0');   // zero-pad single hex digits: 0x5 -> "05"
    }
    _oled.print(value, HEX);
}

void OledHAL::_printFixedMeters(int16_t fixed) {
    if (fixed < 0) {
        _oled.print('-');
        fixed = -fixed;
    }
    _oled.print(fixed / 100);      // whole metres, plain integer division
    _oled.print('.');
    uint8_t frac = fixed % 100;    // remaining centimetres
    if (frac < 10) {
        _oled.print('0');          // zero-pad fractional part: .5 -> ".05"
    }
    _oled.print(frac);
}

// ============================================================
//  BOOT-TIME PRIMITIVES
// ============================================================
void OledHAL::bootLine(const char* text) {
    _oled.println(text);
}

void OledHAL::bootLineHex(const char* label, uint8_t value) {
    _oled.print(label);
    _oled.print("0x");
    _printHex2(value);
    _oled.println();
}

void OledHAL::bootLinePos(const char* label, int16_t x_cm, int16_t y_cm) {
    _oled.print(label);
    _oled.print('(');
    _printFixedMeters(x_cm);
    _oled.print(',');
    _printFixedMeters(y_cm);
    _oled.println(')');
}

void OledHAL::bootLineFreqSF(const char* freqStr, uint8_t sf) {
    _oled.print("LoRa: ");
    _oled.print(freqStr);
    _oled.print(" SF");
    _oled.println(sf);
}

// ============================================================
//  SCROLLING LOG
// ============================================================
void OledHAL::beginScrollLog() {
    _oled.clear();
    _oled.setScrollMode(SCROLL_MODE_AUTO);
}

void OledHAL::logEvent(const char* text) {
    _oled.println(text);
}

void OledHAL::logEventHex(const char* tag, uint8_t addr) {
    _oled.print(tag);
    _oled.print(" 0x");
    _printHex2(addr);
    _oled.println();
}

void OledHAL::logEventHexRssi(const char* tag, uint8_t addr, int8_t rssi) {
    _oled.print(tag);
    _oled.print(" 0x");
    _printHex2(addr);
    _oled.print(" R=");
    _oled.println(rssi);
}

void OledHAL::logEventFrame(const char* tag, uint32_t frame) {
    _oled.print(tag);
    _oled.print(" f=");
    _oled.println(frame);
}

void OledHAL::logEventHexHex(const char* tag, uint8_t a, uint8_t b) {
    _oled.print(tag);
    _oled.print(" 0x");
    _printHex2(a);
    _oled.print('>');
    _oled.print("0x");
    _printHex2(b);
    _oled.println();
}

void OledHAL::logEventPos(int16_t x_cm, int16_t y_cm, uint8_t confidence) {
    _oled.print("POS (");
    _printFixedMeters(x_cm);
    _oled.print(',');
    _printFixedMeters(y_cm);
    _oled.print(") ");
    _oled.print(confidence);
    _oled.println('%');
}

void OledHAL::logEventString(const char* prefix, const char* str) {
    _oled.print(prefix);
    _oled.print(" ");
    _oled.println(str);
}

#endif // HAS_OLED
