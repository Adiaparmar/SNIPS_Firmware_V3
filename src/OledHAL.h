#pragma once
#ifdef HAS_OLED

#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"

class OledHAL {
public:
    OledHAL();
    bool begin(uint8_t i2cAddr);

    // ---- boot-time primitives (node files compose their own sequence) ----
    void bootLine(const char* text);
    void bootLineHex(const char* label, uint8_t value);            // "ID: 0x20"
    void bootLinePos(const char* label, int16_t x_cm, int16_t y_cm); // "Pos:(0.00,0.00)m"
    void bootLineFreqSF(const char* freqStr, uint8_t sf);           // "LoRa: 865.0MHz SF7"

    // ---- scrolling log ----
    void beginScrollLog();
    void logEvent(const char* text);                                  // bare literal line
    void logEventHex(const char* tag, uint8_t addr);                  // "TAG 0x20"
    void logEventHexRssi(const char* tag, uint8_t addr, int8_t rssi); // "TAG 0x20 R=-71"
    void logEventFrame(const char* tag, uint32_t frame);              // "BEACON f=142"
    void logEventHexHex(const char* tag, uint8_t a, uint8_t b);       // "FWD 0x20>0x21"
    void logEventPos(int16_t x_cm, int16_t y_cm, uint8_t confidence); // "POS (2.30,1.10) 60%"
    void logEventString(const char* prefix, const char* str);         // "Msg: Hello"

private:
    SSD1306AsciiWire _oled;

    void _printHex2(uint8_t value);          // shared zero-padded hex helper
    void _printFixedMeters(int16_t fixed);   // shared fixed-point cm -> "X.XX" printer
};

#endif // HAS_OLED
