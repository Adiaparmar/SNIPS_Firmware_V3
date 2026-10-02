// ============================================================
//  SNIPS V3 — Debug Log Shim
//
//  DLOG(fmt, ...) prints to Serial on every node. When HAS_OLED
//  is enabled, the same message is also mirrored to OledHAL
//  so the mobile/anchor builds keep their scrolling display.
// ============================================================
#pragma once

#include <Arduino.h>

// ------------------------------------------------------------
// fmt1f(val, buf) — format a float to 1 decimal place into buf
// using dtostrf() instead of "%f". This avoids pulling the AVR
// float-formatting path into vsnprintf/snprintf callers.
// buf must be at least DLOG_FLTBUF_LEN bytes.
// ------------------------------------------------------------
#define DLOG_FLTBUF_LEN 12
static inline char *fmt1f(float val, char *buf) {
  dtostrf(val, 1, 1, buf);
  return buf;
}

#ifdef NODE_TYPE_ANCHOR
#ifdef HAS_OLED
#include "OledHAL.h"
extern OledHAL oled;

#define DLOG(fmt, ...)                                                         \
  do {                                                                         \
    char _dlog_buf[128];                                                       \
    snprintf(_dlog_buf, sizeof(_dlog_buf), (fmt), ##__VA_ARGS__);              \
    Serial.println(_dlog_buf);                                                 \
    oled.logEvent(_dlog_buf);                                                  \
  } while (0)
#else
#define DLOG(fmt, ...)                                                         \
  do {                                                                         \
    Serial.printf(fmt "\n", ##__VA_ARGS__);                                    \
  } while (0)
#endif
#else
// For relay/mobile, disable DLOG entirely to save flash
#define DLOG(fmt, ...)                                                         \
  do {                                                                         \
  } while (0)
#endif
