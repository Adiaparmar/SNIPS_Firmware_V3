// ============================================================
//  SNIPS V3 — main.cpp
//
//  Compile-time role selection via build flag in platformio.ini:
//    NODE_TYPE_ANCHOR   →  pio run -e anchor   --target upload
//    NODE_TYPE_RELAY    →  pio run -e relay    --target upload
//    NODE_TYPE_MOBILE   →  pio run -e mobile1  --target upload
//                          pio run -e mobile2  --target upload
//
//  NODE_ID is also set by build flag (-DNODE_ID=0xXX).
//  All logic lives in the respective node header files.
//  This file only routes setup()/loop() to the correct module.
// ============================================================
#include <Arduino.h>
#include "snips_config.h"
#include "SX1262HAL.h"
#ifdef HAS_OLED
#include "OledHAL.h"
#endif

// Global radio and OLED instances
SX1262HAL radio(LORA_NSS, LORA_BUSY, LORA_RST, LORA_DIO1);
#ifdef HAS_OLED
OledHAL oled;
#endif

// ISR flags
volatile bool g_rxFlag = false;
volatile uint32_t g_rxTimeUs = 0;

void SNIPS_ISR_ATTR onDIO1() {
    g_rxTimeUs = micros();
    g_rxFlag = true;
}

#if defined(NODE_TYPE_ANCHOR)
#include "node_anchor.h"

#elif defined(NODE_TYPE_RELAY)
#include "node_relay.h"

#elif defined(NODE_TYPE_MOBILE)
#include "node_mobile.h"

#else
#error \
    "No node type defined! Set -DNODE_TYPE_ANCHOR / _RELAY / _MOBILE in platformio.ini"
#endif

// ============================================================
void setup() {
    Serial.begin(115200);
    // Let Serial initialize
    delay(100);

#if defined(NODE_TYPE_ANCHOR)
  anchor_setup();
#elif defined(NODE_TYPE_RELAY)
  relay_setup();
#elif defined(NODE_TYPE_MOBILE)
  mobile_setup();
#endif
}

void loop() {
#if defined(NODE_TYPE_ANCHOR)
  anchor_loop();
#elif defined(NODE_TYPE_RELAY)
  relay_loop();
#elif defined(NODE_TYPE_MOBILE)
  mobile_loop();
#endif
}
