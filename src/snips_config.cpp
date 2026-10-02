#include "snips_config.h"

// Slot owner table for 1-anchor / 2-relay / 2-mobile network
// Slots 0–1: A1 (anchor sync), Slots 2–3: R1, Slots 4–5: R2,
// Slots 6–7: M1, Slots 8–9: M2, Slots 10–11: free
const uint8_t DEFAULT_SLOT_OWNERS[MAX_SLOTS] = {
    ADDR_ANCHOR,   // slot 0
    ADDR_ANCHOR,   // slot 1
    ADDR_RELAY_1,  // slot 2
    ADDR_RELAY_2,  // slot 3
    ADDR_MOBILE_1, // slot 4
    ADDR_MOBILE_2, // slot 5
    0x00,          // slot 6  free
    0x00,          // slot 7  free
    0x00,          // slot 8  free
    0x00,          // slot 9  free
    0x00,          // slot 10 free
    0x00,          // slot 11 free
};

// These coordinate tables are only needed on the relay that runs
// the positioning algorithm.  Guarding them here prevents the
// SSD1306 / avr-libc linker from pulling in the floats on anchor
// and mobile builds where they are never referenced.
#ifdef NODE_TYPE_RELAY
const AnchorInfo ANCHOR_MAP[] = {
    {ADDR_ANCHOR, (int16_t)(ANCHOR_X * 100), (int16_t)(ANCHOR_Y * 100)}
};

const RelayInfo RELAY_MAP[] = {
    {ADDR_RELAY_1, (int16_t)(RELAY_1_X * 100), (int16_t)(RELAY_1_Y * 100)},
    {ADDR_RELAY_2, (int16_t)(RELAY_2_X * 100), (int16_t)(RELAY_2_Y * 100)},
};
#endif // NODE_TYPE_RELAY

const char *const MSG_LIST[] = {
    "ALL CLEAR",
    "NEED HELP",
    "MOVING NOW",
    "STANDBY",
};
