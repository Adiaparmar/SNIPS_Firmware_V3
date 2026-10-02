// ============================================================
//  SNIPS V3 — Shared Configuration
//  All packet types, addresses, timing constants, packet structs,
//  and pin definitions for all three node roles.
//
//  NOTE: Serial_printf has been removed entirely. All debug
//  output now goes through OledHAL::log() in the node files.
// ============================================================
#pragma once
#include <Arduino.h>
#include <math.h>
#include <stdint.h>

// ============================================================
//  PLATFORM GUARD — ISR attribute
//  ESP32: IRAM_ATTR places ISR in internal RAM (required)
//  AVR (Uno R3 / Nano Every): no equivalent attribute needed
// ============================================================
#ifdef ESP32
#define SNIPS_ISR_ATTR IRAM_ATTR
#else
#define SNIPS_ISR_ATTR
#endif

// ============================================================
//  PIN DEFINITIONS
//  Anchor = XIAO ESP32-S3 + Wio-SX1262
//  Relay  = Arduino Uno R3 + Wio-SX1262
//  Mobile = Arduino Nano Every + Wio-SX1262
// ============================================================
#ifdef NODE_TYPE_ANCHOR
// --- XIAO ESP32-S3 (verified from pinMap.h) ---
#define LORA_MISO 8
#define LORA_MOSI 9
#define LORA_SCK 7
#define LORA_NSS 41
#define LORA_BUSY 40
#define LORA_RST 42
#define LORA_DIO1 39
#define LORA_RF_SW1 38 // RF switch: HIGH=TX, LOW=RX
// OLED on XIAO default I2C pins
#define OLED_SDA_PIN 5
#define OLED_SCL_PIN 6
#elif defined(NODE_TYPE_RELAY)
// --- Arduino Uno R3 ---
#define LORA_NSS 10  // D10 — SPI CS
#define LORA_MOSI 11 // D11
#define LORA_MISO 12 // D12
#define LORA_SCK 13  // D13
#define LORA_RST 5   // D5
#define LORA_BUSY 6  // D6
#define LORA_DIO1 3  // D3 — external interrupt capable on ATmega328P
#elif defined(NODE_TYPE_MOBILE)
// --- Arduino Nano Every ---
#define LORA_NSS 10  // D10 — SPI CS
#define LORA_MOSI 11 // D11
#define LORA_MISO 12 // D12
#define LORA_SCK 13  // D13
#define LORA_RST 5   // D5
#define LORA_BUSY 6  // D6
#define LORA_DIO1 3  // D3 — interrupt capable
// OLED on Nano Every uses the shared UNO-style I2C pins
#define OLED_SDA_PIN A4
#define OLED_SCL_PIN A5
// Button — mobiles only, polled
#define BTN_PIN 7 // D7
#else
#error "Set NODE_TYPE_ANCHOR, NODE_TYPE_RELAY, or NODE_TYPE_MOBILE"
#endif

// ============================================================
//  LoRa RF PARAMETERS  (same for all nodes)
// ============================================================
#define LORA_FREQ 865.0f // MHz — ISM band (India)
#define LORA_BW 125.0f   // kHz
#define LORA_SF 7        // Spreading Factor
#define LORA_CR 5        // Coding Rate 4/5
#define LORA_POWER 14    // dBm TX power
#define LORA_PREAMBLE 8  // symbols

// ============================================================
//  NODE ADDRESSING
// ============================================================
#define ADDR_BROADCAST 0xFF
#define ADDR_ANCHOR 0x01
#define ADDR_RELAY_1 0x10
#define ADDR_RELAY_2 0x11
#define ADDR_MOBILE_1 0x20
#define ADDR_MOBILE_2 0x21

// ============================================================
//  PACKET TYPES
// ============================================================
#define PKT_BEACON 0x01     // R1 → broadcast : sync + timing
#define PKT_DATA 0x02       // Any → any      : payload
#define PKT_ACK 0x03        // Receiver → hop_src
#define PKT_POS_REQ 0x04    // Mobile → broadcast : "measure me"
#define PKT_POS_MEAS 0x05   // Anchor → R1    : RSSI measurement
#define PKT_POS_RESULT 0x06 // R1 → Mobile    : computed position
#define PKT_RREQ 0x07       // Any → broadcast : route request
#define PKT_RREP 0x08       // Dest → origin  : route reply
#define PKT_RERR 0x09       // R1 → upstream  : route error

// ============================================================
//  TDMA SUPERFRAME TIMING  (1-second superframe per spec §3)
// ============================================================
#define SF_DURATION_MS 1000 // 1-second superframe
#define SF_SYNC_END_MS 100  // 0–100 ms   : BEACON window
#define SF_CTRL_END_MS 300  // 100–300 ms : RREQ, POS_REQ
#define SF_DATA_END_MS 900  // 300–900 ms : DATA, POS_MEAS, RREP
// Emergency phase: 900–1000 ms

// Data slot structure (12 slots × 50 ms, spec §3)
#define SLOT_DURATION_MS 50
#define MAX_SLOTS 12

extern const uint8_t DEFAULT_SLOT_OWNERS[MAX_SLOTS];

// ============================================================
//  PROTOCOL CONSTANTS
// ============================================================
#define ACK_TIMEOUT_MS                                                         \
  80                                // Wait for hop-level ACK
                                    // (SF7/BW125 packet airtime is a few
                                    // tens of ms; 300ms was oversized and
                                    // could let a single retry chain
                                    // overrun the 600ms TDMA data phase,
                                    // starving RX polling and dropping
                                    // unrelated in-flight packets/ACKs)
#define MAX_RETRIES 3               // TX retries before RERR
#define ROUTE_EXPIRE_FRAMES 10      // Frames before route stale
#define NEIGHBOUR_EXPIRE_FRAMES 5   // Frames before neighbour stale
#define HEARTBEAT_INTERVAL_MS 15000 // Anchor heartbeat period

// ============================================================
//  POSITIONING CONSTANTS
// ============================================================
#define ORDERING_THRESHOLD_US 2048 // 2 × symbol duration at SF7/BW125
#define RSSI_D0 -40.0f             // dBm at 1 m reference (SX1262)
#define PATH_LOSS_N 2.5f           // Path-loss exponent (indoor/suburban)

// ============================================================
//  ANCHOR POSITION MAP  (relay holds this; anchors do NOT transmit coords)
//  Only 1 anchor in this deployment.
// ============================================================
#ifndef ANCHOR_X
#define ANCHOR_X 0.0f
#endif
#ifndef ANCHOR_Y
#define ANCHOR_Y 0.0f
#endif

#ifndef RELAY_1_X
#define RELAY_1_X 0.0f
#endif
#ifndef RELAY_1_Y
#define RELAY_1_Y 0.0f
#endif
#ifndef RELAY_2_X
#define RELAY_2_X 0.0f
#endif
#ifndef RELAY_2_Y
#define RELAY_2_Y 0.0f
#endif

#ifdef NODE_TYPE_RELAY
struct AnchorInfo {
  uint8_t id;
  int16_t x_cm;
  int16_t y_cm;
};
extern const AnchorInfo ANCHOR_MAP[];
#define ANCHOR_MAP_SIZE 1

struct RelayInfo {
  uint8_t id;
  int16_t x_cm;
  int16_t y_cm;
};
extern const RelayInfo RELAY_MAP[];
#define RELAY_MAP_SIZE 2
#endif // NODE_TYPE_RELAY

// ============================================================
//  OLED
// ============================================================
#define OLED_ADDR 0x3C
#define OLED_WIDTH 128
#define OLED_HEIGHT 64

// ============================================================
//  BUTTON (mobiles only)
// ============================================================
#define BTN_DEBOUNCE_MS 50
#define BTN_LONG_PRESS_MS 2000         // >2s = trigger POS_REQ
#define BTN_DOUBLE_PRESS_WINDOW_MS 400 // max ms between presses for double-tap

// ============================================================
//  PREDEFINED MESSAGES (mobile outgoing)
// ============================================================
extern const char *const MSG_LIST[];
#define MSG_COUNT 4

// ============================================================
//  PACKET STRUCTURES  (all __attribute__((packed)) for wire format)
// ============================================================

// Common 6-byte header — prefix of EVERY packet
struct __attribute__((packed)) SnipsHeader {
  uint8_t pkt_type; // PKT_* constant
  uint8_t src;      // Original sender (never changes in transit)
  uint8_t dst;      // Final destination (0xFF = broadcast)
  uint8_t seq;      // Rolling 8-bit counter per node
  uint8_t ttl;      // Hops remaining; decremented at each relay; drop at 0
  uint8_t hop_src;  // Immediate previous sender (ACKs go here, not to src)
};

// PKT_BEACON (31 B) — relay → broadcast
struct __attribute__((packed)) PktBeacon {
  SnipsHeader hdr;
  uint32_t frame_num;     // Superframe counter
  uint32_t frame_ts_ms;   // Relay millis() at beacon send
  uint8_t active_nodes;   // Bitmask of nodes seen last frame
  uint8_t slot_owner[16]; // slot index → node address
};

// PKT_DATA (54 B) — any → any
struct __attribute__((packed)) PktData {
  SnipsHeader hdr;
  char payload[48];
};

// PKT_ACK (9 B) — receiver → hop_src
struct __attribute__((packed)) PktAck {
  SnipsHeader hdr;
  uint8_t ack_seq; // Sequence number being acknowledged
  uint8_t status;  // 0=OK, 1=CRC error
  int8_t rssi_dbm; // RSSI of received packet
};

// PKT_POS_REQ (7 B) — mobile → broadcast
struct __attribute__((packed)) PktPosReq {
  SnipsHeader hdr;
  uint8_t req_id; // Unique request identifier
};

// PKT_POS_MEAS (13 B) — anchor → relay
// NOTE: anchor_x/y removed per spec §5; relay looks up ANCHOR_MAP[]
struct __attribute__((packed)) PktPosMeas {
  SnipsHeader hdr;
  uint8_t req_id;      // Matches PktPosReq.req_id
  uint8_t mobile_addr; // Which mobile was measured
  int8_t rssi_dbm;     // RSSI of POS_REQ at this anchor
  uint32_t rx_time_us; // micros() at DIO1 ISR (for ordering)
};

// PKT_POS_RESULT (19 B) — relay → mobile (requester + peer notify)
struct __attribute__((packed)) PktPosResult {
  SnipsHeader hdr;
  uint8_t req_id;
  uint8_t measured_addr; // Address of the mobile that was measured
  int16_t est_x_cm;      // Estimated X (centimetres from anchor origin)
  int16_t est_y_cm;      // Estimated Y
  uint8_t confidence;    // 0–100%
  uint8_t anchor_count;  // How many anchors contributed
  uint8_t refined;       // 1 if timing disambiguation applied
};

// PKT_RREQ (11 B) — any → broadcast : route request
struct __attribute__((packed)) PktRREQ {
  SnipsHeader hdr;
  uint8_t dest_addr; // Who we're looking for
  uint8_t orig_addr; // Who initiated the discovery
  uint8_t rreq_id;   // Unique per orig_addr
  uint8_t hop_count; // Incremented at each re-broadcast
  int8_t min_rssi;   // Weakest link RSSI on path so far
};

// PKT_RREP (11 B) — destination → origin
struct __attribute__((packed)) PktRREP {
  SnipsHeader hdr;
  uint8_t dest_addr; // Node that was found (RREP generator)
  uint8_t orig_addr; // Node that started the RREQ
  uint8_t rreq_id;   // Matches the RREQ
  uint8_t hop_count; // Hops from dest to orig on this path
  int8_t min_rssi;   // Weakest link on return path
};

// PKT_RERR (8 B) — relay → upstream sender
struct __attribute__((packed)) PktRERR {
  SnipsHeader hdr;
  uint8_t broken_dest; // Destination that became unreachable
  uint8_t error_code;  // 0=no_ack, 1=ttl_expired, 2=queue_full
};

// ============================================================
//  RSSI → DISTANCE  (log-distance path-loss model, spec §14)
//  d = 10 ^ ((RSSI_D0 - rssi) / (10 * N))
// ============================================================
inline float rssiToDistance(int8_t rssi_dbm) {
  return powf(10.0f, (RSSI_D0 - (float)rssi_dbm) / (10.0f * PATH_LOSS_N));
}

// ============================================================
//  Signal bars helper (for OLED)
// ============================================================
inline uint8_t rssiBars(int8_t rssi_dbm) {
  if (rssi_dbm > -60)
    return 4;
  if (rssi_dbm > -75)
    return 3;
  if (rssi_dbm > -90)
    return 2;
  return 1;
}
