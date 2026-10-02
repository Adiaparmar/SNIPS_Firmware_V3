#include "node_anchor.h"
#ifdef NODE_TYPE_ANCHOR

#include "SX1262HAL.h"
#include "OledHAL.h"
#include "mesh_router.h"
#include "snips_config.h"

extern SX1262HAL radio;
#ifdef HAS_OLED
extern OledHAL oled;
#endif
extern volatile bool g_rxFlag;
extern volatile uint32_t g_rxTimeUs;
extern void onDIO1();

// ---- Anchor state --------------------------------------------
static uint32_t a_frame_start = 0;
static uint32_t a_frame_num = 0;
static uint8_t a_seq = 0;
static bool a_synced = false;
static uint32_t a_last_hb_ms = 0;
static int8_t a_last_rssi = -120;

// Pending POS_MEAS to send
static bool a_pos_pending = false;
static uint8_t a_pos_req_id = 0;
static uint8_t a_pos_mobile = 0;
static int8_t a_pos_rssi = 0;
static uint32_t a_pos_rx_time = 0;

uint8_t mesh_nextSeq() {
  static uint8_t seq = 0;
  return ++seq;
}
uint32_t mesh_currentFrame() { return a_frame_num; }

static inline uint8_t a_nextSeq() { return mesh_nextSeq(); }

static void a_sendMeasurement() {
  PktPosMeas m{};
  m.hdr.pkt_type = PKT_POS_MEAS;
  m.hdr.src = NODE_ID;
  m.hdr.dst = ADDR_BROADCAST;
  m.hdr.seq = a_nextSeq();
  m.hdr.ttl = 1;
  m.hdr.hop_src = NODE_ID;
  m.req_id = a_pos_req_id;
  m.mobile_addr = a_pos_mobile;
  m.rssi_dbm = a_pos_rssi;
  m.rx_time_us = a_pos_rx_time;

  delay(30);
  bool ok = radio_send((uint8_t *)&m, sizeof(m));

  Serial.printf(
      "[ANCHOR] POS_MEAS → Relay  mobile=0x%02X  RSSI=%d  t=%lu  %s\n",
      a_pos_mobile, a_pos_rssi, (unsigned long)a_pos_rx_time,
      ok ? "OK" : "FAIL");
#ifdef HAS_OLED
  char buf[32];
  snprintf(buf, sizeof(buf), "MEAS>R1 M=0x%02X %s", (int)a_pos_mobile, ok ? "OK" : "FAIL");
  oled.logEvent(buf);
#endif

  a_pos_pending = false;
}

static void a_sendHeartbeat() {
  PktData d{};
  d.hdr.pkt_type = PKT_DATA;
  d.hdr.src = NODE_ID;
  d.hdr.dst = ADDR_RELAY_1;
  d.hdr.seq = a_nextSeq();
  d.hdr.ttl = 2;
  d.hdr.hop_src = NODE_ID;
  snprintf(d.payload, sizeof(d.payload), "ANCHOR HB f=%lu", (unsigned long)a_frame_num);
  radio_send((uint8_t *)&d, sizeof(d));
  Serial.printf("[ANCHOR] Heartbeat  frame=%lu\n", (unsigned long)a_frame_num);
#ifdef HAS_OLED
  char buf[32];
  snprintf(buf, sizeof(buf), "HB f=%lu", (unsigned long)a_frame_num);
  oled.logEvent(buf);
#endif
}

static bool a_waitAck(uint8_t expected_seq) {
  uint32_t deadline = millis() + ACK_TIMEOUT_MS;
  while (millis() < deadline) {
    if (g_rxFlag) {
      g_rxFlag = false;
      uint8_t buf[sizeof(PktAck)];
      size_t len = radio.readData(buf, sizeof(buf));
      if (len >= sizeof(PktAck)) {
        auto &a = *(PktAck *)buf;
        if (a.hdr.pkt_type == PKT_ACK && a.hdr.dst == NODE_ID &&
            a.ack_seq == expected_seq) {
          return true;
        }
      }
    }
    delay(5);
  }
  return false;
}

static void a_handleRx() {
  static uint8_t buf[64];
  
  uint32_t dio1_time = g_rxTimeUs;
  g_rxFlag = false;

  size_t len = radio.readData(buf, sizeof(buf));
  int8_t rssi = radio.getRSSI();

  if (len < sizeof(SnipsHeader))
    return;

  auto &hdr = *(SnipsHeader *)buf;
  if (hdr.src == NODE_ID)
    return;

  a_last_rssi = rssi;
  g_rt.updateNeighbour(hdr.hop_src, rssi, a_frame_num);

  switch (hdr.pkt_type) {
  case PKT_BEACON: {
    auto &b = *(PktBeacon *)buf;
    a_frame_start = millis();
    a_frame_num = b.frame_num;
    a_synced = true;

    Serial.printf("[ANCHOR] BEACON frame=%lu  RSSI=%d\n", (unsigned long)a_frame_num, rssi);
#ifdef HAS_OLED
    char lbuf[32];
    snprintf(lbuf, sizeof(lbuf), "BEACON f=%lu R=%d", (unsigned long)a_frame_num, (int)rssi);
    oled.logEvent(lbuf);
#endif
    break;
  }
  case PKT_POS_REQ: {
    auto &req = *(PktPosReq *)buf;
    Serial.printf("[ANCHOR] POS_REQ from 0x%02X  req_id=%d  RSSI=%d  t=%lu\n",
                  hdr.src, req.req_id, rssi, (unsigned long)dio1_time);
    mesh_sendAck(hdr, rssi);
    a_pos_pending = true;
    a_pos_req_id = req.req_id;
    a_pos_mobile = hdr.src;
    a_pos_rssi = rssi;
    a_pos_rx_time = dio1_time;
#ifdef HAS_OLED
    char lbuf[32];
    snprintf(lbuf, sizeof(lbuf), "POS_REQ 0x%02X R=%d", (int)hdr.src, (int)rssi);
    oled.logEvent(lbuf);
#endif
    break;
  }
  case PKT_RREQ:
    mesh_handleRREQ(*(PktRREQ *)buf, hdr, rssi);
    break;
  case PKT_RREP:
    mesh_handleRREP(*(PktRREP *)buf, hdr, rssi);
    break;
  case PKT_RERR:
    mesh_handleRERR(*(PktRERR *)buf, hdr);
    break;
  case PKT_ACK: {
    auto &a = *(PktAck *)buf;
    Serial.printf("[ANCHOR] ACK from 0x%02X  seq=%d\n", hdr.src, a.ack_seq);
#ifdef HAS_OLED
    char lbuf[32];
    snprintf(lbuf, sizeof(lbuf), "ACK 0x%02X sq=%d", (int)hdr.src, (int)a.ack_seq);
    oled.logEvent(lbuf);
#endif
    break;
  }
  case PKT_DATA: {
    auto &d = *(PktData *)buf;
    if (hdr.dst == NODE_ID || hdr.dst == ADDR_BROADCAST) {
      Serial.printf("[ANCHOR] DATA from 0x%02X : \"%s\"\n", hdr.src, d.payload);
#ifdef HAS_OLED
      char lbuf[32];
      snprintf(lbuf, sizeof(lbuf), "FROM 0x%02X: %s", (int)hdr.src, d.payload);
      oled.logEvent(lbuf);
#endif
      mesh_sendAck(hdr, rssi);
    } else {
      mesh_sendAck(hdr, rssi);
      mesh_forwardData(d);
    }
    break;
  }
  }

  if (a_pos_pending) {
    a_sendMeasurement();
  }
}

void anchor_setup() {
  Serial.begin(115200);
  delay(100);

#ifdef HAS_OLED
  oled.begin(OLED_ADDR);
  oled.bootLine("=== SNIPS ANCHOR ===");
  oled.bootLineHex("ID: ", NODE_ID);
  oled.bootLinePos("Pos:", (int16_t)(ANCHOR_X*100), (int16_t)(ANCHOR_Y*100));
#endif

  Serial.println("[ANCHOR] LoRa init...");
  
#ifdef NODE_TYPE_ANCHOR
  // ESP32 anchor uses RF_SW1 for TX/RX switching
  pinMode(LORA_RF_SW1, OUTPUT);
  digitalWrite(LORA_RF_SW1, HIGH); // Default to RX? No wait, DIO2 does it.
  // Wait, if it has a manual RF switch not connected to DIO2, we need to handle it.
  // Original lora_hal.cpp had a hack for ESP32.
  // Actually, we added _setDio2AsRfSwitchCtrl to HAL. Let's see if we need manual RF_SW1 here.
  // "User confirms Seeed Wio-SX1262 module uses DIO2 for RF switch control."
  // So we don't need manual toggling of LORA_RF_SW1.
#endif

  int16_t err = radio.begin(LORA_FREQ, LORA_BW, LORA_SF, LORA_CR, 0x12, LORA_POWER, LORA_PREAMBLE);
  if (err != 0) {
    Serial.printf("[ANCHOR] LoRa init FAILED %d\n", err);
#ifdef HAS_OLED
    oled.bootLine("LoRa FAIL");
#endif
    while (true) delay(1000);
  }

  radio.setPacketReceivedAction(onDIO1);
  radio.startReceive();

#ifdef HAS_OLED
  oled.bootLineFreqSF("865.0", LORA_SF);
  delay(1000);
  oled.beginScrollLog();
  oled.logEvent("Waiting for BEACON...");
#endif

  g_rt.init();
  g_rt.addRoute(ADDR_RELAY_1, ADDR_RELAY_1, 1, -50, 0);

  a_frame_start = millis();
  a_last_hb_ms = millis();
  Serial.println("[ANCHOR] Ready");
}

void anchor_loop() {
  if (g_rxFlag) {
    a_handleRx();
  }

  if (a_synced) {
    uint32_t elapsed = millis() - a_frame_start;
    if (elapsed >= SF_CTRL_END_MS && elapsed < SF_DATA_END_MS) {
      mesh_flushSFQueue(a_frame_num);
    }
  }

  if (millis() - a_last_hb_ms > HEARTBEAT_INTERVAL_MS) {
    a_last_hb_ms = millis();
    a_sendHeartbeat();
    bool acked = a_waitAck(a_seq);
    if (!acked) {
      Serial.println("[ANCHOR] Heartbeat: no ACK");
#ifdef HAS_OLED
      oled.logEvent("HB: no ACK");
#endif
    }
  }
}

#endif // NODE_TYPE_ANCHOR
