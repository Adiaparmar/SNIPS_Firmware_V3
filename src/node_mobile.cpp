#include "node_mobile.h"
#ifdef NODE_TYPE_MOBILE

#include "SX1262HAL.h"
#include "OledHAL.h"
#include "mesh_router.h"
#include "snips_config.h"

extern SX1262HAL radio;
#ifdef HAS_OLED
extern OledHAL oled;
#endif
extern volatile bool g_rxFlag;
extern void onDIO1();

// ---- Mobile state -------------------------------------------
static uint32_t m_frame_start = 0;
static uint32_t m_frame_num = 0;
static bool m_synced = false;

static bool m_btn_state = HIGH;
static uint32_t m_btn_last_ms = 0;
static uint32_t m_btn_down_ms = 0;
static uint32_t m_btn_up_ms = 0;
static uint8_t m_tap_count = 0;

static uint8_t m_msg_idx = 0;
static bool m_pos_wait = false;
static uint32_t m_pos_req_ms = 0;
static uint8_t m_last_req_id = 0;

static int16_t m_my_x = 0, m_my_y = 0;
static uint8_t m_my_conf = 0;

static int16_t m_peer_x = 0, m_peer_y = 0;
static uint8_t m_peer_conf = 0;
static uint32_t m_peer_last_rx = 0;

uint8_t mesh_nextSeq() {
  static uint8_t seq = 0;
  return ++seq;
}
uint32_t mesh_currentFrame() { return m_frame_num; }

static inline uint8_t m_nextSeq() { return mesh_nextSeq(); }

static void m_sendPosReq() {
  if (!m_synced) return;
  PktPosReq r{};
  r.hdr.pkt_type = PKT_POS_REQ;
  r.hdr.src = NODE_ID;
  r.hdr.dst = ADDR_BROADCAST;
  r.hdr.seq = m_nextSeq();
  r.hdr.ttl = 1;
  r.hdr.hop_src = NODE_ID;
  r.req_id = ++m_last_req_id;

  radio_send((uint8_t *)&r, sizeof(r));
  m_pos_wait = true;
  m_pos_req_ms = millis();
  DLOG("POS_REQ > bdcast");
#ifdef HAS_OLED
  oled.logEvent("Req POS...");
#endif
}

static void m_sendData(const char *msg, uint8_t dest) {
  if (!m_synced) return;
  PktData d{};
  d.hdr.pkt_type = PKT_DATA;
  d.hdr.src = NODE_ID;
  d.hdr.dst = dest;
  d.hdr.seq = m_nextSeq();
  d.hdr.ttl = 4;
  d.hdr.hop_src = NODE_ID;
  strncpy(d.payload, msg, sizeof(d.payload) - 1);

  bool ok = mesh_forwardData(d);
  if (!ok) {
    g_rt.enqueue(d, m_frame_num);
  }
}

static void m_checkButton() {
  bool raw = digitalRead(BTN_PIN);
  uint32_t now = millis();

  if (m_tap_count > 0 && raw == HIGH && (now - m_btn_up_ms > BTN_DOUBLE_PRESS_WINDOW_MS)) {
    if (m_tap_count == 1) {
      uint8_t target = (NODE_ID == ADDR_MOBILE_1) ? ADDR_MOBILE_2 : ADDR_MOBILE_1;
      m_sendData(MSG_LIST[m_msg_idx], target);
#ifdef HAS_OLED
      oled.logEventString("Sent:", MSG_LIST[m_msg_idx]);
#endif
    } else if (m_tap_count == 2) {
      m_msg_idx = (m_msg_idx + 1) % MSG_COUNT;
#ifdef HAS_OLED
      oled.logEventString("Sel:", MSG_LIST[m_msg_idx]);
#endif
    }
    m_tap_count = 0;
  }

  if (raw != m_btn_state && (now - m_btn_last_ms > BTN_DEBOUNCE_MS)) {
    m_btn_state = raw;
    m_btn_last_ms = now;

    if (raw == LOW) {
      m_btn_down_ms = now;
    } else {
      uint32_t hold_time = now - m_btn_down_ms;
      m_btn_up_ms = now;
      if (hold_time > BTN_LONG_PRESS_MS) {
        m_sendPosReq();
        m_tap_count = 0;
      } else {
        m_tap_count++;
      }
    }
  }
}

static void m_handleRx() {
  static uint8_t buf[64];
  g_rxFlag = false;

  size_t len = radio.readData(buf, sizeof(buf));
  int8_t rssi = radio.getRSSI();

  if (len < sizeof(SnipsHeader)) return;
  auto &hdr = *(SnipsHeader *)buf;
  if (hdr.src == NODE_ID) return;

  g_rt.updateNeighbour(hdr.hop_src, rssi, m_frame_num);

  switch (hdr.pkt_type) {
  case PKT_BEACON: {
    auto &b = *(PktBeacon *)buf;
    m_frame_start = millis();
    m_frame_num = b.frame_num;
    if (!m_synced) {
      m_synced = true;
      DLOG("SYNCED f=%lu", (unsigned long)m_frame_num);
#ifdef HAS_OLED
      oled.logEvent("SYNCED");
#endif
    }
    g_rt.addRoute(hdr.src, hdr.hop_src, 1, rssi, m_frame_num);
    break;
  }
  case PKT_POS_RESULT: {
    auto &r = *(PktPosResult *)buf;
    mesh_sendAck(hdr, rssi);

    if (r.measured_addr == NODE_ID) {
      m_pos_wait = false;
      m_my_x = r.est_x_cm;
      m_my_y = r.est_y_cm;
      m_my_conf = r.confidence;
      DLOG("POS FIX (%d,%d) %d%%", (int)m_my_x, (int)m_my_y, m_my_conf);
#ifdef HAS_OLED
      oled.logEventPos(m_my_x, m_my_y, m_my_conf);
#endif
    } else {
      m_peer_x = r.est_x_cm;
      m_peer_y = r.est_y_cm;
      m_peer_conf = r.confidence;
      m_peer_last_rx = millis();
#ifdef HAS_OLED
      oled.logEventHex("Peer", r.measured_addr);
      oled.logEventPos(m_peer_x, m_peer_y, m_peer_conf);
#endif
    }
    break;
  }
  case PKT_DATA: {
    auto &d = *(PktData *)buf;
    if (hdr.dst == NODE_ID || hdr.dst == ADDR_BROADCAST) {
      mesh_sendAck(hdr, rssi);
#ifdef HAS_OLED
      oled.logEventString("Msg:", d.payload);
#endif
    } else {
      mesh_sendAck(hdr, rssi);
      mesh_forwardData(d);
    }
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
  case PKT_ACK:
    break;
  }
}

void mobile_setup() {
  delay(100);
  pinMode(BTN_PIN, INPUT_PULLUP);

#ifdef HAS_OLED
  oled.begin(OLED_ADDR);
  oled.bootLine("=== SNIPS MOBILE ===");
  oled.bootLineHex("ID: ", NODE_ID);
#endif

  int16_t err = radio.begin(LORA_FREQ, LORA_BW, LORA_SF, LORA_CR, 0x12, LORA_POWER, LORA_PREAMBLE);
  if (err != 0) {
#ifdef HAS_OLED
    oled.bootLine("LoRa FAIL");
#endif
    while (true) delay(1000);
  }

  radio.setPacketReceivedAction(onDIO1);
  radio.startReceive();
  g_rt.init();

#ifdef HAS_OLED
  oled.bootLineFreqSF("865.0", LORA_SF);
  delay(1000);
  oled.beginScrollLog();
  oled.logEvent("Wait for BEACON...");
#endif
}

void mobile_loop() {
  uint32_t now = millis();
  m_checkButton();

  if (m_pos_wait && (now - m_pos_req_ms > 2000)) {
    m_pos_wait = false;
    DLOG("POS req timeout");
#ifdef HAS_OLED
    oled.logEvent("POS timeout");
#endif
  }

  if (m_peer_conf > 0 && (now - m_peer_last_rx > 15000)) {
    m_peer_conf = 0;
  }

  if (g_rxFlag) {
    m_handleRx();
  }

  if (m_synced) {
    uint32_t elapsed = now - m_frame_start;
    if (elapsed >= SF_CTRL_END_MS && elapsed < SF_DATA_END_MS) {
      mesh_flushSFQueue(m_frame_num);
    }
  }
}
#endif // NODE_TYPE_MOBILE
