#include "node_relay.h"
#ifdef NODE_TYPE_RELAY

#include "SX1262HAL.h"
#include "mesh_router.h"
#include "snips_config.h"
#include <math.h>

extern SX1262HAL radio;
extern volatile bool g_rxFlag;
extern volatile uint32_t g_rxTimeUs;
extern void onDIO1();

// ---- Relay state -------------------------------------------
static uint32_t g_frame_num = 0;
static uint32_t g_frame_start = 0;

uint8_t mesh_nextSeq() {
  static uint8_t seq = 0;
  return ++seq;
}
uint32_t mesh_currentFrame() { return g_frame_num; }

static bool    g_pos_pending       = false;
static uint8_t g_pos_req_id        = 0;
static uint8_t g_pos_mobile_addr   = 0;
static int8_t   g_relay_rssi       = -120;
static uint32_t g_relay_rx_time_us = 0;
static bool     g_relay_meas_valid = false;
static int8_t   g_anchor_rssi       = -120;
static uint32_t g_anchor_rx_time_us = 0;
static bool     g_anchor_meas_valid = false;
static bool    g_rerr_pending  = false;
static uint8_t g_rerr_dest     = 0;
static uint8_t g_rerr_upstream = 0;

static inline uint8_t r_nextSeq() { return mesh_nextSeq(); }

static inline float distSq(float ax, float ay, float bx, float by) {
  float dx = ax - bx, dy = ay - by;
  return dx * dx + dy * dy;
}

static void r_computeAndSendPosition() {
  if (!g_relay_meas_valid && !g_anchor_meas_valid) return;

  float   est_x      = 0, est_y = 0;
  uint8_t confidence = 0;
  uint8_t refined    = 0;

  float ax = ANCHOR_MAP[0].x_cm / 100.0f;
  float ay = ANCHOR_MAP[0].y_cm / 100.0f;

  float rx = (NODE_ID == ADDR_RELAY_1) ? (float)RELAY_1_X : (float)RELAY_2_X;
  float ry = (NODE_ID == ADDR_RELAY_1) ? (float)RELAY_1_Y : (float)RELAY_2_Y;

  if (g_anchor_meas_valid && g_relay_meas_valid) {
    float d_a = rssiToDistance(g_anchor_rssi);
    float d_r = rssiToDistance(g_relay_rssi);
    float dx_ar = rx - ax, dy_ar = ry - ay;
    float D = sqrtf(dx_ar * dx_ar + dy_ar * dy_ar);
    bool degenerate = (D < 0.1f) || (fabsf(d_a - d_r) > D) || (d_a + d_r < D);

    if (degenerate) {
      float w_a = 1.0f / (d_a * d_a + 0.01f);
      float w_r = 1.0f / (d_r * d_r + 0.01f);
      est_x      = (w_a * ax + w_r * rx) / (w_a + w_r);
      est_y      = (w_a * ay + w_r * ry) / (w_a + w_r);
      confidence = 20;
    } else {
      float a_val = (d_a * d_a - d_r * d_r + D * D) / (2.0f * D);
      float h_sq  = d_a * d_a - a_val * a_val;
      float h     = (h_sq > 0.0f) ? sqrtf(h_sq) : 0.0f;
      float ux = dx_ar / D, uy = dy_ar / D;
      float px = -uy,       py = ux;
      float p1x = ax + a_val * ux + h * px;
      float p1y = ay + a_val * uy + h * py;
      float p2x = ax + a_val * ux - h * px;
      float p2y = ay + a_val * uy - h * py;

      int32_t dt = (int32_t)(g_anchor_rx_time_us - g_relay_rx_time_us);
      bool anchor_first = (dt < 0);

      if ((uint32_t)abs(dt) >= ORDERING_THRESHOLD_US) {
        bool p1_anchor_closer = distSq(p1x, p1y, ax, ay) < distSq(p1x, p1y, rx, ry);
        if ((anchor_first && p1_anchor_closer) || (!anchor_first && !p1_anchor_closer)) {
          est_x = p1x; est_y = p1y;
        } else {
          est_x = p2x; est_y = p2y;
        }
        confidence = 60; refined = 1;
      } else {
        est_x = (p1x + p2x) * 0.5f;
        est_y = (p1y + p2y) * 0.5f;
        confidence = 45;
      }
    }
  } else if (g_anchor_meas_valid) {
    float d_a = rssiToDistance(g_anchor_rssi);
    est_x = ax + d_a; est_y = ay;
    confidence = 20;
  } else {
    float d_r = rssiToDistance(g_relay_rssi);
    est_x = rx - d_r; est_y = ry;
    confidence = 15;
  }

  PktPosResult res{};
  res.hdr.pkt_type  = PKT_POS_RESULT;
  res.hdr.src       = NODE_ID;
  res.hdr.dst       = g_pos_mobile_addr;
  res.hdr.seq       = r_nextSeq();
  res.hdr.ttl       = 2;
  res.hdr.hop_src   = NODE_ID;
  res.req_id        = g_pos_req_id;
  res.measured_addr = g_pos_mobile_addr;
  res.est_x_cm      = (int16_t)(est_x * 100);
  res.est_y_cm      = (int16_t)(est_y * 100);
  res.confidence    = confidence;
  res.anchor_count  = (g_anchor_meas_valid ? 1 : 0);
  res.refined       = refined;
  radio_send((uint8_t *)&res, sizeof(res));

  uint8_t peer = (g_pos_mobile_addr == ADDR_MOBILE_1) ? ADDR_MOBILE_2 : ADDR_MOBILE_1;
  uint8_t peer_nh = g_rt.getNextHop(peer);
  if (peer_nh != 0x00) {
    delay(60);
    PktPosResult peer_res = res;
    peer_res.hdr.dst = peer;
    peer_res.hdr.hop_src = NODE_ID;
    peer_res.hdr.seq = r_nextSeq();
    radio_send((uint8_t *)&peer_res, sizeof(peer_res));
  }

  g_pos_pending       = false;
  g_anchor_meas_valid = false;
  g_relay_meas_valid  = false;
}

static void r_sendBeacon() {
  PktBeacon bcn{};
  bcn.hdr.pkt_type  = PKT_BEACON;
  bcn.hdr.src       = NODE_ID;
  bcn.hdr.dst       = ADDR_BROADCAST;
  bcn.hdr.seq       = r_nextSeq();
  bcn.hdr.ttl       = 1;
  bcn.hdr.hop_src   = NODE_ID;
  bcn.frame_num     = g_frame_num;
  bcn.frame_ts_ms   = millis();
  bcn.active_nodes  = 0;
  memcpy(bcn.slot_owner, DEFAULT_SLOT_OWNERS, sizeof(DEFAULT_SLOT_OWNERS));
  radio_send((uint8_t *)&bcn, sizeof(bcn));
  DLOG("BEACON f=%lu", (unsigned long)g_frame_num);
}

static void r_handleRx() {
  static uint8_t buf[64];
  uint32_t dio1_time = g_rxTimeUs;
  g_rxFlag = false;

  size_t len = radio.readData(buf, sizeof(buf));
  int8_t rssi = radio.getRSSI();

  if (len < sizeof(SnipsHeader)) return;
  auto &hdr = *(SnipsHeader *)buf;
  if (hdr.src == NODE_ID) return;

  g_rt.updateNeighbour(hdr.hop_src, rssi, g_frame_num);

  switch (hdr.pkt_type) {
  case PKT_DATA: {
    auto &d = *(PktData *)buf;
    mesh_sendAck(hdr, rssi);
    if (hdr.dst == NODE_ID || hdr.dst == ADDR_BROADCAST) {
      DLOG("DATA 0x%02X %.16s", (int)hdr.src, d.payload);
    } else {
      DLOG("FWD 0x%02X>0x%02X", (int)hdr.src, (int)hdr.dst);
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
  case PKT_POS_REQ: {
    auto &req = *(PktPosReq *)buf;
    mesh_sendAck(hdr, rssi);
    g_relay_rssi       = rssi;
    g_relay_rx_time_us = dio1_time;
    g_relay_meas_valid = true;
    g_pos_pending     = true;
    g_pos_req_id      = req.req_id;
    g_pos_mobile_addr = hdr.src;
    g_anchor_meas_valid = false;
    DLOG("POS_REQ 0x%02X R=%d", (int)hdr.src, (int)rssi);
    break;
  }
  case PKT_POS_MEAS: {
    auto &m = *(PktPosMeas *)buf;
    if (g_pos_pending && m.req_id == g_pos_req_id) {
      g_anchor_rssi       = m.rssi_dbm;
      g_anchor_rx_time_us = m.rx_time_us;
      g_anchor_meas_valid = true;
      mesh_sendAck(hdr, rssi);
      r_computeAndSendPosition();
    }
    break;
  }
  case PKT_ACK:
    DLOG("ACK 0x%02X sq=%d", (int)hdr.src, (int)((PktAck *)buf)->ack_seq);
    break;
  case PKT_RERR:
    mesh_handleRERR(*(PktRERR *)buf, hdr);
    break;
  }
}

void relay_setup() {
  delay(100);
  
  int16_t err = radio.begin(LORA_FREQ, LORA_BW, LORA_SF, LORA_CR, 0x12, LORA_POWER, LORA_PREAMBLE);
  if (err != 0) {
    while (true) delay(1000);
  }

  radio.setPacketReceivedAction(onDIO1);
  radio.startReceive();
  g_rt.init();
  g_frame_start = millis();
  r_sendBeacon();
}

void relay_loop() {
  uint32_t now     = millis();
  uint32_t elapsed = now - g_frame_start;

  if (elapsed >= SF_DURATION_MS) {
    g_frame_num++;
    g_frame_start = now;
    g_rt.expireRoutes(g_frame_num);
    g_rt.expireNeighbours(g_frame_num);

    if (g_pos_pending) r_computeAndSendPosition();
    g_pos_pending       = false;
    g_anchor_meas_valid = false;
    g_relay_meas_valid  = false;

    r_sendBeacon();
    return;
  }

  if (elapsed >= 900 && g_rerr_pending) {
    mesh_sendRERR(g_rerr_dest, g_rerr_upstream, 0);
    g_rt.removeRoute(g_rerr_dest);
    for (auto &q : g_rt.sf_queue) {
      if (q.occupied && q.pkt.hdr.dst == g_rerr_dest) q.occupied = false;
    }
    g_rerr_pending = false;
  }

  if (elapsed >= SF_CTRL_END_MS && elapsed < SF_DATA_END_MS) {
    mesh_flushSFQueue(g_frame_num);
  }

  if (g_rxFlag) {
    r_handleRx();
  }
}
#endif // NODE_TYPE_RELAY
