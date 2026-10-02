#include "mesh_router.h"

bool mesh_waitAck(uint8_t expected_seq) {
  uint32_t deadline = millis() + ACK_TIMEOUT_MS;
  while (millis() < deadline) {
    if (!g_rxFlag) {
      delay(5);
      continue;
    }
    g_rxFlag = false;

    uint8_t buf[sizeof(PktAck)];
    size_t len = radio.readData(buf, sizeof(buf));
    if (len < sizeof(PktAck)) {
      continue;
    }

    auto &ack = *(PktAck *)buf;
    if (ack.hdr.pkt_type == PKT_ACK && ack.hdr.dst == NODE_ID &&
        ack.ack_seq == expected_seq) {
      return true;
    }
  }
  return false;
}

void mesh_sendAck(const SnipsHeader &orig, int8_t rssi) {
  PktAck ack{};
  ack.hdr.pkt_type = PKT_ACK;
  ack.hdr.src = NODE_ID;
  ack.hdr.dst = orig.hop_src;
  ack.hdr.seq = mesh_nextSeq();
  ack.hdr.ttl = 1;
  ack.hdr.hop_src = NODE_ID;
  ack.ack_seq = orig.seq;
  ack.status = 0;
  ack.rssi_dbm = rssi;
  radio_send((uint8_t *)&ack, sizeof(ack));
}

bool mesh_sendRERR(uint8_t broken_dest, uint8_t upstream_node,
                   uint8_t error_code) {
  if (upstream_node == 0x00 || upstream_node == NODE_ID)
    return false;

  PktRERR rerr{};
  rerr.hdr.pkt_type = PKT_RERR;
  rerr.hdr.src = NODE_ID;
  rerr.hdr.dst = upstream_node;
  rerr.hdr.seq = mesh_nextSeq();
  rerr.hdr.ttl = 1;
  rerr.hdr.hop_src = NODE_ID;
  rerr.broken_dest = broken_dest;
  rerr.error_code = error_code;
  radio_send((uint8_t *)&rerr, sizeof(rerr));
  DLOG("RERR>0x%02X brk=0x%02X", (int)upstream_node, (int)broken_dest);
  return true;
}

bool mesh_sendForwardedData(PktData &pkt) {
  if (pkt.hdr.ttl <= 1)
    return false;

  PktData fwd = pkt;
  fwd.hdr.ttl = pkt.hdr.ttl - 1;
  fwd.hdr.hop_src = NODE_ID;
  fwd.hdr.seq = mesh_nextSeq();

  delay(20);
  if (!radio_send((uint8_t *)&fwd, sizeof(fwd)))
    return false;
  return mesh_waitAck(fwd.hdr.seq);
}

// Forwarding helper for function pointer cast
static bool _mesh_sendForwardedDataHelper(const PktData &pkt) {
  PktData copy = pkt;
  return mesh_sendForwardedData(copy);
}

bool mesh_forwardData(PktData &pkt) {
  if (pkt.hdr.ttl <= 1) {
    DLOG("TTL exp drop 0x%02X", (int)pkt.hdr.dst);
    return false;
  }

  uint8_t next_hop = g_rt.getNextHop(pkt.hdr.dst);
  if (next_hop == 0x00) {
    if (g_rt.enqueue(pkt, mesh_currentFrame())) {
      DLOG("NOROUTE 0x%02X queued", (int)pkt.hdr.dst);
    } else {
      DLOG("NOROUTE 0x%02X qfull", (int)pkt.hdr.dst);
      mesh_sendRERR(pkt.hdr.dst, pkt.hdr.hop_src, 2);
    }
    return false;
  }

  // Cap the total time this call is allowed to spend retrying so a
  // marginal link can't consume the entire 600ms data phase and starve
  // RX polling for unrelated in-flight packets/ACKs. There's no
  // frame-start reference available here, so this uses a hard ceiling
  // of half the data-phase window per forward call instead.
  static const uint32_t MAX_FORWARD_BUDGET_MS =
      (SF_DATA_END_MS - SF_CTRL_END_MS) / 2; // 300ms
  uint32_t attempt_start = millis();

  for (uint8_t attempt = 0; attempt < MAX_RETRIES; attempt++) {
    if (mesh_sendForwardedData(pkt)) {
      DLOG("FWD 0x%02X>0x%02X OK", (int)pkt.hdr.src, (int)pkt.hdr.dst);
      return true;
    }

    uint32_t backoff = 100u * (1u << attempt);
    if (millis() - attempt_start + backoff > MAX_FORWARD_BUDGET_MS) {
      DLOG("FWD budget exceeded, abort retries");
      break;
    }

    delay(backoff);
  }

  if (!g_rt.enqueue(pkt, mesh_currentFrame())) {
    g_rt.removeRoute(pkt.hdr.dst);
    mesh_sendRERR(pkt.hdr.dst, pkt.hdr.hop_src, 0);
  } else {
    DLOG("FWD FAIL 0x%02X queued", (int)pkt.hdr.dst);
  }
  return false;
}

void mesh_handleRREQ(const PktRREQ &rreq, const SnipsHeader &hdr, int8_t rssi) {
  if (g_rt.isRREQSeen(rreq.orig_addr, rreq.rreq_id))
    return;
  g_rt.addRREQSeen(rreq.orig_addr, rreq.rreq_id);

  int8_t link_rssi = min((int)rssi, (int)rreq.min_rssi);
  g_rt.addRoute(rreq.orig_addr, hdr.hop_src, rreq.hop_count + 1, link_rssi,
                mesh_currentFrame());

  if (rreq.dest_addr == NODE_ID) {
    PktRREP rrep{};
    rrep.hdr.pkt_type = PKT_RREP;
    rrep.hdr.src = NODE_ID;
    rrep.hdr.dst = rreq.orig_addr;
    rrep.hdr.seq = mesh_nextSeq();
    rrep.hdr.ttl = 4;
    rrep.hdr.hop_src = NODE_ID;
    rrep.dest_addr = NODE_ID;
    rrep.orig_addr = rreq.orig_addr;
    rrep.rreq_id = rreq.rreq_id;
    rrep.hop_count = 1;
    rrep.min_rssi = rssi;
    delay(20);
    radio_send((uint8_t *)&rrep, sizeof(rrep));
    DLOG("RREP>0x%02X (I am dst)", (int)rreq.orig_addr);
    return;
  }

  if (hdr.ttl > 1) {
    PktRREQ fwd = rreq;
    fwd.hdr.hop_src = NODE_ID;
    fwd.hdr.ttl = hdr.ttl - 1;
    fwd.hop_count = rreq.hop_count + 1;
    fwd.min_rssi = (int8_t)link_rssi;
    fwd.hdr.seq = mesh_nextSeq();
    delay(20);
    radio_send((uint8_t *)&fwd, sizeof(fwd));
    DLOG("RREQ fwd 0x%02X h=%d", (int)rreq.orig_addr, (int)fwd.hop_count);
  }
}

void mesh_handleRREP(const PktRREP &rrep, const SnipsHeader &hdr, int8_t rssi) {
  int8_t link_rssi = min((int)rssi, (int)rrep.min_rssi);
  g_rt.addRoute(rrep.dest_addr, hdr.hop_src, rrep.hop_count, link_rssi,
                mesh_currentFrame());
  DLOG("RREP 0x%02X h=%d", (int)rrep.dest_addr, (int)rrep.hop_count);

  if (rrep.orig_addr == NODE_ID)
    return;

  uint8_t next_hop = g_rt.getNextHop(rrep.orig_addr);
  if (next_hop == 0x00 || hdr.ttl <= 1)
    return;

  PktRREP fwd = rrep;
  fwd.hdr.hop_src = NODE_ID;
  fwd.hdr.ttl = hdr.ttl - 1;
  fwd.hdr.seq = mesh_nextSeq();
  fwd.hop_count = rrep.hop_count + 1;
  fwd.min_rssi = (int8_t)link_rssi;
  delay(20);
  radio_send((uint8_t *)&fwd, sizeof(fwd));
}

void mesh_handleRERR(const PktRERR &re, const SnipsHeader &hdr) {
  g_rt.removeRoute(re.broken_dest);

  for (auto &q : g_rt.sf_queue) {
    if (q.occupied && q.pkt.hdr.dst == re.broken_dest) {
      q.occupied = false;
    }
  }

  DLOG("RERR<0x%02X brk=0x%02X", (int)hdr.src, (int)re.broken_dest);

  if (hdr.dst == NODE_ID || hdr.dst == ADDR_BROADCAST || hdr.ttl <= 1)
    return;

  uint8_t next_hop = g_rt.getNextHop(hdr.dst);
  if (next_hop == 0x00)
    return;

  PktRERR fwd = re;
  fwd.hdr.hop_src = NODE_ID;
  fwd.hdr.ttl = hdr.ttl - 1;
  fwd.hdr.seq = mesh_nextSeq();
  delay(20);
  radio_send((uint8_t *)&fwd, sizeof(fwd));
}

uint8_t mesh_flushSFQueue(uint32_t frame) {
  return g_rt.flushQueue(frame, _mesh_sendForwardedDataHelper);
}
