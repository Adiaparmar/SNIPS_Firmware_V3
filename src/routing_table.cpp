#include "routing_table.h"

RoutingTable g_rt;

void RoutingTable::init() {
  for (auto &r : routes) {
    r.valid = false;
    r.dest = 0;
  }
  for (auto &n : neighbours) {
    n.valid = false;
    n.addr = 0;
  }
  for (auto &q : sf_queue) {
    q.occupied = false;
  }
  for (auto &s : rreq_seen) {
    s.valid = false;
  }
}

uint8_t RoutingTable::getNextHop(uint8_t dest) {
  for (auto &r : routes) {
    if (r.valid && r.dest == dest)
      return r.next_hop;
  }
  return 0x00;
}

bool RoutingTable::addRoute(uint8_t dest, uint8_t next_hop, uint8_t hops,
                            int8_t min_rssi, uint32_t frame) {
  // Check for existing route to dest
  for (auto &r : routes) {
    if (r.valid && r.dest == dest) {
      bool better = (hops < r.hop_count) ||
                    (hops == r.hop_count && min_rssi > r.min_rssi);
      if (!better)
        return false;
      r.next_hop = next_hop;
      r.hop_count = hops;
      r.min_rssi = min_rssi;
      r.last_updated = frame;
      return true;
    }
  }
  // Find a free slot (or evict oldest)
  RouteEntry *oldest = nullptr;
  for (auto &r : routes) {
    if (!r.valid) {
      oldest = &r;
      break;
    }
    if (!oldest || r.last_updated < oldest->last_updated)
      oldest = &r;
  }
  if (oldest) {
    oldest->dest = dest;
    oldest->next_hop = next_hop;
    oldest->hop_count = hops;
    oldest->min_rssi = min_rssi;
    oldest->last_updated = frame;
    oldest->valid = true;
    return true;
  }
  return false;
}

void RoutingTable::removeRoute(uint8_t dest) {
  for (auto &r : routes) {
    if (r.valid && r.dest == dest) {
      r.valid = false;
      return;
    }
  }
}

void RoutingTable::updateNeighbour(uint8_t addr, int8_t rssi, uint32_t frame) {
  for (auto &n : neighbours) {
    if (n.valid && n.addr == addr) {
      n.rssi = rssi;
      n.last_seen_frame = frame;
      return;
    }
  }
  // Add new — evict oldest or first free
  NeighbourEntry *slot = nullptr;
  for (auto &n : neighbours) {
    if (!n.valid) {
      slot = &n;
      break;
    }
    if (!slot || n.last_seen_frame < slot->last_seen_frame)
      slot = &n;
  }
  if (slot) {
    slot->addr = addr;
    slot->rssi = rssi;
    slot->last_seen_frame = frame;
    slot->valid = true;
  }
}

bool RoutingTable::isRREQSeen(uint8_t orig, uint8_t rreq_id) {
  for (auto &s : rreq_seen) {
    if (s.valid && s.orig_addr == orig && s.rreq_id == rreq_id)
      return true;
  }
  return false;
}

void RoutingTable::addRREQSeen(uint8_t orig, uint8_t rreq_id) {
  // Circular eviction based on the configured cache size.
  uint8_t slot = rreq_id % RREQ_SEEN_SIZE;
  rreq_seen[slot].orig_addr = orig;
  rreq_seen[slot].rreq_id = rreq_id;
  rreq_seen[slot].valid = true;
}

bool RoutingTable::enqueue(const PktData &pkt, uint32_t frame) {
  for (auto &q : sf_queue) {
    if (!q.occupied) {
      q.pkt = pkt;
      q.retry_count = 0;
      q.queued_frame = frame;
      q.occupied = true;
      return true;
    }
  }
  return false; // queue full
}

uint8_t RoutingTable::flushQueue(uint32_t frame,
                                 bool (*sendFn)(const PktData &)) {
  uint8_t sent = 0;
  for (auto &q : sf_queue) {
    if (!q.occupied)
      continue;

    // Age check — silently drop stale entries
    if (frame - q.queued_frame > ROUTE_EXPIRE_FRAMES) {
      q.occupied = false;
      continue;
    }

    // Route check (use this->getNextHop)
    uint8_t nh = this->getNextHop(q.pkt.hdr.dst);
    if (nh == 0x00)
      continue; // RREQ in progress, skip

    // Attempt forward
    bool ok = sendFn(q.pkt);
    if (ok) {
      q.occupied = false;
      sent++;
    } else {
      q.retry_count++;
      if (q.retry_count >= MAX_RETRIES) {
        q.occupied = false; // silently drop after max retries
      }
    }
  }
  return sent;
}

void RoutingTable::expireRoutes(uint32_t frame) {
  for (auto &r : routes) {
    if (r.valid && (frame - r.last_updated) > ROUTE_EXPIRE_FRAMES) {
      r.valid = false;
    }
  }
}

void RoutingTable::expireNeighbours(uint32_t frame) {
  for (auto &n : neighbours) {
    if (n.valid && (frame - n.last_seen_frame) > NEIGHBOUR_EXPIRE_FRAMES) {
      n.valid = false;
    }
  }
}

int8_t RoutingTable::getNeighbourRSSI(uint8_t addr) {
  for (auto &n : neighbours) {
    if (n.valid && n.addr == addr)
      return n.rssi;
  }
  return -120;
}
