#pragma once
#include "snips_config.h"

// Max entries limits to fit within Uno's 2KB SRAM
#ifndef ROUTE_CACHE_SIZE
#define ROUTE_CACHE_SIZE 10
#endif
#ifndef NEIGHBOUR_CACHE_SIZE
#define NEIGHBOUR_CACHE_SIZE 10
#endif
#ifndef QUEUE_SIZE
#define QUEUE_SIZE 5
#endif
#ifndef RREQ_SEEN_SIZE
#define RREQ_SEEN_SIZE 8
#endif

struct RouteEntry {
  bool valid;
  uint8_t dest;
  uint8_t next_hop;
  uint8_t hop_count;
  int8_t min_rssi;
  uint32_t last_updated;
};

struct NeighbourEntry {
  bool valid;
  uint8_t addr;
  int8_t rssi;
  uint32_t last_seen_frame;
};

struct QueueEntry {
  bool occupied;
  PktData pkt;
  uint8_t retry_count;
  uint32_t queued_frame;
};

struct RREQSeenCache {
  bool valid;
  uint8_t orig_addr;
  uint8_t rreq_id;
};

class RoutingTable {
public:
  RouteEntry routes[ROUTE_CACHE_SIZE];
  NeighbourEntry neighbours[NEIGHBOUR_CACHE_SIZE];
  QueueEntry sf_queue[QUEUE_SIZE]; // Store-and-forward queue
  RREQSeenCache rreq_seen[RREQ_SEEN_SIZE];

  void init();

  // Return 0x00 if no route
  uint8_t getNextHop(uint8_t dest);

  // Return true if route was added/updated
  bool addRoute(uint8_t dest, uint8_t next_hop, uint8_t hops, int8_t min_rssi,
                uint32_t frame);
  void removeRoute(uint8_t dest);

  void updateNeighbour(uint8_t addr, int8_t rssi, uint32_t frame);
  int8_t getNeighbourRSSI(uint8_t addr);

  // RREQ flood management
  bool isRREQSeen(uint8_t orig, uint8_t rreq_id);
  void addRREQSeen(uint8_t orig, uint8_t rreq_id);

  // Queue
  bool enqueue(const PktData &pkt, uint32_t frame);

  // Flush calls sendFn(pkt) for each ready packet. Returns count sent.
  uint8_t flushQueue(uint32_t frame, bool (*sendFn)(const PktData &));

  void expireRoutes(uint32_t frame);
  void expireNeighbours(uint32_t frame);
};

extern RoutingTable g_rt;
