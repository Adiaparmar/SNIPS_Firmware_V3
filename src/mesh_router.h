// ============================================================
//  SNIPS V3 — Shared Mesh Router
// ============================================================
#pragma once

#include "debug_log.h"
#include "SX1262HAL.h"
#include "routing_table.h"
#include "snips_config.h"
#include <Arduino.h>

extern SX1262HAL radio;
extern volatile bool g_rxFlag;

inline bool radio_send(uint8_t* buf, size_t len) {
    g_rxFlag = false;
    // For ESP32 Anchor we also need to toggle RF_SW1 manually,
    // but the node_anchor.cpp will handle that before calling transmit if we just call radio.transmit?
    // Wait, let's let node_anchor handle its own pin toggling by exposing radio directly.
    int16_t state = radio.transmit(buf, len);
    radio.startReceive();
    return (state == 0);
}

uint8_t mesh_nextSeq();
uint32_t mesh_currentFrame();

bool mesh_waitAck(uint8_t expected_seq);
void mesh_sendAck(const SnipsHeader &orig, int8_t rssi);
bool mesh_sendRERR(uint8_t broken_dest, uint8_t upstream_node,
                   uint8_t error_code);
bool mesh_sendForwardedData(PktData &pkt);
bool mesh_forwardData(PktData &pkt);
void mesh_handleRREQ(const PktRREQ &rreq, const SnipsHeader &hdr, int8_t rssi);
void mesh_handleRREP(const PktRREP &rrep, const SnipsHeader &hdr, int8_t rssi);
void mesh_handleRERR(const PktRERR &re, const SnipsHeader &hdr);
uint8_t mesh_flushSFQueue(uint32_t frame);
