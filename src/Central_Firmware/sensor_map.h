#pragma once
// Shared by Central, NodeA and NodeB (keep the three copies identical).
//
// Describes which GPIOs of each ESP32-C3 node are free for user sensors and
// validates / edits a sensor assignment (Payload::CmdSensorCfg).
//
// Fixed (not changeable) wiring:
//   Node A : LDR GPIO3, DHT11 GPIO2, battery GPIO0
//   Node B : soil GPIO1, relay GPIO2, battery GPIO0
//   Both   : LoRa SPI on GPIO4,5,6,7,20,21
// Changeable:
//   Node B flow sensor (default GPIO3) and the EXT slots on both nodes, all
//   chosen from the node's pool below.
//
// Pool notes: GPIO8/9 are strapping pins (do not hold them low at boot).
// GPIO18/19 are USB and 12-17 are flash, so they are never offered.
// Only GPIO0-4 are ADC capable on the C3.

#include <Arduino.h>
#include "payloads.h"

namespace SensorMap {

using Payload::EXT_SLOTS;
using Payload::PIN_NONE;
using Payload::CmdSensorCfg;

static const uint8_t FLOW_SLOT = EXT_SLOTS;   // pseudo slot index for the flow pin

struct NodeInfo {
  uint8_t poolN;
  uint8_t pool[6];
  bool    hasFlow;
  uint8_t flowDefault;
};

static inline const NodeInfo& info(uint8_t nodeId) {
  static const NodeInfo A = { 4, {1, 8, 9, 10, 0, 0}, false, PIN_NONE };
  static const NodeInfo B = { 4, {3, 8, 9, 10, 0, 0}, true,  3 };
  return (nodeId == Payload::NODE_B) ? B : A;
}

static inline bool is_adc(uint8_t pin) { return pin <= 4; }

static inline bool in_pool(uint8_t nodeId, uint8_t pin) {
  const NodeInfo& n = info(nodeId);
  for (uint8_t i = 0; i < n.poolN; i++) if (n.pool[i] == pin) return true;
  return false;
}

static inline void defaults(uint8_t nodeId, CmdSensorCfg& c) {
  c.ver = 1;
  c.flow_pin = info(nodeId).flowDefault;
  for (uint8_t i = 0; i < EXT_SLOTS; i++) { c.slot[i].type = Payload::SENS_NONE; c.slot[i].pin = PIN_NONE; }
}

// Bitmask of pins taken by every assignment except `skip` (slot index, FLOW_SLOT or 0xFF).
static inline uint32_t used_mask(uint8_t nodeId, const CmdSensorCfg& c, uint8_t skip) {
  uint32_t m = 0;
  if (info(nodeId).hasFlow && skip != FLOW_SLOT && c.flow_pin < 32) m |= (1UL << c.flow_pin);
  for (uint8_t i = 0; i < EXT_SLOTS; i++) {
    if (i == skip || c.slot[i].type == Payload::SENS_NONE) continue;
    if (c.slot[i].pin < 32) m |= (1UL << c.slot[i].pin);
  }
  return m;
}

static inline bool pin_ok(uint8_t nodeId, const CmdSensorCfg& c, uint8_t skip, uint8_t type, uint8_t pin) {
  if (!in_pool(nodeId, pin)) return false;
  if (used_mask(nodeId, c, skip) & (1UL << pin)) return false;
  if (type == Payload::SENS_ANALOG && !is_adc(pin)) return false;
  return true;
}

static inline bool valid(uint8_t nodeId, const CmdSensorCfg& c) {
  if (info(nodeId).hasFlow) {
    if (!pin_ok(nodeId, c, FLOW_SLOT, Payload::SENS_DIGITAL, c.flow_pin)) return false;
  }
  for (uint8_t i = 0; i < EXT_SLOTS; i++) {
    if (c.slot[i].type > Payload::SENS_PULSE) return false;
    if (c.slot[i].type == Payload::SENS_NONE) continue;
    if (!pin_ok(nodeId, c, i, c.slot[i].type, c.slot[i].pin)) return false;
  }
  return true;
}

// First pin in the pool that is allowed for (slot, type), or PIN_NONE.
static inline uint8_t first_free_pin(uint8_t nodeId, const CmdSensorCfg& c, uint8_t slot, uint8_t type) {
  const NodeInfo& n = info(nodeId);
  for (uint8_t i = 0; i < n.poolN; i++) {
    if (pin_ok(nodeId, c, slot, type, n.pool[i])) return n.pool[i];
  }
  return PIN_NONE;
}

// Step to the next/previous allowed pin for an EXT slot or the flow pin. Pins already
// used by another sensor are skipped. Returns false if nothing else is available.
static inline bool cycle_pin(uint8_t nodeId, CmdSensorCfg& c, uint8_t slot, int8_t dir) {
  const NodeInfo& n = info(nodeId);
  uint8_t type = (slot == FLOW_SLOT) ? Payload::SENS_DIGITAL : c.slot[slot].type;
  uint8_t cur  = (slot == FLOW_SLOT) ? c.flow_pin : c.slot[slot].pin;

  int start = 0;
  for (uint8_t i = 0; i < n.poolN; i++) if (n.pool[i] == cur) start = i;

  for (uint8_t k = 1; k <= n.poolN; k++) {
    int idx = ((start + (dir >= 0 ? (int)k : -(int)k)) % n.poolN + n.poolN) % n.poolN;
    uint8_t p = n.pool[idx];
    if (p == cur) continue;
    if (!pin_ok(nodeId, c, slot, type, p)) continue;
    if (slot == FLOW_SLOT) c.flow_pin = p; else c.slot[slot].pin = p;
    return true;
  }
  return false;
}

// Step the type of an EXT slot. Types with no usable pin are skipped; the pin is
// re-picked when the current one does not suit the new type.
static inline void cycle_type(uint8_t nodeId, CmdSensorCfg& c, uint8_t slot, int8_t dir) {
  const uint8_t N = 4; // NONE, ANALOG, DIGITAL, PULSE
  uint8_t t = c.slot[slot].type;
  for (uint8_t k = 1; k <= N; k++) {
    uint8_t nt = (uint8_t)(((int)t + (dir >= 0 ? (int)k : -(int)k) % N + N) % N);
    if (nt == Payload::SENS_NONE) {
      c.slot[slot].type = nt;
      c.slot[slot].pin = PIN_NONE;
      return;
    }
    uint8_t pin = c.slot[slot].pin;
    if (c.slot[slot].type == Payload::SENS_NONE || !pin_ok(nodeId, c, slot, nt, pin)) {
      pin = first_free_pin(nodeId, c, slot, nt);
    }
    if (pin == PIN_NONE) continue;
    c.slot[slot].type = nt;
    c.slot[slot].pin = pin;
    return;
  }
}

static inline const char* type_name(uint8_t t) {
  switch (t) {
    case Payload::SENS_ANALOG:  return "ANALOG";
    case Payload::SENS_DIGITAL: return "DIGITAL";
    case Payload::SENS_PULSE:   return "PULSE";
    default:                    return "OFF";
  }
}

} // namespace SensorMap
