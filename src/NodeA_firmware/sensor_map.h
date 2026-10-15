#pragma once
// Shared by Central, NodeA and NodeB (keep the three copies identical).
//
// Describes which GPIOs of each ESP32-C3 node are free for user sensors and
// validates / edits a sensor assignment (Payload::CmdSensorCfg).
//
// Fixed wiring (not user editable, not shown in the Sensors page):
//   Node A : LDR GPIO3, DHT11 GPIO2, battery GPIO0
//   Node B : soil GPIO1, main pump relay GPIO2, battery GPIO0
//   Both   : LoRa SPI on GPIO4,5,6,7,20,21
//
// User slots pick a sensor type and a pin from the node's pool below.
// GPIO8/9 are strapping pins (sensors must not hold them low at boot).
// GPIO18/19 are USB and 12-17 are flash, so they are never offered.
// Only GPIO0-4 are ADC capable on the C3, which limits analog sensors.

#include <Arduino.h>
#include "payloads.h"

namespace SensorMap {

using Payload::EXT_SLOTS;
using Payload::PIN_NONE;
using Payload::CmdSensorCfg;

static const uint8_t CFG_VER = 2;

struct NodeInfo {
  uint8_t poolN;
  uint8_t pool[6];
};

static inline const NodeInfo& info(uint8_t nodeId) {
  static const NodeInfo A = { 4, {1, 8, 9, 10, 0, 0} };
  static const NodeInfo B = { 4, {3, 8, 9, 10, 0, 0} };
  return (nodeId == Payload::NODE_B) ? B : A;
}

static inline bool is_adc(uint8_t pin) { return pin <= 4; }

static inline bool needs_adc(uint8_t type) {
  return type == Payload::SENS_LDR || type == Payload::SENS_SOIL ||
         type == Payload::SENS_TDS || type == Payload::SENS_PH;
}

static inline bool is_output(uint8_t type) {
  return type == Payload::SENS_PUMP || type == Payload::SENS_VALVE;
}

static inline bool in_pool(uint8_t nodeId, uint8_t pin) {
  const NodeInfo& n = info(nodeId);
  for (uint8_t i = 0; i < n.poolN; i++) if (n.pool[i] == pin) return true;
  return false;
}

static inline void defaults(uint8_t nodeId, CmdSensorCfg& c) {
  c.ver = CFG_VER;
  for (uint8_t i = 0; i < EXT_SLOTS; i++) { c.slot[i].type = Payload::SENS_NONE; c.slot[i].pin = PIN_NONE; }
  if (nodeId == Payload::NODE_B) {           // existing flow sensor wiring
    c.slot[0].type = Payload::SENS_FLOW;
    c.slot[0].pin = 3;
  }
}

// Bitmask of pins taken by every slot except `skip` (0xFF = none skipped).
static inline uint32_t used_mask(const CmdSensorCfg& c, uint8_t skip) {
  uint32_t m = 0;
  for (uint8_t i = 0; i < EXT_SLOTS; i++) {
    if (i == skip || c.slot[i].type == Payload::SENS_NONE) continue;
    if (c.slot[i].pin < 32) m |= (1UL << c.slot[i].pin);
  }
  return m;
}

static inline bool pin_ok(uint8_t nodeId, const CmdSensorCfg& c, uint8_t slot, uint8_t type, uint8_t pin) {
  if (!in_pool(nodeId, pin)) return false;
  if (used_mask(c, slot) & (1UL << pin)) return false;
  if (needs_adc(type) && !is_adc(pin)) return false;
  return true;
}

static inline bool valid(uint8_t nodeId, const CmdSensorCfg& c) {
  if (c.ver != CFG_VER) return false;
  for (uint8_t i = 0; i < EXT_SLOTS; i++) {
    if (c.slot[i].type >= Payload::SENS_TYPE_COUNT) return false;
    if (c.slot[i].type == Payload::SENS_NONE) continue;
    if (!pin_ok(nodeId, c, i, c.slot[i].type, c.slot[i].pin)) return false;
  }
  return true;
}

static inline bool differs(const CmdSensorCfg& a, const CmdSensorCfg& b) {
  return memcmp(&a, &b, sizeof(CmdSensorCfg)) != 0;
}

// First pin in the pool allowed for (slot, type), or PIN_NONE.
static inline uint8_t first_free_pin(uint8_t nodeId, const CmdSensorCfg& c, uint8_t slot, uint8_t type) {
  const NodeInfo& n = info(nodeId);
  for (uint8_t i = 0; i < n.poolN; i++) {
    if (pin_ok(nodeId, c, slot, type, n.pool[i])) return n.pool[i];
  }
  return PIN_NONE;
}

// Step to the next/previous allowed pin of a slot; pins used elsewhere are skipped.
// Returns false if no other pin is available.
static inline bool cycle_pin(uint8_t nodeId, CmdSensorCfg& c, uint8_t slot, int8_t dir) {
  const NodeInfo& n = info(nodeId);
  const uint8_t type = c.slot[slot].type;
  const uint8_t cur  = c.slot[slot].pin;
  if (type == Payload::SENS_NONE) return false;

  int start = 0;
  for (uint8_t i = 0; i < n.poolN; i++) if (n.pool[i] == cur) start = i;

  for (uint8_t k = 1; k <= n.poolN; k++) {
    int idx = ((start + (dir >= 0 ? (int)k : -(int)k)) % n.poolN + n.poolN) % n.poolN;
    uint8_t p = n.pool[idx];
    if (p == cur || !pin_ok(nodeId, c, slot, type, p)) continue;
    c.slot[slot].pin = p;
    return true;
  }
  return false;
}

// Step the type of a slot. Types with no usable pin (e.g. a second analog sensor
// when the only ADC pin is taken) are skipped; the pin is re-picked if needed.
static inline void cycle_type(uint8_t nodeId, CmdSensorCfg& c, uint8_t slot, int8_t dir) {
  const uint8_t N = Payload::SENS_TYPE_COUNT;
  const uint8_t t = c.slot[slot].type;
  for (uint8_t k = 1; k <= N; k++) {
    uint8_t nt = (uint8_t)(((int)t + (dir >= 0 ? (int)k : -(int)k) % N + N) % N);
    if (nt == Payload::SENS_NONE) {
      c.slot[slot].type = nt;
      c.slot[slot].pin = PIN_NONE;
      return;
    }
    uint8_t pin = c.slot[slot].pin;
    if (t == Payload::SENS_NONE || !pin_ok(nodeId, c, slot, nt, pin)) {
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
    case Payload::SENS_DHT11: return "DHT11";
    case Payload::SENS_LDR:   return "LDR";
    case Payload::SENS_SOIL:  return "SOIL";
    case Payload::SENS_FLOW:  return "FLOW";
    case Payload::SENS_TDS:   return "TDS";
    case Payload::SENS_PH:    return "PH";
    case Payload::SENS_PUMP:  return "PUMP";
    case Payload::SENS_VALVE: return "VALVE";
    default:                  return "EMPTY";
  }
}

} // namespace SensorMap
