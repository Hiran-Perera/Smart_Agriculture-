#pragma once
// Shared by NodeA and NodeB (keep the two copies identical).
//
// Holds the user sensor assignment (EXT slots + NodeB flow pin), stores it in NVS,
// wires the GPIOs/interrupts and samples the values for TELEMETRY_X.

#include <Arduino.h>
#include <Preferences.h>
#include "payloads.h"
#include "sensor_map.h"

namespace ExtSensors {

using namespace Payload;

static const uint8_t FLOW_IDX = EXT_SLOTS;   // pulse counter index used for the flow pin

static Preferences prefs;
static uint8_t g_self = NODE_A;
static CmdSensorCfg g_cfg;

static volatile uint32_t g_pulses[EXT_SLOTS + 1];
static uint32_t g_snap[EXT_SLOTS + 1];
static uint8_t  g_attached[EXT_SLOTS + 1];   // pin currently attached per counter, PIN_NONE if none

static void IRAM_ATTR isr_pulse(void* arg) { g_pulses[(uintptr_t)arg]++; }

static void detach_all() {
  for (uint8_t i = 0; i <= EXT_SLOTS; i++) {
    if (g_attached[i] != PIN_NONE) {
      detachInterrupt(digitalPinToInterrupt(g_attached[i]));
      g_attached[i] = PIN_NONE;
    }
  }
}

static void attach_counter(uint8_t idx, uint8_t pin) {
  pinMode(pin, INPUT_PULLUP);
  g_pulses[idx] = 0;
  attachInterruptArg(digitalPinToInterrupt(pin), isr_pulse, (void*)(uintptr_t)idx, RISING);
  g_attached[idx] = pin;
}

static void apply() {
  detach_all();

  if (SensorMap::info(g_self).hasFlow) attach_counter(FLOW_IDX, g_cfg.flow_pin);

  for (uint8_t i = 0; i < EXT_SLOTS; i++) {
    const SensorSlotCfg& s = g_cfg.slot[i];
    if (s.type == SENS_ANALOG)       pinMode(s.pin, INPUT);
    else if (s.type == SENS_DIGITAL) pinMode(s.pin, INPUT_PULLUP);
    else if (s.type == SENS_PULSE)   attach_counter(i, s.pin);
  }
}

static void begin(uint8_t selfId) {
  g_self = selfId;
  for (uint8_t i = 0; i <= EXT_SLOTS; i++) { g_attached[i] = PIN_NONE; g_pulses[i] = 0; }

  prefs.begin(selfId == NODE_B ? "sensB" : "sensA", false);

  bool ok = prefs.getBytesLength("cfg") == sizeof(CmdSensorCfg);
  if (ok) prefs.getBytes("cfg", &g_cfg, sizeof(g_cfg));
  if (!ok || !SensorMap::valid(g_self, g_cfg)) SensorMap::defaults(g_self, g_cfg);

  apply();
}

// Returns true (and applies + stores) when the assignment is valid.
static bool set_config(const CmdSensorCfg& c) {
  if (c.ver != 1 || !SensorMap::valid(g_self, c)) return false;
  g_cfg = c;
  prefs.putBytes("cfg", &g_cfg, sizeof(g_cfg));
  apply();
  return true;
}

static inline uint8_t flow_pin() { return g_cfg.flow_pin; }

static bool any_active() {
  for (uint8_t i = 0; i < EXT_SLOTS; i++) if (g_cfg.slot[i].type != SENS_NONE) return true;
  return false;
}

// Pulse measurement window: call window_begin(), wait, then read with windowMs.
static void window_begin() {
  for (uint8_t i = 0; i <= EXT_SLOTS; i++) g_snap[i] = g_pulses[i];
}

static uint16_t hz_x10(uint8_t idx, uint32_t windowMs) {
  if (windowMs == 0) return 0;
  uint32_t dp = g_pulses[idx] - g_snap[idx];
  uint32_t v = dp * 10000UL / windowMs;
  return (uint16_t)(v > 65535UL ? 65535UL : v);
}

static bool has_pulse_slot() {
  for (uint8_t i = 0; i < EXT_SLOTS; i++) if (g_cfg.slot[i].type == SENS_PULSE) return true;
  return false;
}

static void sample(TelemetryX& tx, uint32_t windowMs) {
  tx.ver = 1;
  for (uint8_t i = 0; i < EXT_SLOTS; i++) {
    const SensorSlotCfg& s = g_cfg.slot[i];
    tx.s[i].type = s.type;
    tx.s[i].pin = s.pin;
    switch (s.type) {
      case SENS_ANALOG:  tx.s[i].value = (uint16_t)analogRead(s.pin); break;
      case SENS_DIGITAL: tx.s[i].value = (uint16_t)digitalRead(s.pin); break;
      case SENS_PULSE:   tx.s[i].value = hz_x10(i, windowMs); break;
      default:           tx.s[i].value = 0; break;
    }
  }
}

} // namespace ExtSensors
