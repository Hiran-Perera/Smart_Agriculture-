#pragma once
// Shared by NodeA and NodeB (keep the two copies identical).
//
// Holds the user sensor assignment (EXT slots), stores it in NVS, wires the
// GPIOs / interrupts / DHT objects, drives pump & valve outputs and samples
// the values for TELEMETRY_X.

#include <Arduino.h>
#include <Preferences.h>
#include <DHT.h>
#include "driver/gpio.h"
#include "payloads.h"
#include "sensor_map.h"
#include "lora_link.h"

namespace ExtSensors {

using namespace Payload;

// Calibration (adjust for your probes)
static const float TDS_TEMP_C   = 25.0f;  // no temperature probe: assume 25 C
static const float PH_SLOPE     = 3.5f;   // DFRobot analog pH: pH = 3.5 * V + offset
static const float PH_OFFSET    = 0.0f;

static Preferences prefs;
static uint8_t g_self = NODE_A;
static CmdSensorCfg g_cfg;
static uint8_t g_outStates = 0;            // bit i = output slot i on

static volatile uint32_t g_pulses[EXT_SLOTS];
static uint32_t g_snap[EXT_SLOTS];
static uint8_t  g_attached[EXT_SLOTS];     // pin with an interrupt per slot, PIN_NONE if none
static uint8_t  g_heldOut[EXT_SLOTS];      // pin driven as output per slot, PIN_NONE if none
static DHT*     g_dht[EXT_SLOTS];

static void IRAM_ATTR isr_pulse(void* arg) { g_pulses[(uintptr_t)arg]++; }

static void write_output_(uint8_t slot) {
  const uint8_t pin = g_cfg.slot[slot].pin;
  gpio_hold_dis((gpio_num_t)pin);
  digitalWrite(pin, (g_outStates & (1 << slot)) ? HIGH : LOW);
  gpio_hold_en((gpio_num_t)pin);            // keep the level through light sleep
}

static void release_all_() {
  for (uint8_t i = 0; i < EXT_SLOTS; i++) {
    if (g_attached[i] != PIN_NONE) {
      detachInterrupt(digitalPinToInterrupt(g_attached[i]));
      g_attached[i] = PIN_NONE;
    }
    if (g_heldOut[i] != PIN_NONE) {
      gpio_hold_dis((gpio_num_t)g_heldOut[i]);
      digitalWrite(g_heldOut[i], LOW);
      pinMode(g_heldOut[i], INPUT);
      g_heldOut[i] = PIN_NONE;
    }
    if (g_dht[i]) { delete g_dht[i]; g_dht[i] = nullptr; }
  }
}

static void apply_() {
  release_all_();

  for (uint8_t i = 0; i < EXT_SLOTS; i++) {
    const SensorSlotCfg& s = g_cfg.slot[i];
    switch (s.type) {
      case SENS_LDR: case SENS_SOIL: case SENS_TDS: case SENS_PH:
        pinMode(s.pin, INPUT);
        break;
      case SENS_FLOW:
        pinMode(s.pin, INPUT_PULLUP);
        g_pulses[i] = 0;
        attachInterruptArg(digitalPinToInterrupt(s.pin), isr_pulse, (void*)(uintptr_t)i, RISING);
        g_attached[i] = s.pin;
        break;
      case SENS_DHT11:
        g_dht[i] = new DHT(s.pin, DHT11);
        g_dht[i]->begin();
        break;
      case SENS_PUMP: case SENS_VALVE:
        pinMode(s.pin, OUTPUT);
        g_heldOut[i] = s.pin;
        write_output_(i);
        break;
      default: break;
    }
  }
}

static uint8_t output_mask() {
  uint8_t m = 0;
  for (uint8_t i = 0; i < EXT_SLOTS; i++) if (SensorMap::is_output(g_cfg.slot[i].type)) m |= (1 << i);
  return m;
}

static void begin(uint8_t selfId) {
  g_self = selfId;
  for (uint8_t i = 0; i < EXT_SLOTS; i++) {
    g_attached[i] = PIN_NONE; g_heldOut[i] = PIN_NONE; g_dht[i] = nullptr; g_pulses[i] = 0;
  }

  prefs.begin(selfId == NODE_B ? "sensB" : "sensA", false);

  bool ok = prefs.isKey("cfg2") && prefs.getBytesLength("cfg2") == sizeof(CmdSensorCfg);
  if (ok) prefs.getBytes("cfg2", &g_cfg, sizeof(g_cfg));
  if (!ok || !SensorMap::valid(g_self, g_cfg)) SensorMap::defaults(g_self, g_cfg);

  g_outStates = 0;   // pumps / valves always start OFF after a reset
  apply_();
}

// Returns true (and applies + stores) when the assignment is valid.
static bool set_config(const CmdSensorCfg& c) {
  if (!SensorMap::valid(g_self, c)) return false;
  g_cfg = c;
  prefs.putBytes("cfg2", &g_cfg, sizeof(g_cfg));
  g_outStates &= output_mask();
  apply_();
  return true;
}

// Sets pump/valve outputs; bits of non-output slots are ignored. Returns applied states.
static uint8_t set_outputs(uint8_t states) {
  g_outStates = states & output_mask();
  for (uint8_t i = 0; i < EXT_SLOTS; i++) if (g_heldOut[i] != PIN_NONE) write_output_(i);
  return g_outStates;
}

static bool any_active() {
  for (uint8_t i = 0; i < EXT_SLOTS; i++) if (g_cfg.slot[i].type != SENS_NONE) return true;
  return false;
}

static bool has_flow() {
  for (uint8_t i = 0; i < EXT_SLOTS; i++) if (g_cfg.slot[i].type == SENS_FLOW) return true;
  return false;
}

// Pulse measurement window: call window_begin(), wait, then sample with windowMs.
static void window_begin() {
  for (uint8_t i = 0; i < EXT_SLOTS; i++) g_snap[i] = g_pulses[i];
}

static uint16_t hz_x10(uint8_t slot, uint32_t windowMs) {
  if (windowMs == 0) return 0;
  uint32_t dp = g_pulses[slot] - g_snap[slot];
  uint32_t v = dp * 10000UL / windowMs;
  return (uint16_t)(v > 32767UL ? 32767UL : v);
}

// Hz x10 of the first flow slot (NodeB main telemetry keeps reporting it), 0 if none.
static uint16_t first_flow_hz_x10(uint32_t windowMs) {
  for (uint8_t i = 0; i < EXT_SLOTS; i++) if (g_cfg.slot[i].type == SENS_FLOW) return hz_x10(i, windowMs);
  return 0;
}

static float volts_(uint8_t pin) {
  return (float)analogReadMilliVolts(pin) / 1000.0f;
}

static void sample(TelemetryX& tx, uint32_t windowMs) {
  tx.ver = 2;
  for (uint8_t i = 0; i < EXT_SLOTS; i++) {
    const SensorSlotCfg& s = g_cfg.slot[i];
    ExtSlotVal& o = tx.s[i];
    o.type = s.type;
    o.pin = s.pin;
    o.v1 = VAL_NA;
    o.v2 = VAL_NA;

    switch (s.type) {
      case SENS_LDR:
      case SENS_SOIL:
        o.v1 = (int16_t)analogRead(s.pin);
        break;

      case SENS_FLOW:
        o.v1 = (int16_t)hz_x10(i, windowMs);
        break;

      case SENS_TDS: {
        // DFRobot TDS: temperature compensated voltage -> ppm
        float v = volts_(s.pin) / (1.0f + 0.02f * (TDS_TEMP_C - 25.0f));
        float ppm = (133.42f * v * v * v - 255.86f * v * v + 857.39f * v) * 0.5f;
        if (ppm < 0) ppm = 0;
        if (ppm > 30000) ppm = 30000;
        o.v1 = (int16_t)lroundf(ppm);
      } break;

      case SENS_PH: {
        float ph = PH_SLOPE * volts_(s.pin) + PH_OFFSET;
        if (ph < 0) ph = 0;
        if (ph > 14) ph = 14;
        o.v1 = (int16_t)lroundf(ph * 100.0f);
      } break;

      case SENS_DHT11:
        if (g_dht[i]) {
          float t = g_dht[i]->readTemperature();
          float h = g_dht[i]->readHumidity();
          if (isnan(t) || isnan(h)) {           // first read after sleep often fails: retry once
            delay(1100);
            t = g_dht[i]->readTemperature(false, true);
            h = g_dht[i]->readHumidity();
          }
          if (!isnan(t) && t > -40.0f && t < 85.0f) o.v1 = (int16_t)lroundf(t * 10.0f);
          if (!isnan(h) && h >= 0.0f && h <= 100.0f) o.v2 = (int16_t)lroundf(h * 10.0f);
        }
        break;

      case SENS_PUMP:
      case SENS_VALVE:
        o.v1 = (g_outStates & (1 << i)) ? 1 : 0;
        break;

      default: break;
    }
  }
}

// Handles CMD_SENSOR_CFG / CMD_EXT_OUT addressed to this node (acks included).
// Returns true if the packet was one of those.
static bool handle_packet(const Link::RxPacket& rx, uint8_t selfId, const char* dev) {
  if (rx.h.src != GW_ID || rx.h.dst != selfId) return false;

  if (rx.h.type == CMD_SENSOR_CFG) {
    if (rx.h.len != sizeof(CmdSensorCfg)) return true;
    CmdSensorCfg c;
    memcpy(&c, rx.plain, sizeof(c));

    AckSensorCfg a;
    a.ver = SensorMap::CFG_VER;
    a.ok = set_config(c) ? 1 : 0;

    bool ok = Link::send_enc(selfId, GW_ID, ACK_CMD, 2, rx.h.msgId, (uint8_t*)&a, sizeof(a), true);
    Link::log_tx(dev, "ACK_SENSOR", ok, selfId, GW_ID, ACK_CMD, 2, rx.h.msgId);
    Serial.print(dev); Serial.print(" SENSOR cfg "); Serial.println(a.ok ? "applied" : "rejected");
    return true;
  }

  if (rx.h.type == CMD_EXT_OUT) {
    if (rx.h.len != sizeof(CmdExtOut)) return true;
    CmdExtOut c;
    memcpy(&c, rx.plain, sizeof(c));

    AckExtOut a;
    a.ver = 1;
    a.states = set_outputs(c.states);
    a.outMask = output_mask();

    bool ok = Link::send_enc(selfId, GW_ID, ACK_CMD, 2, rx.h.msgId, (uint8_t*)&a, sizeof(a), true);
    Link::log_tx(dev, "ACK_EXT_OUT", ok, selfId, GW_ID, ACK_CMD, 2, rx.h.msgId);
    Serial.print(dev); Serial.print(" OUTPUTS states=0x"); Serial.println(a.states, HEX);
    return true;
  }

  return false;
}

} // namespace ExtSensors
