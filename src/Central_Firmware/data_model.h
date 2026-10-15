#pragma once
#include <Arduino.h>
#include "payloads.h"
#include "sensor_map.h"

static inline const char* yesno(bool v){ return v ? "ON":"OFF"; }

struct NodeRadio { int16_t rssi=0; float snr=0; };

// Latest value of a user assigned (EXT) sensor as reported by the node.
struct ExtVal {
  uint8_t type = 0;      // Payload::SensorType
  uint8_t pin = 0xFF;
  int16_t v1 = Payload::VAL_NA;
  int16_t v2 = Payload::VAL_NA;
  bool    valid = false;
};

struct NodeAState {
  uint16_t light_raw=0;
  uint16_t batt_mV=0;

  // DHT values (0x8000 means NA)
  int16_t tempC_x100 = (int16_t)0x8000;
  int16_t hum_x100   = (int16_t)0x8000;

  uint32_t lastSeenMs=0;
  uint32_t intervalMs=0;     // measured time between telemetry packets (smoothed)
  NodeRadio radio;
  bool valid=false;

  int rssi = 0;
  float snr = 0.0f;

  ExtVal ext[Payload::EXT_SLOTS];
  uint32_t extMs = 0;
};

struct NodeBState {
  uint16_t soil_raw=0;
  uint16_t flow_hz_x10=0;
  uint16_t batt_mV=0;
  bool relay_state=false;
  uint32_t lastSeenMs=0;
  uint32_t intervalMs=0;     // measured time between telemetry packets (smoothed)
  NodeRadio radio;
  bool valid=false;

  int rssi = 0;
  float snr = 0.0f;

  ExtVal ext[Payload::EXT_SLOTS];
  uint32_t extMs = 0;
};

struct AckState {
  bool pending=false;
  bool ok=false;
  uint8_t retriesUsed=0;
  uint32_t lastSendMs=0;
  uint32_t lastAckMs=0;
};

// Sleep timing as edited on the Node Timing page (defaults = node firmware defaults).
struct ConfigState {
  uint16_t cfgA_period_s = 30;
  uint16_t cfgA_rxwin_ms = 1200;
  uint16_t cfgB_period_s = 30;
  uint16_t cfgB_rxwin_ms = 2500;
};

// Sleep timing a node confirmed it runs (from its ACK, kept in NVS).
struct NodeTiming {
  uint16_t period_s;
  uint16_t rxwin_ms;
};

struct AutoPumpSettings {
  bool enabled=false;

  uint16_t soil_on_threshold=2800;
  uint16_t soil_off_threshold=2200;

  uint16_t cooldown_sec=20;
  uint16_t stale_timeout_sec=60;
  uint16_t manual_override_sec=60;

  uint32_t last_auto_action_ms=0;
  uint32_t last_manual_ms=0;

  bool auto_request_pending=false;
  bool auto_target_on=false;
};

struct DataModel {
  NodeAState a;
  NodeBState b;

  bool relay_target_on=false;

  AckState relayAck;
  AckState cfgAAck;
  AckState cfgBAck;

  ConfigState cfg;
  NodeTiming applied[2] = { {30, 1200}, {30, 2500} };
  bool timingDirty = false;   // applied[] changed, save to NVS
  AutoPumpSettings autoPump;

  // Sensor assignment, index 0 = Node A, 1 = Node B.
  // sensCfg is what the Sensors page edits, sensSent what the node was last sent.
  Payload::CmdSensorCfg sensCfg[2];
  Payload::CmdSensorCfg sensSent[2];
  AckState sensAck[2];

  // Pump / valve outputs on EXT slots: requested states (bit i = slot i on)
  uint8_t outTarget[2] = {0, 0};
  AckState outAck[2];

  const ExtVal* ext(uint8_t n) const { return n ? b.ext : a.ext; }

  void reset() { *this = DataModel(); }
};

static inline uint32_t age_s(uint32_t nowMs, uint32_t lastSeenMs) {
  if (lastSeenMs == 0) return 0xFFFFFFFFu;
  return (nowMs - lastSeenMs) / 1000u;
}