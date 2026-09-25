#pragma once
#include <Arduino.h>

static inline const char* yesno(bool v){ return v ? "ON":"OFF"; }

struct NodeRadio { int16_t rssi=0; float snr=0; };

struct NodeAState {
  uint16_t light_raw=0;
  uint16_t batt_mV=0;

  // DHT values (0x8000 means NA)
  int16_t tempC_x100 = (int16_t)0x8000;
  int16_t hum_x100   = (int16_t)0x8000;

  uint32_t lastSeenMs=0;
  NodeRadio radio;
  bool valid=false;

  int rssi = 0;
  float snr = 0.0f;
};

struct NodeBState {
  uint16_t soil_raw=0;
  uint16_t flow_hz_x10=0;
  uint16_t batt_mV=0;
  bool relay_state=false;
  uint32_t lastSeenMs=0;
  NodeRadio radio;
  bool valid=false;

  int rssi = 0;
  float snr = 0.0f;
};

struct AckState {
  bool pending=false;
  bool ok=false;
  uint8_t retriesUsed=0;
  uint32_t lastSendMs=0;
  uint32_t lastAckMs=0;
};

struct ConfigState {
  uint16_t cfgA_period_s = 10;
  uint16_t cfgA_rxwin_ms = 250;
  uint16_t cfgB_period_s = 10;
  uint16_t cfgB_rxwin_ms = 250;
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
  AutoPumpSettings autoPump;

  void reset() { *this = DataModel(); }
};

static inline uint32_t age_s(uint32_t nowMs, uint32_t lastSeenMs) {
  if (lastSeenMs == 0) return 0xFFFFFFFFu;
  return (nowMs - lastSeenMs) / 1000u;
}