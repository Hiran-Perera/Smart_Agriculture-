#pragma once
#include <Arduino.h>
#include <Preferences.h>
#include "data_model.h"

class AutoPumpAutomation {
public:
  void begin(DataModel& m) {
    _prefs.begin("gw_auto", false);

    m.autoPump.enabled = _prefs.getBool("en", false);
    m.autoPump.soil_on_threshold  = _prefs.getUShort("on", 2800);
    m.autoPump.soil_off_threshold = _prefs.getUShort("off", 2200);
    m.autoPump.cooldown_sec       = _prefs.getUShort("cool", 20);
    m.autoPump.stale_timeout_sec  = _prefs.getUShort("stale", 60);
    m.autoPump.manual_override_sec= _prefs.getUShort("man", 60);

    enforce_hysteresis(m.autoPump);

    m.autoPump.last_auto_action_ms = 0;
    m.autoPump.last_manual_ms = 0;
    m.autoPump.auto_request_pending = false;
    m.autoPump.auto_target_on = false;

    Serial.println("AUTO: loaded NVS");
  }

  void save(const DataModel& m) {
    _prefs.putBool("en", m.autoPump.enabled);
    _prefs.putUShort("on", m.autoPump.soil_on_threshold);
    _prefs.putUShort("off", m.autoPump.soil_off_threshold);
    _prefs.putUShort("cool", m.autoPump.cooldown_sec);
    _prefs.putUShort("stale", m.autoPump.stale_timeout_sec);
    _prefs.putUShort("man", m.autoPump.manual_override_sec);
    Serial.println("AUTO: saved NVS");
  }

  void mark_manual_override(DataModel& m, uint32_t nowMs) {
    m.autoPump.last_manual_ms = nowMs;
  }

  static void enforce_hysteresis(AutoPumpSettings& s) {
    if (s.soil_on_threshold <= s.soil_off_threshold) {
      s.soil_on_threshold = s.soil_off_threshold + 50;
    }
  }

  bool tick(DataModel& m, uint32_t nowMs) {
    m.autoPump.auto_request_pending = false;

    if (!m.autoPump.enabled) return false;
    if (!m.b.valid) return false;

    uint32_t ageB = age_s(nowMs, m.b.lastSeenMs);
    if (ageB == 0xFFFFFFFFu || ageB > m.autoPump.stale_timeout_sec) return false;

    if (m.autoPump.last_manual_ms != 0) {
      uint32_t manAge = (nowMs - m.autoPump.last_manual_ms) / 1000u;
      if (manAge < m.autoPump.manual_override_sec) return false;
    }

    if (m.autoPump.last_auto_action_ms != 0) {
      uint32_t cdAge = (nowMs - m.autoPump.last_auto_action_ms) / 1000u;
      if (cdAge < m.autoPump.cooldown_sec) return false;
    }

    enforce_hysteresis(m.autoPump);

    if (!m.b.relay_state && m.b.soil_raw >= m.autoPump.soil_on_threshold) {
      request(m, nowMs, true);
      return true;
    }

    if (m.b.relay_state && m.b.soil_raw <= m.autoPump.soil_off_threshold) {
      request(m, nowMs, false);
      return true;
    }

    return false;
  }

private:
  Preferences _prefs;

  void request(DataModel& m, uint32_t nowMs, bool on) {
    m.autoPump.auto_request_pending = true;
    m.autoPump.auto_target_on = on;
    m.autoPump.last_auto_action_ms = nowMs;

    Serial.print("AUTO: request ");
    Serial.println(on ? "ON" : "OFF");
  }
};