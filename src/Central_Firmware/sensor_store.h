#pragma once
#include <Arduino.h>
#include <Preferences.h>
#include "data_model.h"

// Persists the sensor assignment sent to each node and the sleep timing each node confirmed.
class SensorStore {
public:
  void begin(DataModel& m) {
    _prefs.begin("gw_sens", false);
    load_(m, 0, "cfgA2", Payload::NODE_A);
    load_(m, 1, "cfgB2", Payload::NODE_B);
    loadTiming_(m, 0, "timA");
    loadTiming_(m, 1, "timB");
  }

  void save(const DataModel& m, uint8_t n) {
    _prefs.putBytes(n ? "cfgB2" : "cfgA2", &m.sensCfg[n], sizeof(Payload::CmdSensorCfg));
  }

  void saveTiming(const DataModel& m) {
    _prefs.putBytes("timA", &m.applied[0], sizeof(NodeTiming));
    _prefs.putBytes("timB", &m.applied[1], sizeof(NodeTiming));
  }

private:
  Preferences _prefs;

  void load_(DataModel& m, uint8_t n, const char* key, uint8_t nodeId) {
    Payload::CmdSensorCfg& c = m.sensCfg[n];
    bool ok = _prefs.isKey(key) && _prefs.getBytesLength(key) == sizeof(c);
    if (ok) _prefs.getBytes(key, &c, sizeof(c));
    if (!ok || !SensorMap::valid(nodeId, c)) SensorMap::defaults(nodeId, c);
    m.sensSent[n] = c;
  }

  // The Node Timing page starts from what the node last confirmed.
  void loadTiming_(DataModel& m, uint8_t n, const char* key) {
    NodeTiming t;
    if (_prefs.isKey(key) && _prefs.getBytesLength(key) == sizeof(t)) {
      _prefs.getBytes(key, &t, sizeof(t));
      if (t.period_s >= 5 && t.period_s <= 3600) m.applied[n] = t;
    }
    if (n == 0) { m.cfg.cfgA_period_s = m.applied[0].period_s; m.cfg.cfgA_rxwin_ms = m.applied[0].rxwin_ms; }
    else        { m.cfg.cfgB_period_s = m.applied[1].period_s; m.cfg.cfgB_rxwin_ms = m.applied[1].rxwin_ms; }
  }
};
