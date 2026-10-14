#pragma once
#include <Arduino.h>
#include <Preferences.h>
#include "data_model.h"

// Persists the sensor assignment the gateway last sent to each node.
class SensorStore {
public:
  void begin(DataModel& m) {
    _prefs.begin("gw_sens", false);
    load_(m, 0, "cfgA", Payload::NODE_A);
    load_(m, 1, "cfgB", Payload::NODE_B);
  }

  void save(const DataModel& m, uint8_t n) {
    _prefs.putBytes(n ? "cfgB" : "cfgA", &m.sensCfg[n], sizeof(Payload::CmdSensorCfg));
  }

private:
  Preferences _prefs;

  void load_(DataModel& m, uint8_t n, const char* key, uint8_t nodeId) {
    Payload::CmdSensorCfg& c = m.sensCfg[n];
    bool ok = _prefs.getBytesLength(key) == sizeof(c);
    if (ok) _prefs.getBytes(key, &c, sizeof(c));
    if (!ok || c.ver != 1 || !SensorMap::valid(nodeId, c)) SensorMap::defaults(nodeId, c);
  }
};
