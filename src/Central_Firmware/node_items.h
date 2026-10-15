#pragma once
#include <Arduino.h>
#include "data_model.h"
#include "tft_driver.h"

// One reading shown on a node page (the page cycles through them).
struct NodeItem {
  char label[16];   // "TEMPERATURE"
  char value[12];   // "24.5"
  char unit[7];     // "C"
  char sub[26];     // "DHT11  G8"
  uint16_t dot;     // sensor colour (label dot)
  uint16_t color;   // value colour (white, or state colour for ON/OFF)
};

static const uint8_t NODE_ITEMS_MAX = 3 + Payload::EXT_SLOTS * 2;
static const float FLOW_HZ_PER_LPM = 7.5f;   // YF-S201 style flow sensor

static inline void item_set_(NodeItem& it, const char* label, const char* sub, uint16_t color = Col::ACCENT) {
  snprintf(it.label, sizeof(it.label), "%s", label);
  snprintf(it.sub, sizeof(it.sub), "%s", sub);
  it.value[0] = 0;
  it.unit[0] = 0;
  it.dot = color;
  it.color = Col::TEXT;
}

static inline uint8_t pct_of_raw(int32_t raw) {
  if (raw < 0) raw = 0;
  if (raw > 4095) raw = 4095;
  return (uint8_t)(raw * 100 / 4095);
}

static inline void item_onoff_(NodeItem& it, int16_t v) {
  if (v == Payload::VAL_NA) { snprintf(it.value, sizeof(it.value), "--"); it.color = Col::MUTED; return; }
  snprintf(it.value, sizeof(it.value), v ? "ON" : "OFF");
  it.color = v ? Col::ACCENT : Col::MUTED;
}

// Adds the item(s) for one EXT slot. live may be null (no data yet).
static inline uint8_t ext_items_(const Payload::SensorSlotCfg& s, const ExtVal* live, NodeItem* out, uint8_t room) {
  if (s.type == Payload::SENS_NONE || room == 0) return 0;

  char where[26];
  snprintf(where, sizeof(where), "%s  G%u", SensorMap::type_name(s.type), (unsigned)s.pin);
  const bool have = live && live->valid && live->type == s.type && live->pin == s.pin;
  const int16_t v1 = have ? live->v1 : Payload::VAL_NA;
  const int16_t v2 = have ? live->v2 : Payload::VAL_NA;
  NodeItem& it = out[0];
  char buf[26];

  switch (s.type) {
    case Payload::SENS_DHT11: {
      item_set_(it, "TEMPERATURE", where, Col::BAD);
      if (v1 == Payload::VAL_NA) snprintf(it.value, sizeof(it.value), "--");
      else snprintf(it.value, sizeof(it.value), "%.1f", v1 / 10.0f);
      snprintf(it.unit, sizeof(it.unit), "C");
      if (room < 2) return 1;
      NodeItem& h = out[1];
      item_set_(h, "HUMIDITY", where, Col::INFO);
      if (v2 == Payload::VAL_NA) snprintf(h.value, sizeof(h.value), "--");
      else snprintf(h.value, sizeof(h.value), "%.0f", v2 / 10.0f);
      snprintf(h.unit, sizeof(h.unit), "%%");
      return 2;
    }
    case Payload::SENS_LDR:
      snprintf(buf, sizeof(buf), "%s  raw %d", where, v1 == Payload::VAL_NA ? 0 : v1);
      item_set_(it, "LIGHT", v1 == Payload::VAL_NA ? where : buf, Col::WARN);
      if (v1 == Payload::VAL_NA) snprintf(it.value, sizeof(it.value), "--");
      else { snprintf(it.value, sizeof(it.value), "%u", pct_of_raw(v1)); snprintf(it.unit, sizeof(it.unit), "%%"); }
      return 1;
    case Payload::SENS_SOIL:
      snprintf(buf, sizeof(buf), "%s  raw %d", where, v1 == Payload::VAL_NA ? 0 : v1);
      item_set_(it, "SOIL MOISTURE", v1 == Payload::VAL_NA ? where : buf, Col::ACCENT);
      if (v1 == Payload::VAL_NA) snprintf(it.value, sizeof(it.value), "--");
      else { snprintf(it.value, sizeof(it.value), "%u", 100 - pct_of_raw(v1)); snprintf(it.unit, sizeof(it.unit), "%%"); }
      return 1;
    case Payload::SENS_FLOW:
      snprintf(buf, sizeof(buf), "%s  %.1f Hz", where, v1 == Payload::VAL_NA ? 0.0f : v1 / 10.0f);
      item_set_(it, "FLOW RATE", v1 == Payload::VAL_NA ? where : buf, Col::INFO);
      if (v1 == Payload::VAL_NA) snprintf(it.value, sizeof(it.value), "--");
      else snprintf(it.value, sizeof(it.value), "%.1f", (v1 / 10.0f) / FLOW_HZ_PER_LPM);
      snprintf(it.unit, sizeof(it.unit), "L/min");
      return 1;
    case Payload::SENS_TDS:
      item_set_(it, "TDS", where, Col::INFO);
      if (v1 == Payload::VAL_NA) snprintf(it.value, sizeof(it.value), "--");
      else snprintf(it.value, sizeof(it.value), "%d", v1);
      snprintf(it.unit, sizeof(it.unit), "ppm");
      return 1;
    case Payload::SENS_PH:
      item_set_(it, "PH", where, Col::WARN);
      if (v1 == Payload::VAL_NA) snprintf(it.value, sizeof(it.value), "--");
      else snprintf(it.value, sizeof(it.value), "%.2f", v1 / 100.0f);
      snprintf(it.unit, sizeof(it.unit), "pH");
      return 1;
    case Payload::SENS_PUMP:
      item_set_(it, "PUMP", where);
      item_onoff_(it, v1);
      return 1;
    case Payload::SENS_VALVE:
      item_set_(it, "VALVE", where);
      item_onoff_(it, v1);
      return 1;
    default:
      return 0;
  }
}

// Same count node_items() would produce, without formatting anything.
static inline uint8_t node_item_count(const DataModel& d, uint8_t n) {
  uint8_t c = n ? 2 : 3;
  for (uint8_t i = 0; i < Payload::EXT_SLOTS; i++) {
    uint8_t t = d.sensSent[n].slot[i].type;
    if (t == Payload::SENS_DHT11) c += 2;
    else if (t != Payload::SENS_NONE) c += 1;
  }
  return c > NODE_ITEMS_MAX ? NODE_ITEMS_MAX : c;
}

// n: 0 = Node A, 1 = Node B. Returns the number of items written.
static inline uint8_t node_items(const DataModel& d, uint8_t n, NodeItem* out, uint8_t max) {
  uint8_t c = 0;
  char buf[26];

  if (n == 0) {
    if (c < max) {
      NodeItem& it = out[c++];
      snprintf(buf, sizeof(buf), "LDR  G3  raw %u", (unsigned)d.a.light_raw);
      item_set_(it, "LIGHT", buf, Col::WARN);
      snprintf(it.value, sizeof(it.value), "%u", pct_of_raw(d.a.light_raw));
      snprintf(it.unit, sizeof(it.unit), "%%");
    }
    if (c < max) {
      NodeItem& it = out[c++];
      item_set_(it, "TEMPERATURE", "DHT11  G2", Col::BAD);
      if (d.a.tempC_x100 == Payload::VAL_NA) snprintf(it.value, sizeof(it.value), "--");
      else snprintf(it.value, sizeof(it.value), "%.1f", d.a.tempC_x100 / 100.0f);
      snprintf(it.unit, sizeof(it.unit), "C");
    }
    if (c < max) {
      NodeItem& it = out[c++];
      item_set_(it, "HUMIDITY", "DHT11  G2", Col::INFO);
      if (d.a.hum_x100 == Payload::VAL_NA) snprintf(it.value, sizeof(it.value), "--");
      else snprintf(it.value, sizeof(it.value), "%.0f", d.a.hum_x100 / 100.0f);
      snprintf(it.unit, sizeof(it.unit), "%%");
    }
  } else {
    if (c < max) {
      NodeItem& it = out[c++];
      snprintf(buf, sizeof(buf), "SOIL  G1  raw %u", (unsigned)d.b.soil_raw);
      item_set_(it, "SOIL MOISTURE", buf, Col::ACCENT);
      snprintf(it.value, sizeof(it.value), "%u", 100 - pct_of_raw(d.b.soil_raw));
      snprintf(it.unit, sizeof(it.unit), "%%");
    }
    if (c < max) {
      NodeItem& it = out[c++];
      item_set_(it, "MAIN PUMP", d.autoPump.enabled ? "RELAY G2  AUTO" : "RELAY G2  MANUAL");
      item_onoff_(it, d.b.relay_state ? 1 : 0);
    }
  }

  const ExtVal* live = d.ext(n);
  for (uint8_t i = 0; i < Payload::EXT_SLOTS && c < max; i++) {
    c += ext_items_(d.sensSent[n].slot[i], &live[i], &out[c], (uint8_t)(max - c));
  }
  return c;
}
