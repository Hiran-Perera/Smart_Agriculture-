#pragma once
#include <Arduino.h>
#include <Preferences.h>
#include "esp_sleep.h"

namespace PowerPolicy {
static Preferences prefs;
static inline void begin(const char *ns) {
  prefs.begin(ns, false);
}

static inline uint16_t get_period_s(uint16_t defS) {
  return prefs.getUShort("period_s", defS);
}

static inline uint16_t get_rxwin_ms(uint16_t defMs) {
  return prefs.getUShort("rxwin_ms", defMs);
}

static inline void set_period_s(uint16_t s) {
  prefs.putUShort("period_s", s);
}

static inline void set_rxwin_ms(uint16_t ms) {
  prefs.putUShort("rxwin_ms", ms);
}

static inline void sleep_ms(uint32_t ms) {
  esp_sleep_enable_timer_wakeup((uint64_t)ms * 1000ULL);
  esp_light_sleep_start();
}

} // namespace PowerPolicy