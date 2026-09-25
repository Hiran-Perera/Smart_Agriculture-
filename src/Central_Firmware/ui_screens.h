#pragma once
#include <Arduino.h>
#include <Adafruit_ST77xx.h>
#include "tft_driver.h"
#include "data_model.h"
#include "ui_state.h"

struct UiDrawContext {
  uint32_t nowMs = 0;
  TftDriver* tft = nullptr;
  const DataModel* data = nullptr;
  const UiState* ui = nullptr;
};

static inline int16_t transition_offset(const UiState& ui, uint32_t nowMs, int16_t w) {
  if (!ui.tr.active) return 0;
  uint32_t dt = nowMs - ui.tr.startMs;
  float t = (float)dt / (float)ui.tr.durationMs;
  if (t < 0) t = 0;
  if (t > 1) t = 1;
  float u = 1.0f - (1.0f - t) * (1.0f - t) * (1.0f - t);
  return (int16_t)(u * (float)w);
}

static inline void draw_center_bg(TftDriver& t, int16_t xoff, int16_t y, const char* s, uint16_t fg) {
  int16_t x = xoff + t.centerTextX(s, 1);
  t.drawText(x, y, s, fg, ST77XX_BLACK);
}

static inline bool is_offline(uint32_t nowMs, uint32_t lastSeenMs, uint32_t timeoutMs) {
  if (lastSeenMs == 0) return true;
  return (uint32_t)(nowMs - lastSeenMs) > timeoutMs;
}

// Offline timeout based on configured period + rx window + margin.
// This prevents "OFFLINE" showing after 8 seconds when your period is 10/30/60s.
static inline uint32_t offline_timeout_ms_A(const DataModel& d) {
  uint32_t p = (uint32_t)d.cfg.cfgA_period_s * 1000UL;
  uint32_t w = (uint32_t)d.cfg.cfgA_rxwin_ms;
  return p + (2 * w) + 3000;
}

static inline uint32_t offline_timeout_ms_B(const DataModel& d) {
  uint32_t p = (uint32_t)d.cfg.cfgB_period_s * 1000UL;
  uint32_t w = (uint32_t)d.cfg.cfgB_rxwin_ms;
  return p + (2 * w) + 3000;
}

static inline void ui_draw_one(const UiDrawContext& c, UiPage p, int16_t xoff);

static inline void ui_draw(const UiDrawContext& c) {
  auto& t = *c.tft;
  int16_t W = t.width();

  bool full = c.ui->dirty || c.ui->tr.active;

  t.beginFrame();

  if (c.ui->tr.active) {
    t.fillScreen(ST77XX_BLACK);
    int16_t off = transition_offset(*c.ui, c.nowMs, W);
    ui_draw_one(c, c.ui->tr.from, -off);
    ui_draw_one(c, c.ui->tr.to, (W - off));
    t.endFrame();
    return;
  }

  if (full) t.fillScreen(ST77XX_BLACK);

  ui_draw_one(c, c.ui->page, 0);
  t.endFrame();
}

static inline void draw_home(const UiDrawContext& c, int16_t xoff) {
  auto& t = *c.tft;
  const auto& u = *c.ui;

  t.drawHeader(xoff, "GATEWAY");

  const int16_t cardW = 152;
  const int16_t x = xoff + (t.width() - cardW) / 2;

  const char* items[4] = { "NODE A", "NODE B", "PUMP", "SETTINGS" };

  const int16_t y0 = 28;
  const int16_t itemH = 18;
  const int16_t gap = 6;

  for (int i = 0; i < 4; i++) {
    t.drawListItem(x, (int16_t)(y0 + i * (itemH + gap)), cardW, itemH, items[i], (u.homeIndex == i));
  }
}

static inline void draw_nodeA(const UiDrawContext& c, int16_t xoff) {
  auto& t = *c.tft;
  const auto& d = *c.data;

  t.drawHeader(xoff, "NODE A");

  const int16_t cardW = 152;
  const int16_t x = xoff + (t.width() - cardW) / 2;
  t.drawCard(x, 28, cardW, 92);

  const bool off = is_offline(c.nowMs, d.a.lastSeenMs, offline_timeout_ms_A(d));
  if (!d.a.valid || off) {
    draw_center_bg(t, xoff, 66, "OFFLINE", ST77XX_LIGHTGREY);
    return;
  }

  char buf[48];
  t.setTextSmall();

  snprintf(buf, sizeof(buf), "LIGHT %u", (unsigned)d.a.light_raw);
  draw_center_bg(t, xoff, 40, buf, ST77XX_WHITE);

  snprintf(buf, sizeof(buf), "BATT %u mV", (unsigned)d.a.batt_mV);
  draw_center_bg(t, xoff, 54, buf, ST77XX_WHITE);

  if (d.a.tempC_x100 == (int16_t)0x8000) snprintf(buf, sizeof(buf), "TEMP NA");
  else snprintf(buf, sizeof(buf), "TEMP %.2fC", d.a.tempC_x100 / 100.0f);
  draw_center_bg(t, xoff, 68, buf, ST77XX_WHITE);

  if (d.a.hum_x100 == (int16_t)0x8000) snprintf(buf, sizeof(buf), "HUM NA");
  else snprintf(buf, sizeof(buf), "HUM %.2f%%", d.a.hum_x100 / 100.0f);
  draw_center_bg(t, xoff, 82, buf, ST77XX_WHITE);

  uint32_t age = (uint32_t)((c.nowMs - d.a.lastSeenMs) / 1000);
  snprintf(buf, sizeof(buf), "LAST %lus", (unsigned long)age);
  draw_center_bg(t, xoff, 104, buf, ST77XX_GREEN);
}

static inline void draw_nodeB(const UiDrawContext& c, int16_t xoff) {
  auto& t = *c.tft;
  const auto& d = *c.data;

  t.drawHeader(xoff, "NODE B");

  const int16_t cardW = 152;
  const int16_t x = xoff + (t.width() - cardW) / 2;
  t.drawCard(x, 28, cardW, 92);

  const bool off = is_offline(c.nowMs, d.b.lastSeenMs, offline_timeout_ms_B(d));
  if (!d.b.valid || off) {
    draw_center_bg(t, xoff, 66, "OFFLINE", ST77XX_LIGHTGREY);
    return;
  }

  char buf[48];
  t.setTextSmall();

  snprintf(buf, sizeof(buf), "SOIL %u", (unsigned)d.b.soil_raw);
  draw_center_bg(t, xoff, 40, buf, ST77XX_WHITE);

  snprintf(buf, sizeof(buf), "FLOW %u.%u HZ", (unsigned)(d.b.flow_hz_x10 / 10), (unsigned)(d.b.flow_hz_x10 % 10));
  draw_center_bg(t, xoff, 54, buf, ST77XX_WHITE);

  snprintf(buf, sizeof(buf), "BATT %u mV", (unsigned)d.b.batt_mV);
  draw_center_bg(t, xoff, 68, buf, ST77XX_WHITE);

  snprintf(buf, sizeof(buf), "RELAY %s", d.b.relay_state ? "ON" : "OFF");
  draw_center_bg(t, xoff, 82, buf, d.b.relay_state ? ST77XX_GREEN : ST77XX_RED);

  uint32_t age = (uint32_t)((c.nowMs - d.b.lastSeenMs) / 1000);
  snprintf(buf, sizeof(buf), "LAST %lus", (unsigned long)age);
  draw_center_bg(t, xoff, 104, buf, ST77XX_GREEN);
}

static inline void draw_relay(const UiDrawContext& c, int16_t xoff) {
  auto& t = *c.tft;
  const auto& d = *c.data;
  const auto& u = *c.ui;

  t.drawHeader(xoff, "PUMP");

  const int16_t cardW = 152;
  const int16_t x = xoff + (t.width() - cardW) / 2;

  char buf[48];

  snprintf(buf, sizeof(buf), "MANUAL %s", d.relay_target_on ? "ON" : "OFF");
  t.drawListItem(x, 28, cardW, 18, buf, (u.relayIndex == 0));

  t.drawListItem(x, 52, cardW, 18, "SEND", (u.relayIndex == 1));

  snprintf(buf, sizeof(buf), "AUTO %s", d.autoPump.enabled ? "ON" : "OFF");
  t.drawListItem(x, 76, cardW, 18, buf, (u.relayIndex == 2));

  snprintf(buf, sizeof(buf), "ON TH %u", (unsigned)d.autoPump.soil_on_threshold);
  t.drawListItem(x, 100, cardW, 18, buf, (u.relayIndex == 3));

  if (u.relayEdit) t.drawTextNoBg(xoff + 126, 10, "EDIT", ST77XX_YELLOW);
}

static inline void draw_settings1(const UiDrawContext& c, int16_t xoff) {
  auto& t = *c.tft;
  const auto& d = *c.data;
  const auto& u = *c.ui;

  t.drawHeader(xoff, "SETTINGS A");

  const int16_t cardW = 152;
  const int16_t x = xoff + (t.width() - cardW) / 2;

  char buf[48];

  snprintf(buf, sizeof(buf), "A PERIOD %us", (unsigned)d.cfg.cfgA_period_s);
  t.drawListItem(x, 28, cardW, 18, buf, (u.settingsIndex1 == 0));

  snprintf(buf, sizeof(buf), "A RXWIN %ums", (unsigned)d.cfg.cfgA_rxwin_ms);
  t.drawListItem(x, 52, cardW, 18, buf, (u.settingsIndex1 == 1));

  t.drawListItem(x, 76, cardW, 18, "SEND A", (u.settingsIndex1 == 2));

  draw_center_bg(t, xoff, 110, "ROTATE FOR NEXT", ST77XX_LIGHTGREY);
  if (u.settingsEdit1) t.drawTextNoBg(xoff + 126, 10, "EDIT", ST77XX_YELLOW);
}

static inline void draw_settings2(const UiDrawContext& c, int16_t xoff) {
  auto& t = *c.tft;
  const auto& d = *c.data;
  const auto& u = *c.ui;

  t.drawHeader(xoff, "SETTINGS B");

  const int16_t cardW = 152;
  const int16_t x = xoff + (t.width() - cardW) / 2;

  char buf[48];

  snprintf(buf, sizeof(buf), "B PERIOD %us", (unsigned)d.cfg.cfgB_period_s);
  t.drawListItem(x, 28, cardW, 18, buf, (u.settingsIndex2 == 0));

  snprintf(buf, sizeof(buf), "B RXWIN %ums", (unsigned)d.cfg.cfgB_rxwin_ms);
  t.drawListItem(x, 52, cardW, 18, buf, (u.settingsIndex2 == 1));

  t.drawListItem(x, 76, cardW, 18, "SEND B", (u.settingsIndex2 == 2));

  draw_center_bg(t, xoff, 110, "ROTATE FOR HOME", ST77XX_LIGHTGREY);
  if (u.settingsEdit2) t.drawTextNoBg(xoff + 126, 10, "EDIT", ST77XX_YELLOW);
}

static inline void ui_draw_one(const UiDrawContext& c, UiPage p, int16_t xoff) {
  switch (p) {
    case UiPage::Home:          draw_home(c, xoff); break;
    case UiPage::NodeA_Details: draw_nodeA(c, xoff); break;
    case UiPage::NodeB_Details: draw_nodeB(c, xoff); break;
    case UiPage::Relay:         draw_relay(c, xoff); break;
    case UiPage::Settings1:     draw_settings1(c, xoff); break;
    case UiPage::Settings2:     draw_settings2(c, xoff); break;
    default: break;
  }
}