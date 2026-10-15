#pragma once
#include <Arduino.h>
#include "tft_driver.h"
#include "data_model.h"
#include "ui_state.h"
#include "ui_logic.h"
#include "node_items.h"

struct UiDrawContext {
  uint32_t nowMs = 0;
  TftDriver* tft = nullptr;
  const DataModel* data = nullptr;
  const UiState* ui = nullptr;
};

// ---------- shared helpers ----------
static inline bool is_offline(uint32_t nowMs, uint32_t lastSeenMs, uint32_t timeoutMs) {
  if (lastSeenMs == 0) return true;
  return (uint32_t)(nowMs - lastSeenMs) > timeoutMs;
}

// A node is offline only when its next packet is overdue: one full wake cycle
// (sleep period + listen window + sensing/ack time, or the measured gap if longer)
// plus a grace margin. Sleeping between packets is normal and never shows offline.
static inline uint32_t offline_timeout_ms(const DataModel& d, uint8_t n) {
  const NodeTiming& t = d.applied[n];
  uint32_t cycle = (uint32_t)t.period_s * 1000UL + t.rxwin_ms + 2500UL;
  const uint32_t measured = n ? d.b.intervalMs : d.a.intervalMs;
  if (measured > cycle) cycle = measured;
  return cycle + 5000UL;
}
static inline uint32_t offline_timeout_ms_A(const DataModel& d) { return offline_timeout_ms(d, 0); }
static inline uint32_t offline_timeout_ms_B(const DataModel& d) { return offline_timeout_ms(d, 1); }

static inline uint32_t node_last_seen(const DataModel& d, uint8_t n) { return n ? d.b.lastSeenMs : d.a.lastSeenMs; }

static inline void fmt_age(char* buf, size_t n, uint32_t ms) {
  uint32_t s = ms / 1000;
  if (s < 60)        snprintf(buf, n, "%lus", (unsigned long)s);
  else if (s < 3600) snprintf(buf, n, "%lum", (unsigned long)(s / 60));
  else               snprintf(buf, n, "%luh", (unsigned long)(s / 3600));
}

// Status text for a command that goes to a node on its next wake-up.
static inline const char* ack_text(const AckState& a, uint16_t& col) {
  if (a.pending)        { col = Col::WARN;   return "SENDING"; }
  if (a.lastSendMs == 0){ col = Col::MUTED;  return ""; }
  if (a.ok)             { col = Col::ACCENT; return "DONE"; }
  col = Col::BAD;
  return "FAILED";
}

// ---------- scrolling list ----------
struct Row {
  char left[24];
  char right[14];
  bool editL = false;
  bool editR = false;
  uint16_t rcol = Col::MUTED;
};

static const int16_t LIST_X = 4, LIST_W = 148, LIST_Y0 = 22, ROW_H = 18, ROW_STEP = 21;
static const int8_t  LIST_VISIBLE = 5;

static inline void row_set(Row& r, const char* l, const char* rt = "", uint16_t rc = Col::MUTED) {
  snprintf(r.left, sizeof(r.left), "%s", l);
  snprintf(r.right, sizeof(r.right), "%s", rt);
  r.editL = r.editR = false;
  r.rcol = rc;
}

static inline void draw_list(TftDriver& t, int16_t xoff, const Row* rows, int8_t n, int8_t sel) {
  int8_t first = sel - 2;
  if (first > n - LIST_VISIBLE) first = n - LIST_VISIBLE;
  if (first < 0) first = 0;

  for (int8_t r = first; r < n && r < first + LIST_VISIBLE; r++) {
    const Row& R = rows[r];
    t.drawRow(xoff + LIST_X, (int16_t)(LIST_Y0 + (r - first) * ROW_STEP), LIST_W, ROW_H,
              R.left, R.right, r == sel, R.editL, R.editR, R.rcol);
  }
  t.drawScrollbar(xoff + LIST_X + LIST_W + 2, LIST_Y0, LIST_VISIBLE * ROW_STEP - 3, first, LIST_VISIBLE, n);
}

// ---------- HOME ----------
// Original GATEWAY menu look (plain list, centred labels, green highlight), plus SENSORS.
static inline void draw_home(const UiDrawContext& c, int16_t xoff) {
  auto& t = *c.tft;
  const int16_t W = t.width();

  // header
  t.fillRect(xoff, 0, W, 18, ST77XX_BLACK);
  t.drawFastHLine(xoff, 17, W, ST77XX_GREEN);
  t.drawText(xoff + t.centerTextX("GATEWAY", 1), 3, "GATEWAY", ST77XX_GREEN);

  static const char* items[5] = { "NODE A", "NODE B", "PUMP", "SENSORS", "SETTINGS" };
  const int16_t cardW = 152, itemH = 16, gap = 4, y0 = 24;
  const int16_t x = xoff + (W - cardW) / 2;

  for (int i = 0; i < 5; i++) {
    const bool sel = (c.ui->homeIndex == i);
    const int16_t y = (int16_t)(y0 + i * (itemH + gap));
    const uint16_t bg = sel ? ST77XX_GREEN : ST77XX_BLACK;
    t.fillRect(x, y, cardW, itemH, bg);
    t.drawRect(x, y, cardW, itemH, sel ? ST77XX_GREEN : ST77XX_LIGHTGREY);
    t.drawText(x + (cardW - t.textWidth(items[i])) / 2, y + (itemH - 8) / 2, items[i],
               sel ? ST77XX_BLACK : ST77XX_WHITE);
  }
}

// ---------- NODE PAGES (one reading at a time) ----------
static const int16_t CARD_Y = 22, CARD_H = 76;

// Card layout: [dot LABEL ........ 2/5] / big value + unit (centred on ink) / source line
static inline void draw_item(TftDriver& t, int16_t xoff, int16_t shift, const NodeItem& it, uint8_t idx, uint8_t count) {
  const int16_t W = t.width();
  char buf[8];

  // label row
  const int16_t ly = CARD_Y + 7 + shift;
  t.fillCircle(xoff + 13, ly + 3, 2, it.dot);
  t.drawText(xoff + 19, ly, it.label, Col::MUTED);
  if (count > 1) {
    snprintf(buf, sizeof(buf), "%u/%u", (unsigned)(idx + 1), (unsigned)count);
    t.drawText(xoff + W - 12 - t.textWidth(buf), ly, buf, Col::MUTED);
  }
  t.drawFastHLine(xoff + 10, CARD_Y + 19 + shift, W - 20, Col::BORDER);

  // value + unit, centred on the actual ink width
  const bool na = (strcmp(it.value, "--") == 0);
  const bool deg = (strcmp(it.unit, "C") == 0);
  const int16_t gap = 3, degW = deg ? 6 : 0;

  int16_t ux1 = 0, uw = 0;
  if (it.unit[0] && !na) t.bigBounds(it.unit, 2, ux1, uw);
  const int16_t unitW = uw ? degW + uw : 0;

  uint8_t size = 1;
  int16_t vx1, vw;
  t.bigBounds(it.value, size, vx1, vw);
  if (vw + (unitW ? gap + unitW : 0) > W - 24) { size = 0; t.bigBounds(it.value, size, vx1, vw); }

  const int16_t total = vw + (unitW ? gap + unitW : 0);
  const int16_t left = xoff + (W - total) / 2;
  const int16_t base = CARD_Y + 51 + shift;
  t.drawBig(left - vx1, base, it.value, size, na ? Col::MUTED : it.color);

  if (unitW) {
    int16_t ux = left + vw + gap;
    if (deg) { t.drawCircle(ux + 2, base - 11, 2, Col::MUTED); ux += degW; }
    t.drawBig(ux - ux1, base, it.unit, 2, Col::MUTED);
  }

  // source line (sensor + pin, raw value)
  t.drawText(xoff + (W - t.textWidth(it.sub)) / 2, CARD_Y + 61 + shift, it.sub, Col::MUTED);
}

static inline void draw_node(const UiDrawContext& c, int16_t xoff, uint8_t n) {
  auto& t = *c.tft;
  const auto& d = *c.data;
  const auto& u = *c.ui;
  const int16_t W = t.width();
  char buf[32], age[8];

  const uint32_t seen = node_last_seen(d, n);
  const bool valid = n ? d.b.valid : d.a.valid;
  const bool off = !valid || is_offline(c.nowMs, seen, offline_timeout_ms(d, n));

  t.drawCard(xoff + 4, CARD_Y, W - 8, CARD_H);

  uint8_t count = 0;
  if (off) {
    const char* big = "OFFLINE";
    t.drawBig(xoff + (W - t.bigWidth(big, 0)) / 2, CARD_Y + 45, big, 0, Col::MUTED);
    if (seen == 0) snprintf(buf, sizeof(buf), "waiting for first packet");
    else { fmt_age(age, sizeof(age), c.nowMs - seen); snprintf(buf, sizeof(buf), "last seen %s ago", age); }
    t.drawText(xoff + (W - t.textWidth(buf)) / 2, CARD_Y + 58, buf, Col::MUTED);
  } else {
    NodeItem items[NODE_ITEMS_MAX];
    count = node_items(d, n, items, NODE_ITEMS_MAX);
    const uint8_t cur = count ? (uint8_t)(u.nodeItem % count) : 0;

    if (count && u.itemAnim) {
      const float e = ui_ease(c.nowMs, u.itemAnimStartMs, ITEM_ANIM_MS);
      const int16_t dist = 44;
      const int16_t s = (int16_t)(e * dist);
      draw_item(t, xoff, (int16_t)(-u.itemDir * s), items[u.nodeItemPrev % count], (uint8_t)(u.nodeItemPrev % count), count);
      draw_item(t, xoff, (int16_t)(u.itemDir * (dist - s)), items[cur], cur, count);
    } else if (count) {
      draw_item(t, xoff, 0, items[cur], cur, count);
    }

    // clip the sliding content to the card
    t.fillRect(xoff, 0, W, CARD_Y + 1, Col::BG);
    t.fillRect(xoff, CARD_Y + CARD_H - 1, W, t.height() - (CARD_Y + CARD_H - 1), Col::BG);
    t.fillRect(xoff + 5, CARD_Y + 1, W - 10, 1, Col::PANEL);
    t.fillRect(xoff + 5, CARD_Y + CARD_H - 2, W - 10, 1, Col::PANEL);
    t.drawRoundRect(xoff + 4, CARD_Y, W - 8, CARD_H, 4, Col::BORDER);

    // position dots
    if (count > 1) {
      const int16_t sp = 8;
      int16_t x0 = xoff + (W - (count - 1) * sp) / 2;
      for (uint8_t i = 0; i < count; i++) {
        if (i == cur) t.fillCircle(x0 + i * sp, 104, 2, Col::ACCENT);
        else          t.fillCircle(x0 + i * sp, 104, 1, Col::BORDER);
      }
    }
  }

  t.drawHeader(xoff, n ? "NODE B" : "NODE A");
  t.fillCircle(xoff + 10, 8, 3, seen == 0 ? Col::BORDER : (off ? Col::BAD : Col::ACCENT));

  // footer: battery left, age + signal right
  if (valid) {
    const uint16_t mv = n ? d.b.batt_mV : d.a.batt_mV;
    snprintf(buf, sizeof(buf), "BAT %u.%02uV", mv / 1000, (mv % 1000) / 10);
    t.drawText(xoff + 6, 116, buf, mv < 3300 ? Col::WARN : Col::MUTED);

    fmt_age(age, sizeof(age), c.nowMs - seen);
    snprintf(buf, sizeof(buf), "%s  %ddB", age, n ? d.b.rssi : d.a.rssi);
    t.drawText(xoff + W - 6 - t.textWidth(buf), 116, buf, Col::MUTED);
  }
}

// ---------- PUMP / OUTPUTS ----------
static inline void draw_relay(const UiDrawContext& c, int16_t xoff) {
  auto& t = *c.tft;
  const auto& d = *c.data;
  const auto& u = *c.ui;
  char buf[16];

  t.drawHeader(xoff, "PUMP CONTROL");

  OutRow outs[2 * Payload::EXT_SLOTS];
  const uint8_t nOut = output_rows(d, outs, 2 * Payload::EXT_SLOTS);
  Row rows[RELAY_FIXED_ROWS + 2 * Payload::EXT_SLOTS];

  if (d.relayAck.pending) row_set(rows[0], "MAIN PUMP", "SENDING", Col::WARN);
  else if (!d.b.valid)    row_set(rows[0], "MAIN PUMP", "--");
  else row_set(rows[0], "MAIN PUMP", d.b.relay_state ? "ON" : "OFF", d.b.relay_state ? Col::ACCENT : Col::MUTED);

  row_set(rows[1], "AUTO MODE", d.autoPump.enabled ? "ON" : "OFF", d.autoPump.enabled ? Col::ACCENT : Col::MUTED);
  snprintf(buf, sizeof(buf), "%u", (unsigned)d.autoPump.soil_on_threshold);
  row_set(rows[2], "ON  IF RAW >", buf, Col::TEXT);
  snprintf(buf, sizeof(buf), "%u", (unsigned)d.autoPump.soil_off_threshold);
  row_set(rows[3], "OFF IF RAW <", buf, Col::TEXT);
  snprintf(buf, sizeof(buf), "%us", (unsigned)d.autoPump.cooldown_sec);
  row_set(rows[4], "COOLDOWN", buf, Col::TEXT);
  if (u.relayEdit && u.relayIndex >= 2 && u.relayIndex <= 4) rows[u.relayIndex].editR = true;

  for (uint8_t i = 0; i < nOut; i++) {
    const OutRow& o = outs[i];
    const bool on = output_states(d, o.node) & (1 << o.slot);
    char left[24];
    snprintf(left, sizeof(left), "%c%u %s  G%u", o.node ? 'B' : 'A', (unsigned)(o.slot + 1),
             SensorMap::type_name(d.sensSent[o.node].slot[o.slot].type), (unsigned)d.sensSent[o.node].slot[o.slot].pin);
    if (d.outAck[o.node].pending) row_set(rows[RELAY_FIXED_ROWS + i], left, on ? "ON.." : "OFF..", Col::WARN);
    else row_set(rows[RELAY_FIXED_ROWS + i], left, on ? "ON" : "OFF", on ? Col::ACCENT : Col::MUTED);
  }

  draw_list(t, xoff, rows, (int8_t)(RELAY_FIXED_ROWS + nOut), u.relayIndex);
}

// ---------- SETTINGS ----------
static inline void draw_settings(const UiDrawContext& c, int16_t xoff) {
  auto& t = *c.tft;
  const auto& d = *c.data;
  const auto& u = *c.ui;
  char buf[14];
  uint16_t col;

  t.drawHeader(xoff, "NODE TIMING");

  Row rows[SETTINGS_ROWS];
  snprintf(buf, sizeof(buf), "%us", (unsigned)d.cfg.cfgA_period_s);
  row_set(rows[0], "A WAKE EVERY", buf, Col::TEXT);
  snprintf(buf, sizeof(buf), "%ums", (unsigned)d.cfg.cfgA_rxwin_ms);
  row_set(rows[1], "A LISTEN FOR", buf, Col::TEXT);
  const char* st = ack_text(d.cfgAAck, col);
  row_set(rows[2], "SEND TO NODE A", st, col);

  snprintf(buf, sizeof(buf), "%us", (unsigned)d.cfg.cfgB_period_s);
  row_set(rows[3], "B WAKE EVERY", buf, Col::TEXT);
  snprintf(buf, sizeof(buf), "%ums", (unsigned)d.cfg.cfgB_rxwin_ms);
  row_set(rows[4], "B LISTEN FOR", buf, Col::TEXT);
  st = ack_text(d.cfgBAck, col);
  row_set(rows[5], "SEND TO NODE B", st, col);

  if (u.settingsEdit) rows[u.settingsIndex].editR = true;
  draw_list(t, xoff, rows, SETTINGS_ROWS, u.settingsIndex);
}

// ---------- SENSORS (assignable slots only) ----------
static inline void draw_sensors(const UiDrawContext& c, int16_t xoff) {
  auto& t = *c.tft;
  const auto& d = *c.data;
  const auto& u = *c.ui;
  const uint8_t n = u.sensNode;
  const Payload::CmdSensorCfg& cfg = d.sensCfg[n];
  char l[24], r[14];

  t.drawHeader(xoff, n ? "SENSORS  NODE B" : "SENSORS  NODE A");
  if (u.sensStage) t.drawText(xoff + t.width() - 28, 5, u.sensStage == 1 ? "TYPE" : "PIN", Col::EDIT);

  Row rows[SENS_ROWS];
  row_set(rows[0], n ? "NODE B" : "NODE A", "PRESS: SWITCH");

  for (uint8_t i = 0; i < Payload::EXT_SLOTS; i++) {
    const Payload::SensorSlotCfg& s = cfg.slot[i];
    Row& R = rows[1 + i];
    snprintf(l, sizeof(l), "%u  %s", (unsigned)(i + 1), SensorMap::type_name(s.type));
    if (s.type == Payload::SENS_NONE) snprintf(r, sizeof(r), "--");
    else snprintf(r, sizeof(r), "GPIO %u", (unsigned)s.pin);
    row_set(R, l, r, s.type == Payload::SENS_NONE ? Col::BORDER : Col::TEXT);
    if (u.sensIndex == 1 + i) { R.editL = (u.sensStage == 1); R.editR = (u.sensStage == 2); }
  }

  uint16_t col;
  const char* st;
  const bool unsent = SensorMap::differs(cfg, d.sensSent[n]);
  if (d.sensAck[n].pending)            { st = "SENDING"; col = Col::WARN; }
  else if (unsent)                     { st = (d.sensAck[n].lastSendMs && !d.sensAck[n].ok) ? "FAILED" : "UNSENT";
                                         col = (st[0] == 'F') ? Col::BAD : Col::WARN; }
  else                                 { st = ack_text(d.sensAck[n], col); if (!*st) { st = "SAVED"; col = Col::MUTED; } }
  snprintf(l, sizeof(l), "SEND TO NODE %c", n ? 'B' : 'A');
  row_set(rows[SENS_ROWS - 1], l, st, col);

  draw_list(t, xoff, rows, SENS_ROWS, u.sensIndex);
}

// ---------- frame ----------
static inline void ui_draw_one(const UiDrawContext& c, UiPage p, int16_t xoff) {
  switch (p) {
    case UiPage::Home:          draw_home(c, xoff); break;
    case UiPage::NodeA_Details: draw_node(c, xoff, 0); break;
    case UiPage::NodeB_Details: draw_node(c, xoff, 1); break;
    case UiPage::Relay:         draw_relay(c, xoff); break;
    case UiPage::Sensors:       draw_sensors(c, xoff); break;
    case UiPage::Settings:      draw_settings(c, xoff); break;
    default: break;
  }
}

static inline void ui_draw(const UiDrawContext& c) {
  auto& t = *c.tft;
  const auto& u = *c.ui;
  const int16_t W = t.width();

  t.beginFrame();
  t.fillScreen(Col::BG);

  if (u.tr.active) {
    const int16_t off = (int16_t)(ui_ease(c.nowMs, u.tr.startMs, u.tr.durationMs) * W);
    ui_draw_one(c, u.tr.from, (int16_t)(-u.tr.dir * off));
    ui_draw_one(c, u.tr.to,   (int16_t)(u.tr.dir * (W - off)));
  } else {
    ui_draw_one(c, u.page, 0);
  }

  t.endFrame();
}
