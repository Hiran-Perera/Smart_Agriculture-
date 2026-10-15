#pragma once
#include <Arduino.h>
#include "ui_state.h"
#include "data_model.h"
#include "input_encoder.h"
#include "node_items.h"

// Navigation rules (same on every page):
//   rotate        move the selection / change the value being edited
//   short press   open / toggle / start or confirm editing
//   long press    back to Home from anywhere (cancels editing)
//   rotate above the first row of a list = back to Home

struct UiActions {
  bool requestPageChange = false;
  UiPage nextPage = UiPage::Home;
  int8_t dir = 1;                 // slide direction

  bool sendRelay = false;
  bool relayTargetOn = false;

  bool sendCfgA = false;
  bool sendCfgB = false;

  bool autoSave = false;

  bool sendSens = false;
  uint8_t sensNode = 0;

  bool sendOut = false;
  uint8_t outNode = 0;
  uint8_t outStates = 0;
};

static const uint32_t NODE_CYCLE_MS = 3500;     // auto advance on node pages
static const uint32_t NODE_PAUSE_MS = 10000;    // pause after manual browsing
static const int8_t   SENS_ROWS = 1 + Payload::EXT_SLOTS + 1;
static const int8_t   SETTINGS_ROWS = 6;
static const int8_t   RELAY_FIXED_ROWS = 5;

static inline int32_t clamp_i32(int32_t v, int32_t lo, int32_t hi) { return v < lo ? lo : (v > hi ? hi : v); }
static inline uint16_t clamp_u16(int32_t v, uint16_t lo, uint16_t hi) { return (uint16_t)clamp_i32(v, lo, hi); }

// ---------- pump / valve outputs (listed on the Pump page) ----------
struct OutRow { uint8_t node; uint8_t slot; };

static inline uint8_t output_rows(const DataModel& d, OutRow* out, uint8_t max) {
  uint8_t c = 0;
  for (uint8_t n = 0; n < 2; n++)
    for (uint8_t i = 0; i < Payload::EXT_SLOTS && c < max; i++)
      if (SensorMap::is_output(d.sensSent[n].slot[i].type)) out[c++] = { n, i };
  return c;
}

// Current states of a node's outputs: the request while it is in flight, else what the node reports.
static inline uint8_t output_states(const DataModel& d, uint8_t n) {
  if (d.outAck[n].pending) return d.outTarget[n];
  uint8_t s = 0;
  const ExtVal* e = d.ext(n);
  for (uint8_t i = 0; i < Payload::EXT_SLOTS; i++) {
    if (SensorMap::is_output(e[i].type) && e[i].valid) { if (e[i].v1 == 1) s |= (1 << i); }
    else if (d.outTarget[n] & (1 << i)) s |= (1 << i);
  }
  return s;
}

// ---------- helpers ----------
static inline void ui_go(UiActions& act, UiPage p, int8_t dir) {
  act.requestPageChange = true;
  act.nextPage = p;
  act.dir = dir;
}

static inline void ui_cancel_edits(UiState& ui) {
  ui.relayEdit = false;
  ui.settingsEdit = false;
  ui.sensStage = 0;
}

static inline void ui_enter_node(UiState& ui, uint32_t now) {
  ui.nodeItem = 0;
  ui.nodeItemPrev = 0;
  ui.itemAnim = false;
  ui.cycleMs = now;
  ui.cyclePauseUntil = 0;
}

static inline void ui_item_step(UiState& ui, uint8_t count, int8_t dir, uint32_t now) {
  if (count < 2) return;
  ui.nodeItemPrev = ui.nodeItem % count;
  ui.nodeItem = (uint8_t)((ui.nodeItemPrev + (dir > 0 ? 1 : count - 1)) % count);
  ui.itemDir = dir > 0 ? 1 : -1;
  ui.itemAnim = true;
  ui.itemAnimStartMs = now;
  ui.cycleMs = now;
}

// Moves a list selection; returns true if the user scrolled above the first row.
static inline bool ui_list_move(int8_t& idx, int8_t d, int8_t rows) {
  if (d < 0 && idx == 0) return true;
  idx = (int8_t)clamp_i32(idx + d, 0, rows - 1);
  return false;
}

static inline uint16_t period_step(uint16_t v) { return v < 60 ? 1 : (v < 600 ? 10 : 60); }

// ---------- input ----------
static inline void ui_handle_input(UiState& ui, DataModel& data, const EncoderEvents& ev, UiActions& act, uint32_t now) {
  const int8_t d = ev.delta;

  if (ev.longPress) {
    ui_cancel_edits(ui);
    if (ui.page != UiPage::Home) ui_go(act, UiPage::Home, -1);
    return;
  }

  switch (ui.page) {
    case UiPage::Home: {
      if (d) ui.homeIndex = (int8_t)clamp_i32(ui.homeIndex + d, 0, 4);
      if (ev.shortPress) {
        static const UiPage to[5] = { UiPage::NodeA_Details, UiPage::NodeB_Details, UiPage::Relay,
                                      UiPage::Sensors, UiPage::Settings };
        UiPage p = to[ui.homeIndex];
        if (p == UiPage::NodeA_Details || p == UiPage::NodeB_Details) ui_enter_node(ui, now);
        ui_go(act, p, 1);
      }
    } break;

    case UiPage::NodeA_Details:
    case UiPage::NodeB_Details: {
      const uint8_t n = (ui.page == UiPage::NodeB_Details) ? 1 : 0;
      if (d) {
        ui_item_step(ui, node_item_count(data, n), d, now);
        ui.cyclePauseUntil = now + NODE_PAUSE_MS;
      }
      if (ev.shortPress) ui_go(act, UiPage::Home, -1);
    } break;

    case UiPage::Relay: {
      OutRow outs[2 * Payload::EXT_SLOTS];
      const uint8_t nOut = output_rows(data, outs, 2 * Payload::EXT_SLOTS);
      const int8_t rows = (int8_t)(RELAY_FIXED_ROWS + nOut);
      if (ui.relayIndex >= rows) ui.relayIndex = rows - 1;

      if (!ui.relayEdit) {
        if (d && ui_list_move(ui.relayIndex, d, rows)) { ui_go(act, UiPage::Home, -1); break; }

        if (ev.shortPress) {
          if (ui.relayIndex == 0) {
            bool cur = data.relayAck.pending ? data.relay_target_on : data.b.relay_state;
            data.relay_target_on = !cur;
            act.sendRelay = true;
            act.relayTargetOn = data.relay_target_on;
          } else if (ui.relayIndex == 1) {
            data.autoPump.enabled = !data.autoPump.enabled;
            act.autoSave = true;
          } else if (ui.relayIndex < RELAY_FIXED_ROWS) {
            ui.relayEdit = true;
          } else {
            const OutRow& o = outs[ui.relayIndex - RELAY_FIXED_ROWS];
            uint8_t states = output_states(data, o.node) ^ (uint8_t)(1 << o.slot);
            data.outTarget[o.node] = states;
            act.sendOut = true;
            act.outNode = o.node;
            act.outStates = states;
          }
        }
      } else {
        if (d) {
          AutoPumpSettings& a = data.autoPump;
          if (ui.relayIndex == 2) a.soil_on_threshold  = clamp_u16((int32_t)a.soil_on_threshold  + d * 10, 0, 4095);
          if (ui.relayIndex == 3) a.soil_off_threshold = clamp_u16((int32_t)a.soil_off_threshold + d * 10, 0, 4095);
          if (ui.relayIndex == 4) a.cooldown_sec       = clamp_u16((int32_t)a.cooldown_sec       + d * 5, 0, 3600);
        }
        if (ev.shortPress) {
          ui.relayEdit = false;
          act.autoSave = true;
        }
      }
    } break;

    case UiPage::Settings: {
      if (!ui.settingsEdit) {
        if (d && ui_list_move(ui.settingsIndex, d, SETTINGS_ROWS)) { ui_go(act, UiPage::Home, -1); break; }
        if (ev.shortPress) {
          if (ui.settingsIndex == 2) act.sendCfgA = true;
          else if (ui.settingsIndex == 5) act.sendCfgB = true;
          else ui.settingsEdit = true;
        }
      } else {
        if (d) {
          ConfigState& c = data.cfg;
          switch (ui.settingsIndex) {
            case 0: c.cfgA_period_s = clamp_u16((int32_t)c.cfgA_period_s + d * period_step(c.cfgA_period_s), 5, 3600); break;
            case 1: c.cfgA_rxwin_ms = clamp_u16((int32_t)c.cfgA_rxwin_ms + d * 50, 100, 5000); break;
            case 3: c.cfgB_period_s = clamp_u16((int32_t)c.cfgB_period_s + d * period_step(c.cfgB_period_s), 5, 3600); break;
            case 4: c.cfgB_rxwin_ms = clamp_u16((int32_t)c.cfgB_rxwin_ms + d * 50, 100, 8000); break;
          }
        }
        if (ev.shortPress) ui.settingsEdit = false;
      }
    } break;

    case UiPage::Sensors: {
      const uint8_t nodeId = ui.sensNode ? Payload::NODE_B : Payload::NODE_A;
      Payload::CmdSensorCfg& cfg = data.sensCfg[ui.sensNode];

      if (ui.sensStage == 0) {
        if (d && ui_list_move(ui.sensIndex, d, SENS_ROWS)) { ui_go(act, UiPage::Home, -1); break; }

        if (ev.shortPress) {
          if (ui.sensIndex == 0) {
            ui.sensNode ^= 1;
          } else if (ui.sensIndex == SENS_ROWS - 1) {
            act.sendSens = true;
            act.sensNode = ui.sensNode;
          } else {
            ui.sensStage = 1;                         // type first, then pin
          }
        }
      } else {
        const uint8_t slot = (uint8_t)(ui.sensIndex - 1);
        if (d) {
          const int8_t dir = (d > 0) ? 1 : -1;
          if (ui.sensStage == 1) SensorMap::cycle_type(nodeId, cfg, slot, dir);
          else                   SensorMap::cycle_pin(nodeId, cfg, slot, dir);
        }
        if (ev.shortPress) {
          if (ui.sensStage == 1 && cfg.slot[slot].type != Payload::SENS_NONE) ui.sensStage = 2;
          else ui.sensStage = 0;
        }
      }
    } break;

    default: break;
  }
}

// ---------- time based updates (call every loop) ----------
// Returns true when the screen needs a redraw.
static inline bool ui_tick(UiState& ui, const DataModel& data, uint32_t now) {
  bool redraw = false;

  if (ui.itemAnim && (uint32_t)(now - ui.itemAnimStartMs) >= ITEM_ANIM_MS) {
    ui.itemAnim = false;
    redraw = true;
  }

  const bool nodePage = (ui.page == UiPage::NodeA_Details || ui.page == UiPage::NodeB_Details);
  if (nodePage && !ui.tr.active && !ui.itemAnim &&
      (int32_t)(now - ui.cyclePauseUntil) >= 0 && (uint32_t)(now - ui.cycleMs) >= NODE_CYCLE_MS) {
    const uint8_t n = (ui.page == UiPage::NodeB_Details) ? 1 : 0;
    ui.cycleMs = now;
    ui_item_step(ui, node_item_count(data, n), 1, now);
    redraw = true;
  }

  // refresh "last seen" ages and live values once a second
  if ((uint32_t)(now - ui.lastPeriodicMs) >= 1000) {
    ui.lastPeriodicMs = now;
    redraw = true;
  }
  return redraw;
}
