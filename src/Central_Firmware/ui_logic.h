#pragma once
#include <Arduino.h>
#include "ui_state.h"
#include "data_model.h"
#include "input_encoder.h"

struct UiActions {
  bool requestPageChange = false;
  UiPage nextPage = UiPage::Home;

  bool sendRelayToggle = false;
  bool relayTargetOn = false;

  bool sendCfgA = false;
  uint16_t cfgA_period_s = 10;
  uint16_t cfgA_rxwin_ms = 400;

  bool sendCfgB = false;
  uint16_t cfgB_period_s = 10;
  uint16_t cfgB_rxwin_ms = 400;

  bool autoSave = false;
};

static inline uint16_t clamp_u16(int32_t v, uint16_t lo, uint16_t hi) {
  if (v < (int32_t)lo) return lo;
  if (v > (int32_t)hi) return hi;
  return (uint16_t)v;
}

static inline void ui_go_home(UiState& ui, UiActions& act) {
  act.requestPageChange = true;
  act.nextPage = UiPage::Home;
}

static inline void ui_handle_input(UiState& ui, DataModel& data, const EncoderEvents& ev, UiActions& act) {
  int8_t d = ev.delta;

  if (ev.longPress) {
    ui_go_home(ui, act);
    return;
  }

  switch (ui.page) {
    case UiPage::Home: {
      if (d) ui.homeIndex = (int8_t)clamp_u16(ui.homeIndex + d, 0, 3);

      if (ev.shortPress) {
        UiPage to = UiPage::Home;
        if (ui.homeIndex == 0) to = UiPage::NodeA_Details;
        if (ui.homeIndex == 1) to = UiPage::NodeB_Details;
        if (ui.homeIndex == 2) to = UiPage::Relay;
        if (ui.homeIndex == 3) to = UiPage::Settings1;
        act.requestPageChange = true;
        act.nextPage = to;
      }
    } break;

    case UiPage::NodeA_Details: {
      if (d) {
        act.requestPageChange = true;
        act.nextPage = (d > 0) ? UiPage::NodeB_Details : UiPage::Home;
      }
    } break;

    case UiPage::NodeB_Details: {
      if (d) {
        act.requestPageChange = true;
        act.nextPage = (d > 0) ? UiPage::Relay : UiPage::NodeA_Details;
      }
    } break;

    case UiPage::Relay: {
      // 0 MANUAL TARGET
      // 1 SEND
      // 2 AUTO ENABLE
      // 3 ON THRESH
      // 4 OFF THRESH
      // 5 COOLDOWN
      if (!ui.relayEdit) {
        if (d) ui.relayIndex = (int8_t)clamp_u16(ui.relayIndex + d, 0, 5);

        if (ev.shortPress) {
          if (ui.relayIndex == 0) {
            data.relay_target_on = !data.relay_target_on;
          } else if (ui.relayIndex == 1) {
            act.sendRelayToggle = true;
            act.relayTargetOn = data.relay_target_on;
          } else if (ui.relayIndex == 2) {
            data.autoPump.enabled = !data.autoPump.enabled;
            act.autoSave = true;
          } else {
            ui.relayEdit = true;
          }
        }
      } else {
        if (d) {
          if (ui.relayIndex == 3) data.autoPump.soil_on_threshold  = clamp_u16((int32_t)data.autoPump.soil_on_threshold  + d * 5, 0, 4095);
          if (ui.relayIndex == 4) data.autoPump.soil_off_threshold = clamp_u16((int32_t)data.autoPump.soil_off_threshold + d * 5, 0, 4095);
          if (ui.relayIndex == 5) data.autoPump.cooldown_sec       = clamp_u16((int32_t)data.autoPump.cooldown_sec       + d * 5, 0, 3600);
          act.autoSave = true;
        }
        if (ev.shortPress) {
          ui.relayEdit = false;
          act.autoSave = true;
        }
      }

      if (d && !ui.relayEdit && (ui.relayIndex == 5) && d > 0) {
        act.requestPageChange = true;
        act.nextPage = UiPage::Settings1;
      }
      if (d && !ui.relayEdit && (ui.relayIndex == 0) && d < 0) {
        act.requestPageChange = true;
        act.nextPage = UiPage::NodeB_Details;
      }
    } break;

    case UiPage::Settings1: {
      // 0 A PERIOD
      // 1 A RXWIN
      // 2 SEND A
      if (!ui.settingsEdit1) {
        if (d) ui.settingsIndex1 = (int8_t)clamp_u16(ui.settingsIndex1 + d, 0, 2);

        if (ev.shortPress) {
          if (ui.settingsIndex1 == 2) {
            act.sendCfgA = true;
            act.cfgA_period_s = data.cfg.cfgA_period_s;
            act.cfgA_rxwin_ms = data.cfg.cfgA_rxwin_ms;
          } else {
            ui.settingsEdit1 = true;
          }
        }

        if (d > 0 && ui.settingsIndex1 == 2) {
          act.requestPageChange = true;
          act.nextPage = UiPage::Settings2;
        }
        if (d < 0 && ui.settingsIndex1 == 0) {
          act.requestPageChange = true;
          act.nextPage = UiPage::Relay;
        }
      } else {
        if (d) {
          if (ui.settingsIndex1 == 0) data.cfg.cfgA_period_s = clamp_u16((int32_t)data.cfg.cfgA_period_s + d, 1, 3600);
          if (ui.settingsIndex1 == 1) data.cfg.cfgA_rxwin_ms = clamp_u16((int32_t)data.cfg.cfgA_rxwin_ms + d * 10, 50, 5000);
        }
        if (ev.shortPress) ui.settingsEdit1 = false;
      }
    } break;

    case UiPage::Settings2: {
      // 0 B PERIOD
      // 1 B RXWIN
      // 2 SEND B
      if (!ui.settingsEdit2) {
        if (d) ui.settingsIndex2 = (int8_t)clamp_u16(ui.settingsIndex2 + d, 0, 2);

        if (ev.shortPress) {
          if (ui.settingsIndex2 == 2) {
            act.sendCfgB = true;
            act.cfgB_period_s = data.cfg.cfgB_period_s;
            act.cfgB_rxwin_ms = data.cfg.cfgB_rxwin_ms;
          } else {
            ui.settingsEdit2 = true;
          }
        }

        if (d > 0 && ui.settingsIndex2 == 2) {
          ui_go_home(ui, act);
        }
        if (d < 0 && ui.settingsIndex2 == 0) {
          act.requestPageChange = true;
          act.nextPage = UiPage::Settings1;
        }
      } else {
        if (d) {
          if (ui.settingsIndex2 == 0) data.cfg.cfgB_period_s = clamp_u16((int32_t)data.cfg.cfgB_period_s + d, 1, 3600);
          if (ui.settingsIndex2 == 1) data.cfg.cfgB_rxwin_ms = clamp_u16((int32_t)data.cfg.cfgB_rxwin_ms + d * 10, 50, 5000);
        }
        if (ev.shortPress) ui.settingsEdit2 = false;
      }
    } break;

    default: break;
  }
}