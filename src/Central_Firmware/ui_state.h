#pragma once
#include <Arduino.h>

enum class UiPage : uint8_t {
  Home = 0,
  NodeA_Details,
  NodeB_Details,
  Relay,
  Settings1,
  Settings2,
  COUNT
};

struct UiTransition {
  bool active = false;
  UiPage from = UiPage::Home;
  UiPage to = UiPage::Home;
  uint32_t startMs = 0;
  uint16_t durationMs = 180;
};

struct UiState {
  UiPage page = UiPage::Home;

  int8_t homeIndex = 0;      // 0..3: NodeA, NodeB, Pump, Settings
  int8_t relayIndex = 0;     // 0..5
  bool relayEdit = false;

  int8_t settingsIndex1 = 0; // 0..2
  bool settingsEdit1 = false;

  int8_t settingsIndex2 = 0; // 0..2
  bool settingsEdit2 = false;

  // screensaver
  uint32_t lastInputMs = 0;
  bool saverActive = false;

  bool dirty = true;
  bool dirtySoft = true;

  UiTransition tr;

  void reset() {
    page = UiPage::Home;

    homeIndex = 0;
    relayIndex = 0;
    relayEdit = false;

    settingsIndex1 = 0;
    settingsEdit1 = false;

    settingsIndex2 = 0;
    settingsEdit2 = false;

    lastInputMs = millis();
    saverActive = false;

    dirty = true;
    dirtySoft = true;
    tr = UiTransition();
  }

  void markDirty() { dirty = true; }
  void markDirtySoft() { dirtySoft = true; }
  void clearDirty() { dirty = false; dirtySoft = false; }

  bool shouldRedraw(uint32_t) const { return dirty || dirtySoft || tr.active; }
};

static inline void ui_start_transition(UiState& ui, UiPage to) {
  if (ui.page == to) return;
  ui.tr.active = true;
  ui.tr.from = ui.page;
  ui.tr.to = to;
  ui.tr.startMs = millis();
}

static inline bool ui_transition_tick(UiState& ui, uint32_t now) {
  if (!ui.tr.active) return false;
  if ((uint32_t)(now - ui.tr.startMs) >= ui.tr.durationMs) {
    ui.tr.active = false;
    ui.page = ui.tr.to;
    return true;
  }
  return true;
}