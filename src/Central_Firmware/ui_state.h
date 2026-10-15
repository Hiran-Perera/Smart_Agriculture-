#pragma once
#include <Arduino.h>

enum class UiPage : uint8_t {
  Home = 0,
  NodeA_Details,
  NodeB_Details,
  Relay,
  Sensors,
  Settings,
  COUNT
};

struct UiTransition {
  bool active = false;
  UiPage from = UiPage::Home;
  UiPage to = UiPage::Home;
  int8_t dir = 1;              // +1 new page slides in from the right, -1 from the left
  uint32_t startMs = 0;
  uint16_t durationMs = 220;
};

struct UiState {
  UiPage page = UiPage::Home;

  int8_t homeIndex = 0;        // 0..4: NodeA, NodeB, Pump, Sensors, Settings

  int8_t relayIndex = 0;       // 0 pump, 1 auto, 2 on th, 3 off th, 4 cooldown, 5.. outputs
  bool relayEdit = false;

  int8_t settingsIndex = 0;    // 0 A period, 1 A rxwin, 2 send A, 3 B period, 4 B rxwin, 5 send B
  bool settingsEdit = false;

  // sensors page
  uint8_t sensNode = 0;        // 0 = Node A, 1 = Node B
  int8_t sensIndex = 0;        // 0 node, 1..EXT_SLOTS slots, then SEND
  uint8_t sensStage = 0;       // 0 browse, 1 editing type, 2 editing pin

  // node pages: one sensor at a time, auto cycling
  uint8_t nodeItem = 0;
  uint8_t nodeItemPrev = 0;
  int8_t itemDir = 1;
  bool itemAnim = false;
  uint32_t itemAnimStartMs = 0;
  uint32_t cycleMs = 0;
  uint32_t cyclePauseUntil = 0;

  uint32_t lastPeriodicMs = 0;

  // screensaver
  uint32_t lastInputMs = 0;
  bool saverActive = false;

  bool dirty = true;
  bool dirtySoft = true;

  UiTransition tr;

  void reset() { *this = UiState(); lastInputMs = millis(); }

  void markDirty() { dirty = true; }
  void markDirtySoft() { dirtySoft = true; }
  void clearDirty() { dirty = false; dirtySoft = false; }

  bool shouldRedraw(uint32_t) const { return dirty || dirtySoft || tr.active || itemAnim; }
};

static const uint16_t ITEM_ANIM_MS = 260;

static inline void ui_start_transition(UiState& ui, UiPage to, int8_t dir) {
  if (ui.page == to && !ui.tr.active) return;
  ui.tr.active = true;
  ui.tr.from = ui.page;
  ui.tr.to = to;
  ui.tr.dir = dir >= 0 ? 1 : -1;
  ui.tr.startMs = millis();
  // the destination is "current" from now on, so input acts on it during the slide
  ui.page = to;
}

static inline bool ui_transition_tick(UiState& ui, uint32_t now) {
  if (!ui.tr.active) return false;
  if ((uint32_t)(now - ui.tr.startMs) >= ui.tr.durationMs) ui.tr.active = false;
  return true;
}

// 0..1 eased progress (cubic ease-out)
static inline float ui_ease(uint32_t now, uint32_t start, uint16_t dur) {
  float t = (float)(uint32_t)(now - start) / (float)dur;
  if (t < 0) t = 0;
  if (t > 1) t = 1;
  float u = 1.0f - t;
  return 1.0f - u * u * u;
}
