#include <Arduino.h>
#include "bt_panel.h"
#include "screensaver.h"
#include "tft_driver.h"
#include "data_model.h"
#include "input_encoder.h"
#include "lora_manager.h"
#include "ui_state.h"
#include "ui_logic.h"
#include "ui_screens.h"
#include "automation.h"

static const int TFT_CS  = 5;   // D5
static const int TFT_DC  = 2;   // D2
static const int TFT_RST = 4;   // D4
static const int TFT_BL  = -1;  // tied to 3.3V

static const int ENC_A_PIN  = 33;
static const int ENC_B_PIN  = 32;
static const int ENC_SW_PIN = 25;

static const uint8_t TFT_ROTATION = 1;

static ScreenSaver g_saver;
TftDriver g_tft;
DataModel g_data;
EncoderInput g_enc;
LoRaManager g_lora;
UiState g_ui;
AutoPumpAutomation g_auto;

void setup()
{
  Serial.begin(115200);

  TftConfig cfg;
  cfg.cs = TFT_CS;
  cfg.dc = TFT_DC;
  cfg.rst = TFT_RST;
  cfg.rotation = TFT_ROTATION;
  g_tft.begin(cfg);

  g_data.reset();
  g_auto.begin(g_data);

  EncoderConfig ec;
  ec.pinA = ENC_A_PIN;
  ec.pinB = ENC_B_PIN;
  ec.pinSw = ENC_SW_PIN;
  g_enc.begin(ec);

  LoRaConfig lc;
  g_lora.begin(lc);

  g_ui.reset();
  g_ui.markDirty();

  g_ui.lastInputMs = millis();
  g_ui.saverActive = false;

  BtPanel::begin("SmartAgroGW");
  BtPanel::bind(&g_data, &g_ui);

// Connect panel actions to your existing LoRa functions
  BtPanel::onRelay([](bool on){ g_lora.sendRelayTarget(on); });
  BtPanel::onCfgA([](uint16_t p, uint16_t w){ g_lora.sendCfgA(p, w); });
  BtPanel::onCfgB([](uint16_t p, uint16_t w){ g_lora.sendCfgB(p, w); });
}

void loop()
{
  BtPanel::poll();

  uint32_t now = millis();

  // Always poll LoRa first
  if (g_lora.poll(g_data)) {
    g_ui.markDirty();
  }

  EncoderEvents ev;
  g_enc.poll(ev);

  if (ev.delta || ev.shortPress || ev.longPress)
  {
    g_ui.lastInputMs = now;

    if (g_ui.saverActive) {
      g_ui.saverActive = false;
      g_ui.markDirty();
    }

    UiActions act;
    ui_handle_input(g_ui, g_data, ev, act);

    if (act.requestPageChange)
      ui_start_transition(g_ui, act.nextPage);

    if (act.sendRelayToggle)
    {
      g_lora.sendRelayTarget(act.relayTargetOn);
      g_auto.mark_manual_override(g_data, now);
    }

    // Send config commands from UI
if (act.sendCfgA) {
  g_lora.sendCfgA(act.cfgA_period_s, act.cfgA_rxwin_ms);
}

if (act.sendCfgB) {
  g_lora.sendCfgB(act.cfgB_period_s, act.cfgB_rxwin_ms);
}

    if (act.autoSave)
    {
      AutoPumpAutomation::enforce_hysteresis(g_data.autoPump);
      g_auto.save(g_data);
    }

    g_ui.markDirty();
  }

  // Auto pump (runs even if no UI input)
  if (g_auto.tick(g_data, now))
  {
    if (g_data.autoPump.auto_request_pending)
    {
      g_lora.sendRelayTarget(g_data.autoPump.auto_target_on);
      g_data.autoPump.auto_request_pending = false;
      g_ui.markDirty();
    }
  }

  const uint32_t SAVER_TIMEOUT_MS = 30000;
  if (!g_ui.saverActive && !g_ui.tr.active && (uint32_t)(now - g_ui.lastInputMs) >= SAVER_TIMEOUT_MS) {
    g_ui.saverActive = true;
    g_saver.reset(now);
  }

  // Screensaver draw mode
  if (g_ui.saverActive) {
    g_saver.tick(g_tft, now);
    return;
  }

  if (ui_transition_tick(g_ui, now))
    g_ui.markDirty();

  if (g_ui.shouldRedraw(now))
  {
    UiDrawContext ctx;
    ctx.nowMs = now;
    ctx.tft = &g_tft;
    ctx.data = &g_data;
    ctx.ui = &g_ui;
    ui_draw(ctx);
    g_ui.clearDirty();
  }
}