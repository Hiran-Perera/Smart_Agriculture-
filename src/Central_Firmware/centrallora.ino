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
#include "sensor_store.h"

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
SensorStore g_sens;

// Draws the current UI page at an x offset (used under the sliding screensaver).
static void draw_page_under(int16_t xoff, uint32_t now)
{
  UiDrawContext ctx;
  ctx.nowMs = now;
  ctx.tft = &g_tft;
  ctx.data = &g_data;
  ctx.ui = &g_ui;
  ui_draw_one(ctx, g_ui.page, xoff);
}

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
  g_sens.begin(g_data);

  EncoderConfig ec;
  ec.pinA = ENC_A_PIN;
  ec.pinB = ENC_B_PIN;
  ec.pinSw = ENC_SW_PIN;
  g_enc.begin(ec);

  LoRaConfig lc;
  g_lora.begin(lc, g_data);

  g_ui.reset();
  g_ui.markDirty();

  g_ui.lastInputMs = millis();
  g_ui.saverActive = false;

  BtPanel::begin("SmartAgroGW");
  BtPanel::bind(&g_data, &g_ui);

  // Connect panel actions to the LoRa command queue
  BtPanel::onRelay([](bool on){ g_lora.sendRelayTarget(on); });
  BtPanel::onCfgA([](uint16_t p, uint16_t w){ g_lora.sendCfgA(p, w); });
  BtPanel::onCfgB([](uint16_t p, uint16_t w){ g_lora.sendCfgB(p, w); });
}

void loop()
{
  BtPanel::poll();

  uint32_t now = millis();

  // Always poll LoRa first (never blocks: queued commands are sent from inside poll)
  if (g_lora.poll(g_data)) {
    g_ui.markDirtySoft();
  }

  EncoderEvents ev;
  g_enc.poll(ev);

  if (ev.delta || ev.shortPress || ev.longPress)
  {
    g_ui.lastInputMs = now;

    if (g_saver.active() && !g_saver.exiting()) {
      // the first touch only wakes the screen (saver slides away), it never acts on a page
      g_saver.stop(now);
    } else {
      UiActions act;
      ui_handle_input(g_ui, g_data, ev, act, now);

      if (act.requestPageChange)
        ui_start_transition(g_ui, act.nextPage, act.dir);

      if (act.sendRelay) {
        g_lora.sendRelayTarget(act.relayTargetOn);
        g_auto.mark_manual_override(g_data, now);
      }

      if (act.sendCfgA) g_lora.sendCfgA(g_data.cfg.cfgA_period_s, g_data.cfg.cfgA_rxwin_ms);
      if (act.sendCfgB) g_lora.sendCfgB(g_data.cfg.cfgB_period_s, g_data.cfg.cfgB_rxwin_ms);

      if (act.sendSens) {
        g_sens.save(g_data, act.sensNode);
        g_lora.sendSensorCfg(act.sensNode, g_data.sensCfg[act.sensNode]);
      }

      if (act.sendOut) g_lora.sendOutputs(act.outNode, act.outStates);

      if (act.autoSave) {
        AutoPumpAutomation::enforce_hysteresis(g_data.autoPump);
        g_auto.save(g_data);
      }

      g_ui.markDirtySoft();
    }
  }

  if (g_data.timingDirty) {
    g_data.timingDirty = false;
    g_sens.saveTiming(g_data);
  }

  // Auto pump (runs even if no UI input)
  if (g_auto.tick(g_data, now))
  {
    if (g_data.autoPump.auto_request_pending)
    {
      g_data.relay_target_on = g_data.autoPump.auto_target_on;
      g_lora.sendRelayTarget(g_data.autoPump.auto_target_on);
      g_data.autoPump.auto_request_pending = false;
      g_ui.markDirtySoft();
    }
  }

  const uint32_t SAVER_TIMEOUT_MS = 30000;
  if (!g_saver.active() && !g_ui.tr.active && (uint32_t)(now - g_ui.lastInputMs) >= SAVER_TIMEOUT_MS) {
    ui_cancel_edits(g_ui);
    g_saver.start(now);
  }

  // Screensaver owns the screen while it slides in, shows, or slides out
  if (g_saver.active()) {
    g_saver.tick(g_tft, now, draw_page_under);
    g_ui.saverActive = g_saver.active();
    if (!g_saver.active()) g_ui.markDirty();   // fully back: normal drawing resumes
    return;
  }
  g_ui.saverActive = false;

  ui_transition_tick(g_ui, now);
  if (ui_tick(g_ui, g_data, now)) g_ui.markDirtySoft();

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
