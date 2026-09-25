#pragma once
#include <Arduino.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED) || !defined(CONFIG_BT_SPP_ENABLED)
  #error "Bluetooth SPP not enabled on this target. Use ESP32 (not ESP32-C3) for gateway."
#endif

#include <BluetoothSerial.h>

#include "data_model.h"
#include "ui_state.h"

// Non-blocking BT control panel.
// Commands update DataModel/UiState and call your existing LoRa functions via callbacks.
class BtPanel {
public:
  using FnRelay = void (*)(bool on);
  using FnCfg   = void (*)(uint16_t period_s, uint16_t rxwin_ms);

  static void begin(const char* deviceName) {
    _line = "";
    _dm = nullptr;
    _ui = nullptr;
    _fnRelay = nullptr;
    _fnCfgA = nullptr;
    _fnCfgB = nullptr;

    if (!_bt.begin(deviceName)) {
      Serial.println("BT SPP start failed");
    } else {
      Serial.print("BT SPP ready name=");
      Serial.println(deviceName);
    }

    help();
    prompt();
  }

  static void bind(DataModel* dm, UiState* ui) {
    _dm = dm;
    _ui = ui;
  }

  static void onRelay(FnRelay fn) { _fnRelay = fn; }
  static void onCfgA(FnCfg fn)    { _fnCfgA = fn; }
  static void onCfgB(FnCfg fn)    { _fnCfgB = fn; }

  static void poll() {
    while (_bt.available()) {
      char c = (char)_bt.read();
      // echo
      _bt.write((uint8_t)c);

      if (c == '\r') continue;
      if (c == '\n') {
        String s = _line;
        _line = "";
        s.trim();
        if (s.length()) handleLine(s);
        prompt();
      } else {
        if (_line.length() < 160) _line += c;
      }
    }
  }

  static void println(const String& s) { _bt.println(s); }
  static void print(const String& s)   { _bt.print(s); }

  static void help() {
    _bt.println("");
    _bt.println("BT PANEL COMMANDS");
    _bt.println("  help");
    _bt.println("  status");
    _bt.println("  page home|a|b|pump|setA|setB");
    _bt.println("  relay on|off");
    _bt.println("  cfgA <period_s> <rxwin_ms>");
    _bt.println("  cfgB <period_s> <rxwin_ms>");
    _bt.println("  auto on|off");
    _bt.println("  th <on> <off>");
    _bt.println("  cool <sec>");
    _bt.println("");
  }

private:
  static BluetoothSerial _bt;
  static String _line;

  static DataModel* _dm;
  static UiState*   _ui;

  static FnRelay _fnRelay;
  static FnCfg   _fnCfgA;
  static FnCfg   _fnCfgB;

  static void prompt() { _bt.print("> "); }

  static void status() {
    if (!_dm) { _bt.println("status: DataModel not bound"); return; }

    uint32_t now = millis();

    _bt.println("");
    _bt.println("STATUS");

    // Node A
    _bt.print("A valid="); _bt.print(_dm->a.valid ? "1" : "0");
    _bt.print(" age_s=");
    if (_dm->a.lastSeenMs == 0) _bt.print("NA");
    else _bt.print((now - _dm->a.lastSeenMs) / 1000);
    _bt.print(" rssi="); _bt.print(_dm->a.rssi);
    _bt.print(" snr=");  _bt.print(_dm->a.snr, 1);
    _bt.print(" light="); _bt.print(_dm->a.light_raw);
    _bt.print(" batt=");  _bt.print(_dm->a.batt_mV);

    _bt.print(" temp=");
    if (_dm->a.tempC_x100 == (int16_t)0x8000) _bt.print("NA");
    else _bt.print((float)_dm->a.tempC_x100 / 100.0f, 2);

    _bt.print(" hum=");
    if (_dm->a.hum_x100 == (int16_t)0x8000) _bt.print("NA");
    else _bt.print((float)_dm->a.hum_x100 / 100.0f, 2);

    _bt.println("");

    // Node B
    _bt.print("B valid="); _bt.print(_dm->b.valid ? "1" : "0");
    _bt.print(" age_s=");
    if (_dm->b.lastSeenMs == 0) _bt.print("NA");
    else _bt.print((now - _dm->b.lastSeenMs) / 1000);
    _bt.print(" rssi="); _bt.print(_dm->b.rssi);
    _bt.print(" snr=");  _bt.print(_dm->b.snr, 1);
    _bt.print(" soil="); _bt.print(_dm->b.soil_raw);
    _bt.print(" batt="); _bt.print(_dm->b.batt_mV);
    _bt.print(" relay="); _bt.print(_dm->b.relay_state ? "ON" : "OFF");
    _bt.println("");

    // Config
    _bt.print("cfgA period="); _bt.print(_dm->cfg.cfgA_period_s);
    _bt.print(" rxwin="); _bt.println(_dm->cfg.cfgA_rxwin_ms);

    _bt.print("cfgB period="); _bt.print(_dm->cfg.cfgB_period_s);
    _bt.print(" rxwin="); _bt.println(_dm->cfg.cfgB_rxwin_ms);

    // Auto
    _bt.print("auto en="); _bt.print(_dm->autoPump.enabled ? "1" : "0");
    _bt.print(" th_on="); _bt.print(_dm->autoPump.soil_on_threshold);
    _bt.print(" th_off="); _bt.print(_dm->autoPump.soil_off_threshold);
    _bt.print(" cool_s="); _bt.println(_dm->autoPump.cooldown_sec);

    _bt.println("");
  }

  static void setPage(const String& arg) {
    if (!_ui) { _bt.println("page: UiState not bound"); return; }

    if (arg == "home") _ui->page = UiPage::Home;
    else if (arg == "a") _ui->page = UiPage::NodeA_Details;
    else if (arg == "b") _ui->page = UiPage::NodeB_Details;
    else if (arg == "pump") _ui->page = UiPage::Relay;
    else if (arg == "setA") _ui->page = UiPage::Settings1;
    else if (arg == "setB") _ui->page = UiPage::Settings2;
    else { _bt.println("page: use home|a|b|pump|setA|setB"); return; }

    _ui->markDirty();
    _bt.print("page ok: "); _bt.println(arg);
  }

  static void handleLine(const String& s) {
    if (s == "help") { help(); return; }
    if (s == "status") { status(); return; }

    if (s.startsWith("page ")) {
      String arg = s.substring(5); arg.trim();
      setPage(arg);
      return;
    }

    if (s == "relay on") {
      if (_dm) _dm->relay_target_on = true;
      if (_fnRelay) _fnRelay(true);
      _bt.println("relay cmd: ON");
      return;
    }
    if (s == "relay off") {
      if (_dm) _dm->relay_target_on = false;
      if (_fnRelay) _fnRelay(false);
      _bt.println("relay cmd: OFF");
      return;
    }

    if (s.startsWith("cfgA")) {
      uint16_t p=0,w=0;
      int n = sscanf(s.c_str(), "cfgA %hu %hu", &p, &w);
      if (n != 2) { _bt.println("cfgA usage: cfgA <period_s> <rxwin_ms>"); return; }
      if (_dm) { _dm->cfg.cfgA_period_s = p; _dm->cfg.cfgA_rxwin_ms = w; }
      if (_fnCfgA) _fnCfgA(p, w);
      _bt.println("cfgA sent");
      return;
    }

    if (s.startsWith("cfgB")) {
      uint16_t p=0,w=0;
      int n = sscanf(s.c_str(), "cfgB %hu %hu", &p, &w);
      if (n != 2) { _bt.println("cfgB usage: cfgB <period_s> <rxwin_ms>"); return; }
      if (_dm) { _dm->cfg.cfgB_period_s = p; _dm->cfg.cfgB_rxwin_ms = w; }
      if (_fnCfgB) _fnCfgB(p, w);
      _bt.println("cfgB sent");
      return;
    }

    if (s == "auto on") {
      if (_dm) { _dm->autoPump.enabled = true; }
      _bt.println("auto=ON");
      return;
    }
    if (s == "auto off") {
      if (_dm) { _dm->autoPump.enabled = false; }
      _bt.println("auto=OFF");
      return;
    }

    if (s.startsWith("th ")) {
      uint16_t onv=0, offv=0;
      int n = sscanf(s.c_str(), "th %hu %hu", &onv, &offv);
      if (n != 2) { _bt.println("th usage: th <on> <off>"); return; }
      if (_dm) { _dm->autoPump.soil_on_threshold = onv; _dm->autoPump.soil_off_threshold = offv; }
      _bt.println("th updated");
      return;
    }

    if (s.startsWith("cool ")) {
      uint16_t v=0;
      int n = sscanf(s.c_str(), "cool %hu", &v);
      if (n != 1) { _bt.println("cool usage: cool <sec>"); return; }
      if (_dm) { _dm->autoPump.cooldown_sec = v; }
      _bt.println("cool updated");
      return;
    }

    _bt.println("unknown. type: help");
  }
};

BluetoothSerial BtPanel::_bt;
String BtPanel::_line;
DataModel* BtPanel::_dm = nullptr;
UiState* BtPanel::_ui = nullptr;
BtPanel::FnRelay BtPanel::_fnRelay = nullptr;
BtPanel::FnCfg BtPanel::_fnCfgA = nullptr;
BtPanel::FnCfg BtPanel::_fnCfgB = nullptr;