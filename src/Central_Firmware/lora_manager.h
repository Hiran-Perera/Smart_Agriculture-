#pragma once
#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>

#include "payloads.h"
#include "lora_link.h"
#include "data_model.h"

struct LoRaConfig {
  long freq = 433E6;

  // KEEP THESE MATCHING YOUR WIRING
  int sck  = 18;
  int miso = 19;
  int mosi = 23;
  int cs   = 26;
  int rst  = 27;
  int dio0 = 14;

  int txPower = 17;
};

class LoRaManager {
public:
  void begin(const LoRaConfig& cfg) {
    _cfg = cfg;

    SPI.begin(_cfg.sck, _cfg.miso, _cfg.mosi);
    LoRa.setPins(_cfg.cs, _cfg.rst, _cfg.dio0);

    if (!LoRa.begin(_cfg.freq)) {
      Serial.println("GW LORA init fail");
      while (true) delay(1000);
    }

    LoRa.setTxPower(_cfg.txPower);
    LoRa.enableCrc();
    LoRa.receive();

    Serial.println("LoRaManager ready");
  }

  bool poll(DataModel& data) {
    bool changed = false;
    uint32_t now = millis();

    Link::RxPacket rx;
    if (!Link::read_enc(rx)) return false;

    Link::log_rx("GW", rx);

    if (rx.h.dst != Payload::GW_ID) return false;

    // ---------- TELEMETRY ----------
    if (rx.h.type == Payload::TELEMETRY) {
      // ACK telemetry fast
      send_rxack_(rx.h.src, rx.h.msgId);

      if (rx.h.src == Payload::NODE_A) {
        if (rx.h.len == sizeof(Payload::TelemetryA) && !is_dup_A_(rx.h.msgId)) {
          Payload::TelemetryA ta;
          memcpy(&ta, rx.plain, sizeof(ta));
          if (ta.ver == 1) {
            data.a.light_raw  = ta.light_raw;
            data.a.batt_mV    = ta.batt_mV;
            data.a.tempC_x100 = ta.tempC_x100;
            data.a.hum_x100   = ta.hum_x100;
            data.a.lastSeenMs = now;
            data.a.valid      = true;
            data.a.radio.rssi = (int16_t)rx.rssi;
            data.a.radio.snr  = rx.snr;
            data.a.rssi = rx.rssi;
            data.a.snr  = rx.snr;
            changed = true;
          }
        }

        // IMPORTANT: wait a bit after RXACK so node enters rx window
        if (_pend_cfgA) {
          delay(80);
          if (try_send_cfgA_()) changed = true;
        }
      }

      if (rx.h.src == Payload::NODE_B) {
        if (rx.h.len == sizeof(Payload::TelemetryB) && !is_dup_B_(rx.h.msgId)) {
          Payload::TelemetryB tb;
          memcpy(&tb, rx.plain, sizeof(tb));
          if (tb.ver == 1) {
            data.b.soil_raw    = tb.soil_raw;
            data.b.flow_hz_x10  = tb.flow_hz_x10;
            data.b.batt_mV      = tb.batt_mV;
            data.b.relay_state  = (tb.relay_state != 0);
            data.b.lastSeenMs   = now;
            data.b.valid        = true;
            data.b.radio.rssi   = (int16_t)rx.rssi;
            data.b.radio.snr    = rx.snr;
            data.b.rssi = rx.rssi;
            data.b.snr  = rx.snr;
            changed = true;
          }
        }

        // IMPORTANT: wait a bit after RXACK so node enters rx window
        if (_pend_relay || _pend_cfgB) {
          delay(80);
          if (_pend_relay) { if (try_send_relay_()) changed = true; }
          if (_pend_cfgB)  { if (try_send_cfgB_()) changed = true; }
        }
      }
    }

    // ---------- ACK_CMD ----------
    if (rx.h.type == Payload::ACK_CMD) {
      if (rx.h.src == Payload::NODE_A && rx.h.len == sizeof(Payload::AckCfgSleepA)) {
        Payload::AckCfgSleepA a;
        memcpy(&a, rx.plain, sizeof(a));
        Serial.print("GW ACK_CFG_A period_s="); Serial.print(a.applied_period_s);
        Serial.print(" rxwin_ms="); Serial.println(a.applied_rxwin_ms);

        if (_pend_cfgA && rx.h.msgId == _pend_cfgA_id) {
          _pend_cfgA = false;
          Serial.println("GW CFG_A success");
          changed = true;
        }
      }

      if (rx.h.src == Payload::NODE_B) {
        if (rx.h.len == sizeof(Payload::AckCfgSleepB)) {
          Payload::AckCfgSleepB a;
          memcpy(&a, rx.plain, sizeof(a));
          Serial.print("GW ACK_CFG_B period_s="); Serial.print(a.applied_period_s);
          Serial.print(" rxwin_ms="); Serial.println(a.applied_rxwin_ms);

          if (_pend_cfgB && rx.h.msgId == _pend_cfgB_id) {
            _pend_cfgB = false;
            Serial.println("GW CFG_B success");
            changed = true;
          }
        }

        if (rx.h.len == sizeof(Payload::AckRelay)) {
          Payload::AckRelay a;
          memcpy(&a, rx.plain, sizeof(a));
          Serial.print("GW ACK_RELAY state="); Serial.println((int)a.applied_state);

          data.b.relay_state = (a.applied_state != 0);
          data.b.valid = true;

          if (_pend_relay && rx.h.msgId == _pend_relay_id) {
            _pend_relay = false;
            Serial.println("GW RELAY success");
            changed = true;
          }
        }
      }
    }

    return changed;
  }

  void sendRelayTarget(bool on) {
    _pend_relay_data.relay_on = on ? 1 : 0;
    _pend_relay = true;
    _pend_relay_id = _msgId++;
    _pend_relay_tries = 0;
    Serial.print("GW RELAY pending target="); Serial.println(on ? 1 : 0);
  }

  void sendCfgA(uint16_t period_s, uint16_t rxwin_ms) {
    _pend_cfgA_data.period_s = period_s;
    _pend_cfgA_data.rxwin_ms = rxwin_ms;
    _pend_cfgA = true;
    _pend_cfgA_id = _msgId++;
    _pend_cfgA_tries = 0;
    Serial.println("GW CFG_A pending");
  }

  void sendCfgB(uint16_t period_s, uint16_t rxwin_ms) {
    _pend_cfgB_data.period_s = period_s;
    _pend_cfgB_data.rxwin_ms = rxwin_ms;
    _pend_cfgB = true;
    _pend_cfgB_id = _msgId++;
    _pend_cfgB_tries = 0;
    Serial.println("GW CFG_B pending");
  }

private:
  LoRaConfig _cfg;
  uint32_t _msgId = 100;

  uint32_t _lastIdA = 0;
  uint32_t _lastIdB = 0;

  bool _pend_cfgA = false;
  Payload::CmdCfgSleepA _pend_cfgA_data = {};
  uint32_t _pend_cfgA_id = 0;
  uint8_t  _pend_cfgA_tries = 0;

  bool _pend_cfgB = false;
  Payload::CmdCfgSleepB _pend_cfgB_data = {};
  uint32_t _pend_cfgB_id = 0;
  uint8_t  _pend_cfgB_tries = 0;

  bool _pend_relay = false;
  Payload::CmdRelay _pend_relay_data = {};
  uint32_t _pend_relay_id = 0;
  uint8_t  _pend_relay_tries = 0;

  static bool is_dup_(uint32_t& lastId, uint32_t id) {
    if (id == lastId) return true;
    lastId = id;
    return false;
  }
  bool is_dup_A_(uint32_t id) { return is_dup_(_lastIdA, id); }
  bool is_dup_B_(uint32_t id) { return is_dup_(_lastIdB, id); }

  bool send_rxack_(uint8_t dst, uint32_t msgId) {
    uint8_t b = 0;
    bool ok = Link::send_enc(Payload::GW_ID, dst, Payload::RXACK, 2, msgId, &b, 1, true);
    Link::log_tx("GW", "RXACK", ok, Payload::GW_ID, dst, Payload::RXACK, 2, msgId);
    return ok;
  }

  bool try_send_cfgA_() {
    if (!_pend_cfgA) return false;
    if (_pend_cfgA_tries >= 3) {
      Serial.println("GW CFG_A failed");
      _pend_cfgA = false;
      return true;
    }
    bool ok = Link::send_enc(Payload::GW_ID, Payload::NODE_A, Payload::CMD_CFG_A, 2, _pend_cfgA_id,
                             (uint8_t*)&_pend_cfgA_data, sizeof(_pend_cfgA_data), true);
    Link::log_tx("GW", "CMD_CFG_A", ok, Payload::GW_ID, Payload::NODE_A, Payload::CMD_CFG_A, 2, _pend_cfgA_id);
    _pend_cfgA_tries++;
    return true;
  }

  bool try_send_cfgB_() {
    if (!_pend_cfgB) return false;
    if (_pend_cfgB_tries >= 3) {
      Serial.println("GW CFG_B failed");
      _pend_cfgB = false;
      return true;
    }
    bool ok = Link::send_enc(Payload::GW_ID, Payload::NODE_B, Payload::CMD_CFG_B, 2, _pend_cfgB_id,
                             (uint8_t*)&_pend_cfgB_data, sizeof(_pend_cfgB_data), true);
    Link::log_tx("GW", "CMD_CFG_B", ok, Payload::GW_ID, Payload::NODE_B, Payload::CMD_CFG_B, 2, _pend_cfgB_id);
    _pend_cfgB_tries++;
    return true;
  }

  bool try_send_relay_() {
    if (!_pend_relay) return false;
    if (_pend_relay_tries >= 3) {
      Serial.println("GW RELAY failed");
      _pend_relay = false;
      return true;
    }
    bool ok = Link::send_enc(Payload::GW_ID, Payload::NODE_B, Payload::CMD_RELAY, 2, _pend_relay_id,
                             (uint8_t*)&_pend_relay_data, sizeof(_pend_relay_data), true);
    Link::log_tx("GW", "CMD_RELAY", ok, Payload::GW_ID, Payload::NODE_B, Payload::CMD_RELAY, 2, _pend_relay_id);
    _pend_relay_tries++;
    return true;
  }
};