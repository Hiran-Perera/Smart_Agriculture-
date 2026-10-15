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

// Gateway radio: receives telemetry, and queues commands for the sleeping nodes.
// A queued command is sent ~80 ms after the node's next telemetry (when the node
// opens its receive window) and retried on up to 3 wake-ups until acknowledged.
class LoRaManager {
public:
  void begin(const LoRaConfig& cfg, DataModel& data) {
    _cfg = cfg;
    _dm = &data;

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
    uint32_t now = millis();

    // Deferred sends happen here instead of delay(), so the UI loop is never blocked.
    bool changed = service_deferred_(now);

    Link::RxPacket rx;
    if (!Link::read_enc(rx)) return changed;

    Link::log_rx("GW", rx);

    if (rx.h.dst != Payload::GW_ID) return changed;

    // ---------- EXTRA SENSORS (sent just before the node's TELEMETRY, no RXACK) ----------
    if (rx.h.type == Payload::TELEMETRY_X && rx.h.len == sizeof(Payload::TelemetryX)) {
      Payload::TelemetryX tx;
      memcpy(&tx, rx.plain, sizeof(tx));
      ExtVal* dst = nullptr;
      if (rx.h.src == Payload::NODE_A) { dst = data.a.ext; data.a.extMs = now; }
      if (rx.h.src == Payload::NODE_B) { dst = data.b.ext; data.b.extMs = now; }
      if (dst && tx.ver == 2) {
        for (uint8_t i = 0; i < Payload::EXT_SLOTS; i++) {
          dst[i].type  = tx.s[i].type;
          dst[i].pin   = tx.s[i].pin;
          dst[i].v1    = tx.s[i].v1;
          dst[i].v2    = tx.s[i].v2;
          dst[i].valid = (tx.s[i].type != Payload::SENS_NONE);
        }
        changed = true;
      }
      return changed;
    }

    // ---------- TELEMETRY ----------
    if (rx.h.type == Payload::TELEMETRY) {
      send_rxack_(rx.h.src, rx.h.msgId);

      if (rx.h.src == Payload::NODE_A) {
        if (rx.h.len == sizeof(Payload::TelemetryA) && !is_dup_(_lastIdA, rx.h.msgId)) {
          Payload::TelemetryA ta;
          memcpy(&ta, rx.plain, sizeof(ta));
          if (ta.ver == 1) {
            note_interval_(data.a.lastSeenMs, data.a.intervalMs, data.applied[0], now);
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
            // Extra sensors arrive right before telemetry; if none did, they are switched off.
            if ((uint32_t)(now - data.a.extMs) > 3000) clear_ext_(data.a.ext);
            changed = true;
          }
        }
        arm_deferred_(Payload::NODE_A, now);
      }

      if (rx.h.src == Payload::NODE_B) {
        if (rx.h.len == sizeof(Payload::TelemetryB) && !is_dup_(_lastIdB, rx.h.msgId)) {
          Payload::TelemetryB tb;
          memcpy(&tb, rx.plain, sizeof(tb));
          if (tb.ver == 1) {
            note_interval_(data.b.lastSeenMs, data.b.intervalMs, data.applied[1], now);
            data.b.soil_raw     = tb.soil_raw;
            data.b.flow_hz_x10  = tb.flow_hz_x10;
            data.b.batt_mV      = tb.batt_mV;
            data.b.relay_state  = (tb.relay_state != 0);
            data.b.lastSeenMs   = now;
            data.b.valid        = true;
            data.b.radio.rssi   = (int16_t)rx.rssi;
            data.b.radio.snr    = rx.snr;
            data.b.rssi = rx.rssi;
            data.b.snr  = rx.snr;
            if ((uint32_t)(now - data.b.extMs) > 3000) clear_ext_(data.b.ext);
            changed = true;
          }
        }
        arm_deferred_(Payload::NODE_B, now);
      }
    }

    // ---------- ACK_CMD ----------
    if (rx.h.type == Payload::ACK_CMD) {
      for (uint8_t k = 0; k < CMD_COUNT; k++) {
        Pend& p = _pend[k];
        if (!p.active || p.dst != rx.h.src || p.id != rx.h.msgId) continue;
        if (handle_ack_(k, rx, data, now)) changed = true;
        break;
      }
    }

    return changed;
  }

  // ---------- commands (all delivered on the node's next wake-up) ----------
  void sendRelayTarget(bool on) {
    Payload::CmdRelay c; c.relay_on = on ? 1 : 0;
    queue_(CMD_RELAY, Payload::NODE_B, Payload::CMD_RELAY, &c, sizeof(c));
  }

  void sendCfgA(uint16_t period_s, uint16_t rxwin_ms) {
    Payload::CmdCfgSleepA c; c.period_s = period_s; c.rxwin_ms = rxwin_ms;
    queue_(CMD_CFG_A, Payload::NODE_A, Payload::CMD_CFG_A, &c, sizeof(c));
  }

  void sendCfgB(uint16_t period_s, uint16_t rxwin_ms) {
    Payload::CmdCfgSleepB c; c.period_s = period_s; c.rxwin_ms = rxwin_ms;
    queue_(CMD_CFG_B, Payload::NODE_B, Payload::CMD_CFG_B, &c, sizeof(c));
  }

  // n: 0 = Node A, 1 = Node B
  void sendSensorCfg(uint8_t n, const Payload::CmdSensorCfg& cfg) {
    queue_(n ? CMD_SENS_B : CMD_SENS_A, n ? Payload::NODE_B : Payload::NODE_A,
           Payload::CMD_SENSOR_CFG, &cfg, sizeof(cfg));
  }

  void sendOutputs(uint8_t n, uint8_t states) {
    Payload::CmdExtOut c; c.ver = 1; c.states = states;
    queue_(n ? CMD_OUT_B : CMD_OUT_A, n ? Payload::NODE_B : Payload::NODE_A,
           Payload::CMD_EXT_OUT, &c, sizeof(c));
  }

private:
  enum CmdSlot : uint8_t { CMD_RELAY, CMD_CFG_A, CMD_CFG_B, CMD_SENS_A, CMD_SENS_B, CMD_OUT_A, CMD_OUT_B, CMD_COUNT };

  struct Pend {
    bool     active = false;
    uint8_t  dst = 0;
    uint8_t  type = 0;
    uint32_t id = 0;
    uint8_t  tries = 0;
    uint8_t  len = 0;
    uint8_t  buf[16];
    uint16_t sentInWindow = 0;   // wake-up window it was last sent in
  };

  static constexpr uint8_t MAX_TRIES = 3;
  static constexpr uint32_t SEND_DELAY_MS = 80;   // node needs a moment after RXACK to listen

  LoRaConfig _cfg;
  DataModel* _dm = nullptr;
  uint32_t _msgId = 100;
  uint32_t _lastIdA = 0, _lastIdB = 0;

  Pend _pend[CMD_COUNT];
  uint16_t _window[3] = {0, 0, 0};   // per node: counts telemetry wake-ups (index = node id)
  bool _deferA = false, _deferB = false;
  uint32_t _deferAAt = 0, _deferBAt = 0;

  AckState* ack_for_(uint8_t k) {
    if (!_dm) return nullptr;
    switch (k) {
      case CMD_RELAY:  return &_dm->relayAck;
      case CMD_CFG_A:  return &_dm->cfgAAck;
      case CMD_CFG_B:  return &_dm->cfgBAck;
      case CMD_SENS_A: return &_dm->sensAck[0];
      case CMD_SENS_B: return &_dm->sensAck[1];
      case CMD_OUT_A:  return &_dm->outAck[0];
      case CMD_OUT_B:  return &_dm->outAck[1];
      default:         return nullptr;
    }
  }

  static const char* name_(uint8_t k) {
    static const char* N[CMD_COUNT] = { "CMD_RELAY", "CMD_CFG_A", "CMD_CFG_B", "CMD_SENSOR_A",
                                        "CMD_SENSOR_B", "CMD_EXT_OUT_A", "CMD_EXT_OUT_B" };
    return k < CMD_COUNT ? N[k] : "?";
  }

  void queue_(uint8_t k, uint8_t dst, uint8_t type, const void* payload, uint8_t len) {
    Pend& p = _pend[k];
    p.active = true;
    p.dst = dst;
    p.type = type;
    p.id = _msgId++;
    p.tries = 0;
    p.sentInWindow = 0;
    p.len = len;
    memcpy(p.buf, payload, len);

    if (AckState* a = ack_for_(k)) { a->pending = true; a->ok = false; a->retriesUsed = 0; a->lastSendMs = millis(); }
    Serial.print("GW "); Serial.print(name_(k)); Serial.println(" queued");
  }

  void arm_deferred_(uint8_t node, uint32_t now) {
    if (++_window[node] == 0) _window[node] = 1;
    for (uint8_t k = 0; k < CMD_COUNT; k++) {
      if (!_pend[k].active || _pend[k].dst != node) continue;
      if (node == Payload::NODE_A) { _deferA = true; _deferAAt = now + SEND_DELAY_MS; }
      else                         { _deferB = true; _deferBAt = now + SEND_DELAY_MS; }
      return;
    }
  }

  bool service_deferred_(uint32_t now) {
    bool changed = false;
    if (_deferA && (int32_t)(now - _deferAAt) >= 0) {
      _deferA = false;
      if (send_next_for_(Payload::NODE_A, now)) { changed = true; _deferA = true; _deferAAt = now + 10; }
    }
    if (_deferB && (int32_t)(now - _deferBAt) >= 0) {
      _deferB = false;
      if (send_next_for_(Payload::NODE_B, now)) { changed = true; _deferB = true; _deferBAt = now + 10; }
    }
    return changed;
  }

  // Sends one queued command for `node` per call (the UI gets a frame in between).
  // Returns false when nothing is left to send in this wake-up window.
  bool send_next_for_(uint8_t node, uint32_t now) {
    for (uint8_t k = 0; k < CMD_COUNT; k++) {
      Pend& p = _pend[k];
      if (!p.active || p.dst != node || p.sentInWindow == _window[node]) continue;
      p.sentInWindow = _window[node];

      AckState* a = ack_for_(k);
      if (p.tries >= MAX_TRIES) {
        p.active = false;
        if (a) { a->pending = false; a->ok = false; }
        Serial.print("GW "); Serial.print(name_(k)); Serial.println(" failed");
        return true;
      }

      // Blocking TX: the next packet may only start once this one is on air.
      bool ok = Link::send_enc(Payload::GW_ID, p.dst, p.type, 2, p.id, p.buf, p.len, false);
      Link::log_tx("GW", name_(k), ok, Payload::GW_ID, p.dst, p.type, 2, p.id);
      p.tries++;
      if (a) a->retriesUsed = p.tries;
      return true;
    }
    return false;
  }

  // Ack for pending command k (msgId and source already matched). Lengths tell the acks apart.
  bool handle_ack_(uint8_t k, const Link::RxPacket& rx, DataModel& data, uint32_t now) {
    Pend& p = _pend[k];
    bool ok = true;

    switch (k) {
      case CMD_RELAY: {
        if (rx.h.len != sizeof(Payload::AckRelay)) return false;
        Payload::AckRelay a; memcpy(&a, rx.plain, sizeof(a));
        data.b.relay_state = (a.applied_state != 0);
        data.b.valid = true;
      } break;

      case CMD_CFG_A: case CMD_CFG_B: {
        if (rx.h.len != sizeof(Payload::AckCfgSleepA)) return false;
        Payload::AckCfgSleepA a; memcpy(&a, rx.plain, sizeof(a));
        NodeTiming& t = data.applied[k == CMD_CFG_B ? 1 : 0];
        t.period_s = a.applied_period_s;
        t.rxwin_ms = a.applied_rxwin_ms;
        data.timingDirty = true;
      } break;

      case CMD_SENS_A: case CMD_SENS_B: {
        if (rx.h.len != sizeof(Payload::AckSensorCfg)) return false;
        Payload::AckSensorCfg a; memcpy(&a, rx.plain, sizeof(a));
        ok = (a.ok != 0);
        // the node now runs this assignment
        if (ok) memcpy(&data.sensSent[k == CMD_SENS_B ? 1 : 0], p.buf, sizeof(Payload::CmdSensorCfg));
      } break;

      case CMD_OUT_A: case CMD_OUT_B: {
        if (rx.h.len != sizeof(Payload::AckExtOut)) return false;
        Payload::AckExtOut a; memcpy(&a, rx.plain, sizeof(a));
        // reflect the applied output states right away (telemetry confirms later)
        ExtVal* e = (k == CMD_OUT_B) ? data.b.ext : data.a.ext;
        for (uint8_t i = 0; i < Payload::EXT_SLOTS; i++) {
          if (a.outMask & (1 << i)) e[i].v1 = (a.states & (1 << i)) ? 1 : 0;
        }
      } break;

      default: return false;
    }

    p.active = false;
    if (AckState* s = ack_for_(k)) { s->pending = false; s->ok = ok; s->lastAckMs = now; }
    Serial.print("GW "); Serial.print(name_(k)); Serial.println(ok ? " success" : " rejected");
    return true;
  }

  // Smoothed gap between telemetry packets. Gaps that look like missed packets are ignored.
  static void note_interval_(uint32_t lastSeen, uint32_t& interval, const NodeTiming& t, uint32_t now) {
    if (lastSeen == 0) return;
    const uint32_t gap = now - lastSeen;
    const uint32_t expect = (uint32_t)t.period_s * 1000UL + t.rxwin_ms + 2500UL;
    if (gap < 1000 || gap > expect + expect / 2 + 5000) return;
    interval = interval ? (interval * 3 + gap) / 4 : gap;
  }

  static bool is_dup_(uint32_t& lastId, uint32_t id) {
    if (id == lastId) return true;
    lastId = id;
    return false;
  }

  static void clear_ext_(ExtVal* e) {
    for (uint8_t i = 0; i < Payload::EXT_SLOTS; i++) e[i].valid = false;
  }

  bool send_rxack_(uint8_t dst, uint32_t msgId) {
    uint8_t b = 0;
    bool ok = Link::send_enc(Payload::GW_ID, dst, Payload::RXACK, 2, msgId, &b, 1, true);
    Link::log_tx("GW", "RXACK", ok, Payload::GW_ID, dst, Payload::RXACK, 2, msgId);
    return ok;
  }
};
