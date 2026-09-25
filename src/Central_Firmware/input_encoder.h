#pragma once
#include <Arduino.h>

struct EncoderConfig {
  int pinA = -1;
  int pinB = -1;
  int pinSw = -1;

  bool pullups = true;

  uint16_t debounceMs = 25;
  uint16_t longPressMs = 800;

  // Typical mechanical encoder gives 4 transitions per detent.
  uint8_t stepsPerDetent = 4;
};

struct EncoderEvents {
  int8_t delta = 0;       // detents, not raw edges
  bool shortPress = false;
  bool longPress = false;
};

class EncoderInput {
public:
  void begin(const EncoderConfig& cfg) {
    _cfg = cfg;

    if (_cfg.pullups) {
      pinMode(_cfg.pinA, INPUT_PULLUP);
      pinMode(_cfg.pinB, INPUT_PULLUP);
      pinMode(_cfg.pinSw, INPUT_PULLUP);
    } else {
      pinMode(_cfg.pinA, INPUT);
      pinMode(_cfg.pinB, INPUT);
      pinMode(_cfg.pinSw, INPUT);
    }

    uint8_t a = (uint8_t)digitalRead(_cfg.pinA);
    uint8_t b = (uint8_t)digitalRead(_cfg.pinB);
    _prevAB = (a << 1) | b;

    _accumSteps = 0;

    _btnStable = digitalRead(_cfg.pinSw);
    _btnLastRead = _btnStable;
    _btnLastChangeMs = millis();

    _pressing = false;
    _pressStartMs = 0;
    _longFired = false;
  }

  void poll(EncoderEvents& ev) {
    ev.delta = 0;
    ev.shortPress = false;
    ev.longPress = false;

    // ----- Quadrature decode (polling) -----
    // Table from prevAB<<2 | curAB : -1, 0, +1
    // Valid transitions only. Invalid transitions treated as 0.
    static const int8_t qdec[16] = {
      0, -1, +1,  0,
      +1, 0,  0, -1,
      -1, 0,  0, +1,
      0, +1, -1, 0
    };

    uint8_t a = (uint8_t)digitalRead(_cfg.pinA);
    uint8_t b = (uint8_t)digitalRead(_cfg.pinB);
    uint8_t curAB = (a << 1) | b;

    uint8_t idx = (uint8_t)((_prevAB << 2) | curAB);
    int8_t step = qdec[idx];
    _prevAB = curAB;

    if (step != 0) {
      _accumSteps += step;

      int8_t det = 0;
      const int8_t spd = (int8_t)_cfg.stepsPerDetent;

      while (_accumSteps >= spd) { _accumSteps -= spd; det++; }
      while (_accumSteps <= -spd) { _accumSteps += spd; det--; }

      if (det != 0) ev.delta = det;
    }

    // ----- Button debounce + short/long press -----
    uint32_t now = millis();
    int btn = digitalRead(_cfg.pinSw);

    if (btn != _btnLastRead) {
      _btnLastRead = btn;
      _btnLastChangeMs = now;
    }

    if ((uint32_t)(now - _btnLastChangeMs) >= _cfg.debounceMs) {
      if (btn != _btnStable) {
        _btnStable = btn;

        // Active low
        if (_btnStable == LOW) {
          _pressing = true;
          _pressStartMs = now;
          _longFired = false;
        } else {
          // released
          if (_pressing) {
            uint32_t held = now - _pressStartMs;
            if (!_longFired && held < _cfg.longPressMs) {
              ev.shortPress = true;
            }
          }
          _pressing = false;
        }
      }
    }

    if (_pressing && !_longFired) {
      if ((uint32_t)(now - _pressStartMs) >= _cfg.longPressMs) {
        _longFired = true;
        ev.longPress = true;
      }
    }
  }

private:
  EncoderConfig _cfg;

  uint8_t _prevAB = 0;
  int16_t _accumSteps = 0;

  int _btnStable = HIGH;
  int _btnLastRead = HIGH;
  uint32_t _btnLastChangeMs = 0;

  bool _pressing = false;
  uint32_t _pressStartMs = 0;
  bool _longFired = false;
};