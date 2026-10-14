#pragma once
#include <Arduino.h>

// Interrupt-driven rotary encoder + push button.
//
// The A/B lines and the switch are decoded inside GPIO interrupts, so rotation
// and presses are captured even while the main loop is busy (TFT redraws,
// LoRa delays, Bluetooth). poll() only drains what the ISRs already counted.

struct EncoderConfig {
  int pinA = -1;
  int pinB = -1;
  int pinSw = -1;

  bool pullups = true;

  uint16_t debounceMs = 20;
  uint16_t longPressMs = 800;

  // Typical mechanical encoder gives 4 transitions per detent.
  uint8_t stepsPerDetent = 4;
};

struct EncoderEvents {
  int8_t delta = 0;       // detents, not raw edges
  bool shortPress = false;
  bool longPress = false;
};

namespace EncIsr {

static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

static int pinA = -1, pinB = -1, pinSw = -1;
static uint8_t  stepsPerDetent = 4;
static uint32_t debounceUs = 20000;
static uint32_t longPressMs = 800;

// Valid Gray-code transitions only; anything else counts as 0.
static const DRAM_ATTR int8_t qdec[16] = {
   0, -1, +1,  0,
  +1,  0,  0, -1,
  -1,  0,  0, +1,
   0, +1, -1,  0
};

static volatile uint8_t  prevAB = 0;
static volatile int16_t  accum = 0;     // raw edges, -N..+N within a detent
static volatile int16_t  detents = 0;   // completed detents not yet consumed

static volatile uint8_t  btnLevel = HIGH;
static volatile uint32_t btnLastEdgeUs = 0;
static volatile uint32_t pressStartMs = 0;
static volatile bool     pressing = false;
static volatile bool     longFired = false;
static volatile uint8_t  shortCount = 0;
static volatile uint8_t  longCount = 0;

static void IRAM_ATTR isrAB() {
  uint8_t cur = (uint8_t)((digitalRead(pinA) << 1) | digitalRead(pinB));

  portENTER_CRITICAL_ISR(&mux);
  int8_t step = qdec[(prevAB << 2) | cur];
  prevAB = cur;

  if (step != 0) {
    accum += step;
    if (accum >= (int16_t)stepsPerDetent)       { accum -= stepsPerDetent; detents++; }
    else if (accum <= -(int16_t)stepsPerDetent) { accum += stepsPerDetent; detents--; }
  }
  portEXIT_CRITICAL_ISR(&mux);
}

static void IRAM_ATTR isrSw() {
  uint32_t nowUs = micros();
  uint8_t level = (uint8_t)digitalRead(pinSw);

  portENTER_CRITICAL_ISR(&mux);
  if ((uint32_t)(nowUs - btnLastEdgeUs) >= debounceUs && level != btnLevel) {
    btnLastEdgeUs = nowUs;
    btnLevel = level;
    uint32_t nowMs = nowUs / 1000;

    if (level == LOW) {
      pressing = true;
      longFired = false;
      pressStartMs = nowMs;
    } else {
      if (pressing && !longFired && (uint32_t)(nowMs - pressStartMs) < longPressMs) {
        if (shortCount < 255) shortCount++;
      }
      pressing = false;
    }
  }
  portEXIT_CRITICAL_ISR(&mux);
}

} // namespace EncIsr

class EncoderInput {
public:
  void begin(const EncoderConfig& cfg) {
    _cfg = cfg;

    const uint8_t mode = _cfg.pullups ? INPUT_PULLUP : INPUT;
    pinMode(_cfg.pinA, mode);
    pinMode(_cfg.pinB, mode);
    pinMode(_cfg.pinSw, mode);

    EncIsr::pinA = _cfg.pinA;
    EncIsr::pinB = _cfg.pinB;
    EncIsr::pinSw = _cfg.pinSw;
    EncIsr::stepsPerDetent = _cfg.stepsPerDetent ? _cfg.stepsPerDetent : 4;
    EncIsr::debounceUs = (uint32_t)_cfg.debounceMs * 1000UL;
    EncIsr::longPressMs = _cfg.longPressMs;

    EncIsr::prevAB = (uint8_t)((digitalRead(_cfg.pinA) << 1) | digitalRead(_cfg.pinB));
    EncIsr::accum = 0;
    EncIsr::detents = 0;
    EncIsr::btnLevel = (uint8_t)digitalRead(_cfg.pinSw);
    EncIsr::btnLastEdgeUs = micros();
    EncIsr::pressing = false;
    EncIsr::longFired = false;
    EncIsr::shortCount = 0;
    EncIsr::longCount = 0;

    attachInterrupt(digitalPinToInterrupt(_cfg.pinA), EncIsr::isrAB, CHANGE);
    attachInterrupt(digitalPinToInterrupt(_cfg.pinB), EncIsr::isrAB, CHANGE);
    attachInterrupt(digitalPinToInterrupt(_cfg.pinSw), EncIsr::isrSw, CHANGE);
  }

  // True when there is something waiting for poll() (cheap, no locking).
  bool pending() const {
    return EncIsr::detents != 0 || EncIsr::shortCount != 0 || EncIsr::longCount != 0 ||
           (EncIsr::pressing && !EncIsr::longFired);
  }

  void poll(EncoderEvents& ev) {
    ev.delta = 0;
    ev.shortPress = false;
    ev.longPress = false;

    uint32_t nowMs = millis();

    portENTER_CRITICAL(&EncIsr::mux);

    // A held button turns into a long press as soon as the threshold passes.
    if (EncIsr::pressing && !EncIsr::longFired &&
        (uint32_t)(nowMs - EncIsr::pressStartMs) >= EncIsr::longPressMs) {
      EncIsr::longFired = true;
      if (EncIsr::longCount < 255) EncIsr::longCount++;
    }

    int16_t d = EncIsr::detents;
    EncIsr::detents = 0;
    if (d > 127) d = 127;
    if (d < -127) d = -127;
    ev.delta = (int8_t)d;

    ev.shortPress = EncIsr::shortCount > 0;
    ev.longPress  = EncIsr::longCount > 0;
    EncIsr::shortCount = 0;
    EncIsr::longCount = 0;

    portEXIT_CRITICAL(&EncIsr::mux);
  }

private:
  EncoderConfig _cfg;
};
