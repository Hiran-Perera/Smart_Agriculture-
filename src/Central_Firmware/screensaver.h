#pragma once
#include <Arduino.h>
#include "tft_driver.h"
#include "ui_state.h"
#include "logo_fieldlink.h"

// SmartAgro FIELDLINK screensaver.
// It slides in over the current page (like a page change) and slides back out
// on the first knob touch, revealing the page again. While shown it is static
// (the LCD has no burn-in), so it costs nothing.
struct ScreenSaver {
  // Draws the UI page underneath at a horizontal offset (into the frame buffer).
  using DrawUnder = void (*)(int16_t xoff, uint32_t now);

  enum State : uint8_t { OFF, ENTER, SHOW, EXIT };

  static const uint16_t SLIDE_IN_MS  = 420;
  static const uint16_t SLIDE_OUT_MS = 260;

  State st = OFF;
  uint32_t t0 = 0;
  bool drawn = false;

  void start(uint32_t now) { st = ENTER; t0 = now; drawn = false; }
  void stop(uint32_t now) {
    if (st == OFF || st == EXIT) return;
    // leaving mid-entry: continue from where the slide currently is
    float p = (st == ENTER) ? ui_ease(now, t0, SLIDE_IN_MS) : 1.0f;
    st = EXIT;
    t0 = now - (uint32_t)((1.0f - p) * SLIDE_OUT_MS);
  }
  bool active() const  { return st != OFF; }
  bool exiting() const { return st == EXIT; }

  // Call every loop while active().
  void tick(TftDriver& t, uint32_t now, DrawUnder under) {
    const int16_t W = t.width();

    switch (st) {
      case ENTER: {
        const int16_t off = (int16_t)(ui_ease(now, t0, SLIDE_IN_MS) * W);
        frame_(t, now, under, (int16_t)(W - off), (int16_t)(-off));
        if ((uint32_t)(now - t0) >= SLIDE_IN_MS) { st = SHOW; drawn = true; }
      } break;

      case SHOW:
        if (!drawn) { frame_(t, now, under, 0, W); drawn = true; }
        break;

      case EXIT: {
        const int16_t off = (int16_t)(ui_ease(now, t0, SLIDE_OUT_MS) * W);
        frame_(t, now, under, off, (int16_t)(off - W));
        if ((uint32_t)(now - t0) >= SLIDE_OUT_MS) st = OFF;
      } break;

      default: break;
    }
  }

private:
  void frame_(TftDriver& t, uint32_t now, DrawUnder under, int16_t saverX, int16_t pageX) {
    t.beginFrame();
    t.fillScreen(Col::BG);
    if (under && pageX > -t.width() && pageX < t.width()) under(pageX, now);
    draw_saver_(t, saverX);
    t.endFrame();
  }

  static void draw_saver_(TftDriver& t, int16_t xoff) {
    const int16_t W = t.width(), H = t.height();
    if (xoff >= W || xoff <= -W) return;
    t.fillRect(xoff, 0, W, H, FIELDLINK_BG);
    const int16_t x = xoff + (W - (int16_t)FIELDLINK_W) / 2;
    const int16_t y = (H - (int16_t)FIELDLINK_H) / 2;
    t.drawRGB565_P(x, y, logo_fieldlink, FIELDLINK_W, FIELDLINK_H);
  }
};
