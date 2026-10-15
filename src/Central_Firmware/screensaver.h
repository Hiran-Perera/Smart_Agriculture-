#pragma once
#include <Arduino.h>
#include <Adafruit_ST77xx.h>
#include "tft_driver.h"

// Black screen with a mint-green gateway glyph (dot rows + bar) and a sprout.
// The dots ripple slowly; only that small area is redrawn.
struct ScreenSaver {
  static constexpr uint16_t MINT = 0x6795;   // bright mint green
  static constexpr uint16_t DIM  = 0x1AC9;   // dark green for idle dots

  bool drawn = false;
  uint32_t lastAnimMs = 0;
  uint8_t phase = 0;

  void reset(uint32_t now) { drawn = false; lastAnimMs = now; phase = 0; }

  bool tick(TftDriver& t, uint32_t now) {
    if (!drawn) {
      drawn = true;
      t.beginFrame();
      t.fill(ST77XX_BLACK);
      draw_leaf_(t);
      draw_glyph_(t);
      t.endFrame();
      return true;
    }

    if ((uint32_t)(now - lastAnimMs) < 400) return false;
    lastAnimMs = now;
    phase = (uint8_t)((phase + 1) % 8);
    draw_glyph_(t);
    return true;
  }

private:
  // Glyph centred around x=48, y=64: 4 dots, a wide bar, 4 dots.
  void draw_glyph_(TftDriver& t) {
    const int16_t d = 7, gap = 4;
    const int16_t rowW = 4 * d + 3 * gap;          // 40
    const int16_t x0 = 48 - rowW / 2;
    const int16_t y0 = 64 - (3 * d + 2 * gap) / 2; // 3 rows

    for (uint8_t i = 0; i < 4; i++) {
      t.fillRect(x0 + i * (d + gap), y0, d, d, lit_(i));
      t.fillRect(x0 + i * (d + gap), y0 + 2 * (d + gap), d, d, lit_((uint8_t)(i + 4)));
    }
    // middle bar, narrower than the rows like the reference
    t.fillRect(x0 + d + gap / 2, y0 + d + gap, 2 * d + gap, d, MINT);
  }

  // Dim dots with a bright wave travelling over the 8 dot positions.
  uint16_t lit_(uint8_t idx) const {
    uint8_t dist = (uint8_t)((idx + 8 - phase) % 8);
    return dist < 3 ? MINT : DIM;
  }

  // Two-leaf sprout centred around x=112, y=64.
  void draw_leaf_(TftDriver& t) {
    const int16_t cx = 112, by = 82;
    // left leaf (points up-left)
    t.fillTriangle(cx - 1, by,      cx - 22, by - 6,  cx - 12, by - 26, MINT);
    t.fillTriangle(cx - 1, by,      cx - 12, by - 26, cx - 1,  by - 14,  MINT);
    t.fillCircle(cx - 12, by - 10, 8, MINT);
    // right leaf (larger, points up-right)
    t.fillTriangle(cx + 1, by - 4,  cx + 24, by - 12, cx + 12, by - 36, MINT);
    t.fillTriangle(cx + 1, by - 4,  cx + 12, by - 36, cx + 1,  by - 22, MINT);
    t.fillCircle(cx + 12, by - 18, 10, MINT);
    // vein cut-outs
    t.drawLine(cx - 2, by - 2, cx - 14, by - 18, ST77XX_BLACK);
    t.drawLine(cx + 2, by - 6, cx + 14, by - 26, ST77XX_BLACK);
  }
};
