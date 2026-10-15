#pragma once
#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <Adafruit_ST77xx.h>
#include <Fonts/FreeSansBold12pt7b.h>
#include <Fonts/FreeSansBold18pt7b.h>
#include <Fonts/FreeSansBold9pt7b.h>

#ifndef ST77XX_DARKGREY
#define ST77XX_DARKGREY 0x7BEF
#endif
#ifndef ST77XX_LIGHTGREY
#define ST77XX_LIGHTGREY 0xC618
#endif

// UI palette (RGB565): black background with mint green accents.
namespace Col {
static const uint16_t BG      = 0x0000;
static const uint16_t ACCENT  = 0x07E0;   // bright green (same as the Home screen)
static const uint16_t ACC_DIM = 0x0320;   // dark green
static const uint16_t PANEL   = 0x0861;   // near-black panel fill
static const uint16_t BORDER  = 0x2945;   // dark grey outline
static const uint16_t TEXT    = 0xFFFF;
static const uint16_t MUTED   = 0x9CD3;   // grey text
static const uint16_t WARN    = 0xFE60;   // amber
static const uint16_t BAD     = 0xF986;   // soft red
static const uint16_t INFO    = 0x5D7F;   // light blue
static const uint16_t EDIT    = 0x05EB;   // vibrant green (0,190,90): field being edited
static const uint16_t EDIT_BG = 0x0204;   // deep green (0,64,32): row being edited
}

struct TftConfig {
  int cs = -1;
  int dc = -1;
  int rst = -1;
  uint8_t rotation = 1;
};

// All drawing goes into an off-screen frame buffer (when RAM allows) and
// endFrame() pushes the whole frame at once: no flicker, no half drawn pages.
class TftDriver {
public:
  TftDriver() {}

  void begin(const TftConfig& cfg) {
    if (_tft) { delete _tft; _tft = nullptr; }

    _tft = new Adafruit_ST7735(cfg.cs, cfg.dc, cfg.rst);
    _tft->initR(INITR_BLACKTAB);
    _tft->setRotation(cfg.rotation);
    _tft->fillScreen(Col::BG);
    _tft->setTextWrap(false);

    if (!_canvas) {
      _canvas = new GFXcanvas16(_tft->width(), _tft->height());
      if (!_canvas->getBuffer()) { delete _canvas; _canvas = nullptr; }
    }
    if (_canvas) _canvas->setTextWrap(false);
    Serial.println(_canvas ? "TFT: frame buffer on" : "TFT: no RAM for frame buffer, drawing direct");
  }

  int16_t width()  const { return _tft ? _tft->width()  : 160; }
  int16_t height() const { return _tft ? _tft->height() : 128; }

  void beginFrame() {}
  void beginFrame(bool) { beginFrame(); }
  void endFrame() {
    if (_tft && _canvas) _tft->drawRGBBitmap(0, 0, _canvas->getBuffer(), _canvas->width(), _canvas->height());
  }

  Adafruit_GFX& gfx() { return _canvas ? (Adafruit_GFX&)*_canvas : (Adafruit_GFX&)*_tft; }

  // ---------- primitives ----------
  void fillScreen(uint16_t c) { if (_tft) gfx().fillScreen(c); }
  void fill(uint16_t c) { fillScreen(c); }
  void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t c) { if (_tft) gfx().fillRect(x, y, w, h, c); }
  void drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t c) { if (_tft) gfx().drawRect(x, y, w, h, c); }
  void fillRoundRect(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t c) { if (_tft) gfx().fillRoundRect(x, y, w, h, r, c); }
  void drawRoundRect(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t c) { if (_tft) gfx().drawRoundRect(x, y, w, h, r, c); }
  void fillCircle(int16_t x, int16_t y, int16_t r, uint16_t c) { if (_tft) gfx().fillCircle(x, y, r, c); }
  void drawCircle(int16_t x, int16_t y, int16_t r, uint16_t c) { if (_tft) gfx().drawCircle(x, y, r, c); }
  void fillTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t c) {
    if (_tft) gfx().fillTriangle(x0, y0, x1, y1, x2, y2, c);
  }
  void drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t c) { if (_tft) gfx().drawLine(x0, y0, x1, y1, c); }
  void drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t c) { if (_tft) gfx().drawFastHLine(x, y, w, c); }

  // ---------- text (built-in 6x8 font) ----------
  void setTextSmall()  { if (_tft) gfx().setTextSize(1); }
  void setTextMedium() { if (_tft) gfx().setTextSize(2); }
  void setTextLarge()  { if (_tft) gfx().setTextSize(3); }

  void drawText(int16_t x, int16_t y, const char* s, uint16_t fg) {
    if (!_tft) return;
    Adafruit_GFX& g = gfx();
    g.setFont(nullptr);
    g.setTextSize(1);
    g.setCursor(x, y);
    g.setTextColor(fg);
    g.print(s);
  }
  void drawText(int16_t x, int16_t y, const char* s, uint16_t fg, uint16_t) { drawText(x, y, s, fg); }
  void drawTextNoBg(int16_t x, int16_t y, const char* s, uint16_t fg) { drawText(x, y, s, fg); }

  int16_t textWidth(const char* s) { return s ? (int16_t)strlen(s) * 6 : 0; }

  int16_t centerTextX(const char* s, uint8_t textSize) {
    return (width() - (int16_t)strlen(s) * 6 * textSize) / 2;
  }

  // ---------- text (proportional font, y = baseline) ----------
  // size 0 = FreeSansBold12pt, 1 = FreeSansBold18pt, 2 = FreeSansBold9pt
  static const GFXfont* font_(uint8_t size) {
    return size == 1 ? &FreeSansBold18pt7b : (size == 2 ? &FreeSansBold9pt7b : &FreeSansBold12pt7b);
  }

  // Ink bounds of s drawn at x=0: left edge offset x1 and width w.
  void bigBounds(const char* s, uint8_t size, int16_t& x1, int16_t& w) {
    x1 = 0; w = 0;
    if (!_tft || !s || !*s) return;
    Adafruit_GFX& g = gfx();
    g.setFont(font_(size));
    g.setTextSize(1);
    int16_t y1; uint16_t ww, hh;
    g.getTextBounds(s, 0, 0, &x1, &y1, &ww, &hh);
    g.setFont(nullptr);
    w = (int16_t)ww;
  }

  int16_t bigWidth(const char* s, uint8_t size) {
    int16_t x1, w;
    bigBounds(s, size, x1, w);
    return w + x1;
  }

  void drawBig(int16_t x, int16_t baseline, const char* s, uint8_t size, uint16_t fg) {
    if (!_tft) return;
    Adafruit_GFX& g = gfx();
    g.setFont(font_(size));
    g.setTextSize(1);
    g.setTextColor(fg);
    g.setCursor(x, baseline);
    g.print(s);
    g.setFont(nullptr);
  }

  // ---------- widgets ----------
  void drawHeader(int16_t xoff, const char* title) {
    if (!_tft) return;
    fillRect(xoff, 0, width(), 18, Col::BG);
    drawFastHLine(xoff + 4, 17, width() - 8, Col::ACC_DIM);
    drawText(xoff + centerTextX(title, 1), 5, title, Col::ACCENT);
  }

  void drawCard(int16_t x, int16_t y, int16_t w, int16_t h) {
    fillRoundRect(x, y, w, h, 4, Col::PANEL);
    drawRoundRect(x, y, w, h, 4, Col::BORDER);
  }

  // List row with a left label and an optional right value.
  // editL / editR highlight the field being edited by the encoder.
  void drawRow(int16_t x, int16_t y, int16_t w, int16_t h, const char* left, const char* right,
               bool selected, bool editL = false, bool editR = false, uint16_t rightCol = Col::MUTED) {
    if (!_tft) return;
    const int16_t ty = y + (h - 8) / 2;
    const bool editing = selected && (editL || editR);

    if (editing) {
      fillRoundRect(x, y, w, h, 3, Col::EDIT_BG);
      drawRoundRect(x, y, w, h, 3, Col::EDIT);
    } else if (selected) {
      fillRoundRect(x, y, w, h, 3, Col::ACCENT);
    } else {
      fillRoundRect(x, y, w, h, 3, Col::PANEL);
      drawRoundRect(x, y, w, h, 3, Col::BORDER);
    }

    const uint16_t fg  = (selected && !editing) ? Col::BG : Col::TEXT;
    const uint16_t fgR = (selected && !editing) ? Col::BG : rightCol;

    if (left) {
      if (editing && editL) {
        fillRoundRect(x + 3, y + 2, textWidth(left) + 6, h - 4, 2, Col::EDIT);
        drawText(x + 6, ty, left, Col::BG);
      } else {
        drawText(x + 6, ty, left, fg);
      }
    }

    if (right && *right) {
      int16_t rw = textWidth(right);
      int16_t rx = x + w - 6 - rw;
      if (editing && editR) {
        fillRoundRect(rx - 3, y + 2, rw + 6, h - 4, 2, Col::EDIT);
        drawText(rx, ty, right, Col::BG);
      } else {
        drawText(rx, ty, right, fgR);
      }
    }
  }

  // Thin scrollbar for lists longer than the screen.
  void drawScrollbar(int16_t x, int16_t y, int16_t h, int16_t first, int16_t visible, int16_t total) {
    if (total <= visible) return;
    fillRect(x, y, 2, h, Col::BORDER);
    int16_t bh = (int16_t)((int32_t)h * visible / total);
    if (bh < 6) bh = 6;
    int16_t by = y + (int16_t)((int32_t)(h - bh) * first / (total - visible));
    fillRect(x, by, 2, bh, Col::ACCENT);
  }

  void drawRGB565_P(int16_t x, int16_t y, const uint16_t* data, int16_t w, int16_t h) {
    if (!_tft) return;
    uint16_t line[160];
    if (w > 160) w = 160;
    for (int16_t row = 0; row < h; row++) {
      for (int16_t col = 0; col < w; col++) line[col] = pgm_read_word(&data[row * w + col]);
      gfx().drawRGBBitmap(x, y + row, line, w, 1);
    }
  }

private:
  Adafruit_ST7735* _tft = nullptr;
  GFXcanvas16* _canvas = nullptr;
};
