#pragma once
#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <Adafruit_ST77xx.h>

#ifndef ST77XX_DARKGREY
#define ST77XX_DARKGREY 0x7BEF
#endif
#ifndef ST77XX_LIGHTGREY
#define ST77XX_LIGHTGREY 0xC618
#endif

struct TftConfig {
  int cs = -1;
  int dc = -1;
  int rst = -1;
  uint8_t rotation = 1;
};

class TftDriver {
public:
  TftDriver() {}

  void begin(const TftConfig& cfg) {
    _cs = cfg.cs;
    _dc = cfg.dc;
    _rst = cfg.rst;

    if (_tft) { delete _tft; _tft = nullptr; }

    _tft = new Adafruit_ST7735(_cs, _dc, _rst);
    _tft->initR(INITR_BLACKTAB);
    _tft->setRotation(cfg.rotation);
    _tft->fillScreen(ST77XX_BLACK);
    _tft->setTextWrap(false);
  }

  int16_t width()  const { return _tft ? _tft->width()  : 160; }
  int16_t height() const { return _tft ? _tft->height() : 128; }

  void beginFrame() {}
  void beginFrame(bool) { beginFrame(); }
  void endFrame() {}

  void drawTextNoBg(int16_t x, int16_t y, const char* s, uint16_t fg) {
    if (!_tft) return;
    _tft->setCursor(x, y);
    _tft->setTextColor(fg);
    _tft->print(s);
  }

  void fillScreen(uint16_t c) { if (_tft) _tft->fillScreen(c); }
  void fill(uint16_t c) { fillScreen(c); }

  void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t c) {
    if (_tft) _tft->fillRect(x, y, w, h, c);
  }

  void setTextSmall()  { if (_tft) _tft->setTextSize(1); }
  void setTextMedium() { if (_tft) _tft->setTextSize(2); }
  void setTextLarge()  { if (_tft) _tft->setTextSize(3); }

  void drawText(int16_t x, int16_t y, const char* s, uint16_t fg) {
    drawText(x, y, s, fg, ST77XX_BLACK);
  }

  void drawText(int16_t x, int16_t y, const char* s, uint16_t fg, uint16_t bg) {
    if (!_tft) return;
    _tft->setCursor(x, y);
    _tft->setTextColor(fg, bg);
    _tft->print(s);
  }

  void drawHeader(int16_t xoff, const char* title) {
    if (!_tft) return;
    _tft->fillRect(xoff, 0, width(), 18, ST77XX_BLACK);
    _tft->drawFastHLine(xoff, 17, width(), ST77XX_GREEN);

    setTextSmall();
    int16_t tx = xoff + centerTextX(title, 1);
    drawText(tx, 3, title, ST77XX_GREEN, ST77XX_BLACK);
  }

  void drawCard(int16_t x, int16_t y, int16_t w, int16_t h) {
    if (!_tft) return;
    _tft->drawRect(x, y, w, h, ST77XX_GREEN);
  }

  void drawListItem(int16_t x, int16_t y, int16_t w, int16_t h, const char* label, bool selected) {
    if (!_tft) return;

    uint16_t border = selected ? ST77XX_GREEN : ST77XX_LIGHTGREY;
    uint16_t fg = selected ? ST77XX_BLACK : ST77XX_WHITE;
    uint16_t bg = selected ? ST77XX_GREEN : ST77XX_BLACK;

    _tft->fillRect(x, y, w, h, bg);
    _tft->drawRect(x, y, w, h, border);

    setTextSmall();
    int16_t tw = (int16_t)textWidth(label, 1);
    int16_t tx = x + (w - tw) / 2;
    int16_t ty = y + (h - 8) / 2;
    drawText(tx, ty, label, fg, bg);
  }

  int16_t centerTextX(const char* s, uint8_t textSize) {
    if (!_tft) return 0;
    int16_t x1, y1;
    uint16_t w, h;
    _tft->setTextSize(textSize);
    _tft->getTextBounds((char*)s, 0, 0, &x1, &y1, &w, &h);
    return (width() - (int16_t)w) / 2;
  }
  void drawRGB565_P(int16_t x, int16_t y, const uint16_t* data, int16_t w, int16_t h) {
  if (!_tft) return;

  // w is small (logo), so a small line buffer is fine
  uint16_t line[160];
  if (w > 160) w = 160;

  for (int16_t row = 0; row < h; row++) {
    for (int16_t col = 0; col < w; col++) {
      line[col] = pgm_read_word(&data[row * w + col]);
    }
    _tft->drawRGBBitmap(x, y + row, line, w, 1);
  }
}
private:
  size_t textWidth(const char* s, uint8_t textSize) {
    if (!s) return 0;
    if (!_tft) return strlen(s) * 6 * textSize;
    int16_t x1, y1;
    uint16_t w, h;
    _tft->setTextSize(textSize);
    _tft->getTextBounds((char*)s, 0, 0, &x1, &y1, &w, &h);
    return w;
  }

private:
  int _cs = -1;
  int _dc = -1;
  int _rst = -1;
  Adafruit_ST7735* _tft = nullptr;
};