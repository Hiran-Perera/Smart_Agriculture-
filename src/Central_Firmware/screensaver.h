#pragma once
#include <Arduino.h>
#include <Adafruit_ST77xx.h>
#include "tft_driver.h"
#include "logo65.h"

struct ScreenSaver {
  bool drawn = false;

  void reset(uint32_t) { drawn = false; }

  bool tick(TftDriver& t, uint32_t) {
    if (drawn) return false;
    drawn = true;

  const int16_t W = t.width();

t.beginFrame();
t.fill(ST77XX_WHITE);

t.setTextSmall();

const char* line = "SMART_AGRO";
int16_t textY = 6;

t.drawText(
  t.centerTextX(line, 1),
  textY,
  line,
  ST77XX_BLACK,
  ST77XX_WHITE
);

const int16_t logoX = (W - (int16_t)LOGO_W) / 2;
const int16_t logoY = textY + 16;

t.drawRGB565_P(logoX, logoY, logo65, LOGO_W, LOGO_H);

t.endFrame();
return true;
  }
};