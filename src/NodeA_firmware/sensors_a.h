#pragma once
#include <Arduino.h>

namespace SensorsA {

static const float ADC_REF = 3.3f;
static const int   ADC_MAX = 4095;

static inline uint16_t read_adc_u16(int pin) {
  return (uint16_t)analogRead(pin);
}

static inline uint16_t read_batt_mV(int pinBatt, float dividerRatio) {
  uint16_t raw = (uint16_t)analogRead(pinBatt);
  float v = ((float)raw * ADC_REF / (float)ADC_MAX) * dividerRatio;
  if (v < 0) v = 0;
  return (uint16_t)(v * 1000.0f);
}

} // namespace SensorsA