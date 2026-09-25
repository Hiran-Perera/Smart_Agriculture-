// smartagro_nodeA.ino  (ESP32-C3 + SX1278 + DHT11 on GPIO2)

#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <DHT.h>

#include "payloads.h"
#include "lora_link.h"
#include "mesh_forward.h"
#include "power_policy.h"
#include "sensors_a.h"

static const char *DEV = "NA";
using namespace Payload;

static const uint8_t SELF_ID = NODE_A;
static const long LORA_FREQ = 433E6;

// ESP32-C3 LoRa pins
static const int LORA_SCK  = 4;
static const int LORA_MISO = 5;
static const int LORA_MOSI = 6;
static const int LORA_CS   = 7;
static const int LORA_RST  = 21;
static const int LORA_DIO0 = 20;

// Sensors pins
static const int PIN_LDR  = 3;   // ADC
static const int PIN_BATT = 0;   // ADC
static const float BATT_DIVIDER_RATIO = 2.0f;

// DHT11
static const int PIN_DHT = 2;
static const int DHT_TYPE = DHT11;
static DHT dht(PIN_DHT, DHT_TYPE);

// Defaults
static const uint16_t DEF_PERIOD_S = 30;
static const uint16_t DEF_RXWIN_MS = 1200;

static uint32_t g_msgId = 1;

static bool wait_rxack(uint32_t id, uint32_t timeoutMs) {
  uint32_t t0 = millis();
  while (millis() - t0 < timeoutMs) {
    Link::RxPacket rx;
    if (Link::read_enc(rx)) {
      Link::log_rx(DEV, rx);

      if (Mesh::should_forward(SELF_ID, rx)) {
        Mesh::forward_packet(DEV, SELF_ID, rx);
        continue;
      }

      if (rx.h.dst != SELF_ID) continue;

      if (rx.h.type == RXACK && rx.h.msgId == id) {
        Serial.print(DEV);
        Serial.print(" RXACK ok id=");
        Serial.println(id);
        return true;
      }
    }
    delay(5);
  }
  Serial.print(DEV);
  Serial.print(" RXACK timeout id=");
  Serial.println(id);
  return false;
}

static void handle_cfg_window(uint16_t rxwinMs) {
  uint32_t t0 = millis();
  while (millis() - t0 < rxwinMs) {
    Link::RxPacket rx;
    if (!Link::read_enc(rx)) { delay(5); continue; }

    Link::log_rx(DEV, rx);

    if (Mesh::should_forward(SELF_ID, rx)) {
      Mesh::forward_packet(DEV, SELF_ID, rx);
      continue;
    }

    if (rx.h.dst != SELF_ID) continue;

    if (rx.h.type == CMD_CFG_A && rx.h.src == GW_ID && rx.h.len == sizeof(CmdCfgSleepA)) {
      CmdCfgSleepA c;
      memcpy(&c, rx.plain, sizeof(c));

      if (c.period_s < 5) c.period_s = 5;
      if (c.period_s > 3600) c.period_s = 3600;
      if (c.rxwin_ms < 100) c.rxwin_ms = 100;
      if (c.rxwin_ms > 5000) c.rxwin_ms = 5000;

      PowerPolicy::set_period_s(c.period_s);
      PowerPolicy::set_rxwin_ms(c.rxwin_ms);

      AckCfgSleepA a;
      a.applied_period_s = c.period_s;
      a.applied_rxwin_ms = c.rxwin_ms;

      bool ok = Link::send_enc(SELF_ID, GW_ID, ACK_CMD, 2, rx.h.msgId, (uint8_t*)&a, sizeof(a), true);
      Link::log_tx(DEV, "ACK_CFG_A", ok, SELF_ID, GW_ID, ACK_CMD, 2, rx.h.msgId);

      Serial.print(DEV);
      Serial.print(" CFG applied period_s=");
      Serial.print(c.period_s);
      Serial.print(" rxwin_ms=");
      Serial.println(c.rxwin_ms);
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("NA ready");

  analogReadResolution(12);
  PowerPolicy::begin("nodeA");

  dht.begin();

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("NA LORA init fail");
    while (true) delay(1000);
  }

  LoRa.setTxPower(17);
  LoRa.enableCrc();
  LoRa.receive();
}

void loop() {
  uint16_t period_s = PowerPolicy::get_period_s(DEF_PERIOD_S);
  uint16_t rxwin_ms = PowerPolicy::get_rxwin_ms(DEF_RXWIN_MS);
  if (period_s < 5) period_s = 5;

  TelemetryA ta;
  ta.ver = 1;

  // DHT11 read
  float h = dht.readHumidity();
  float tC = dht.readTemperature();

  ta.tempC_x100 = (int16_t)0x8000;
  ta.hum_x100   = (int16_t)0x8000;

  if (!isnan(tC) && tC > -40.0f && tC < 85.0f) {
    ta.tempC_x100 = (int16_t)lroundf(tC * 100.0f);
  }
  if (!isnan(h) && h >= 0.0f && h <= 100.0f) {
    ta.hum_x100 = (int16_t)lroundf(h * 100.0f);
  }

  ta.light_raw  = SensorsA::read_adc_u16(PIN_LDR);
  ta.batt_mV    = SensorsA::read_batt_mV(PIN_BATT, BATT_DIVIDER_RATIO);

  Serial.print(DEV);
  Serial.print(" DATA tempC=");
  if (ta.tempC_x100 == (int16_t)0x8000) Serial.print("NA");
  else Serial.print((float)ta.tempC_x100 / 100.0f, 2);

  Serial.print(" hum=");
  if (ta.hum_x100 == (int16_t)0x8000) Serial.print("NA");
  else Serial.print((float)ta.hum_x100 / 100.0f, 2);

  Serial.print(" light=");
  Serial.print(ta.light_raw);
  Serial.print(" battmV=");
  Serial.println(ta.batt_mV);

  uint32_t id = g_msgId++;
  bool sent = Link::send_enc(SELF_ID, GW_ID, TELEMETRY, 2, id, (uint8_t*)&ta, sizeof(ta), true);
  Link::log_tx(DEV, "TEL_A", sent, SELF_ID, GW_ID, TELEMETRY, 2, id);

  if (sent) {
    wait_rxack(id, 1500);
  }

  handle_cfg_window(rxwin_ms);

  uint32_t sleepMs = (uint32_t)period_s * 1000UL;
  Serial.print(DEV); Serial.print(" SLEEP ms="); Serial.println(sleepMs);
  Serial.flush();
  delay(300);
  PowerPolicy::sleep_ms(sleepMs);
}