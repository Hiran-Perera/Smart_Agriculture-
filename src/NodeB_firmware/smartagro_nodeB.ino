// smartagro_nodeB.ino
// ESP32-C3 + SX1278 + non-blocking SOFT START with LOW-FREQ PWM (less RF noise)
// Inverted stage: ON=GPIO LOW, OFF=GPIO HIGH
// Keeps LoRa running during ramp.

#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>

#include "payloads.h"
#include "lora_link.h"
#include "mesh_forward.h"
#include "power_policy.h"
#include "sensors_b.h"

#include "driver/gpio.h"

static const char *DEV = "NB";
using namespace Payload;

static const uint8_t SELF_ID = NODE_B;
static const long LORA_FREQ = 433E6;

// ESP32-C3 LoRa pins
static const int LORA_SCK  = 4;
static const int LORA_MISO = 5;
static const int LORA_MOSI = 6;
static const int LORA_CS   = 7;
static const int LORA_RST  = 21;
static const int LORA_DIO0 = 20;

// I/O pins (your wiring)
static const int PIN_SOIL  = 1;   // ADC
static const int PIN_BATT  = 0;   // ADC
static const int PIN_FLOW  = 3;   // interrupt input
static const int PIN_RELAY = 2;   // motor gate/driver pin (inverted stage)

static const float BATT_DIVIDER_RATIO = 2.0f;

// Defaults
static const uint16_t DEF_PERIOD_S = 30;
static const uint16_t DEF_RXWIN_MS = 2500;

static volatile uint32_t g_flowPulses = 0;
static uint32_t g_msgId = 1;

// Motor steady state
static bool g_motorOn = false;
static bool g_targetOn = false;

// ---------- PWM soft start (low noise) ----------
static const int PWM_CH   = 0;
static const int PWM_RES  = 10;       // 0..1023
static const int PWM_FREQ = 2000;     // 2 kHz (lower noise than 20 kHz)
static const uint16_t DUTY_MAX = 900; // limit peak duty

// Slow ramp
static const uint16_t SOFTSTART_MS = 3000;  // 3 seconds
static const uint16_t SOFTSTEP_MS  = 30;    // update every 30ms

static bool     g_rampActive = false;
static uint32_t g_rampStartMs = 0;
static uint32_t g_rampLastStepMs = 0;

static void IRAM_ATTR isr_flow() { g_flowPulses++; }

// Inverted stage: ON->LOW, OFF->HIGH
static inline int motor_level_for(bool onLogical) { return onLogical ? 0 : 1; }

// Safe PWM attach holding OFF
static void pwm_begin_off() {
  pinMode(PIN_RELAY, OUTPUT);
  gpio_set_level((gpio_num_t)PIN_RELAY, motor_level_for(false)); // OFF = HIGH
  delay(2);

  ledcSetup(PWM_CH, PWM_FREQ, PWM_RES);
  ledcAttachPin(PIN_RELAY, PWM_CH);

  // OFF for inverted stage = HIGH all the time = DUTY_MAX
  ledcWrite(PWM_CH, DUTY_MAX);
  delay(2);
}

// Safe PWM detach (park then detach then force GPIO)
static void pwm_end_and_set_level(bool onLogical) {
  if (onLogical) {
    ledcWrite(PWM_CH, 0);         // ON inverted = LOW all the time
  } else {
    ledcWrite(PWM_CH, DUTY_MAX);  // OFF inverted = HIGH all the time
  }
  delay(2);

  ledcDetachPin(PIN_RELAY);

  pinMode(PIN_RELAY, OUTPUT);
  gpio_set_level((gpio_num_t)PIN_RELAY, motor_level_for(onLogical));
  delay(2);
}

// Non-blocking inverted PWM ramp (OFF->ON)
static void motor_tick(uint32_t now) {
  if (!g_rampActive) return;

  if (g_rampLastStepMs != 0 && (uint32_t)(now - g_rampLastStepMs) < SOFTSTEP_MS) return;
  g_rampLastStepMs = now;

  uint32_t dt = now - g_rampStartMs;

  if (dt >= SOFTSTART_MS) {
    g_rampActive = false;
    pwm_end_and_set_level(true);
    g_motorOn = true;

    Serial.print(DEV);
    Serial.println(" MOTOR ramp done state=1");
    return;
  }

  // duty goes 0..DUTY_MAX (PWM sweep)
  uint32_t duty = (uint32_t)DUTY_MAX * dt / SOFTSTART_MS;
  uint32_t dutyInv = DUTY_MAX - duty;
  ledcWrite(PWM_CH, (uint32_t)dutyInv);
}

static void apply_motor(bool on) {
  gpio_hold_dis((gpio_num_t)PIN_RELAY);
  g_targetOn = on;

  if (!on) {
    g_rampActive = false;
    pwm_end_and_set_level(false);
    g_motorOn = false;

    Serial.print(DEV);
    Serial.println(" MOTOR state=0");
    return;
  }

  // Start ramp
  g_rampActive = true;
  g_rampStartMs = millis();
  g_rampLastStepMs = 0;

  pwm_begin_off();

  Serial.print(DEV);
  Serial.println(" MOTOR ramp start");
}

static void apply_cfg(uint16_t period_s, uint16_t rxwin_ms) {
  if (period_s < 5) period_s = 5;
  if (period_s > 3600) period_s = 3600;
  if (rxwin_ms < 100) rxwin_ms = 100;
  if (rxwin_ms > 8000) rxwin_ms = 8000;

  PowerPolicy::set_period_s(period_s);
  PowerPolicy::set_rxwin_ms(rxwin_ms);

  Serial.print(DEV);
  Serial.print(" CFG applied period_s=");
  Serial.print(period_s);
  Serial.print(" rxwin_ms=");
  Serial.println(rxwin_ms);
}

static void handle_cfg_pkt(const Link::RxPacket &rx) {
  if (rx.h.type != CMD_CFG_B) return;
  if (rx.h.src != GW_ID) return;
  if (rx.h.len != sizeof(CmdCfgSleepB)) return;

  CmdCfgSleepB c;
  memcpy(&c, rx.plain, sizeof(c));
  apply_cfg(c.period_s, c.rxwin_ms);

  AckCfgSleepB a;
  a.applied_period_s = PowerPolicy::get_period_s(DEF_PERIOD_S);
  a.applied_rxwin_ms = PowerPolicy::get_rxwin_ms(DEF_RXWIN_MS);

  bool ok = Link::send_enc(SELF_ID, GW_ID, ACK_CMD, 2, rx.h.msgId, (uint8_t*)&a, sizeof(a), true);
  Link::log_tx(DEV, "ACK_CFG_B", ok, SELF_ID, GW_ID, ACK_CMD, 2, rx.h.msgId);
}

static void handle_motor_pkt(const Link::RxPacket &rx) {
  if (rx.h.type != CMD_RELAY) return;
  if (rx.h.src != GW_ID) return;
  if (rx.h.len != sizeof(CmdRelay)) return;

  CmdRelay c;
  memcpy(&c, rx.plain, sizeof(c));

  apply_motor(c.relay_on != 0);

  AckRelay a;
  a.applied_state = g_targetOn ? 1 : 0;

  bool ok = Link::send_enc(SELF_ID, GW_ID, ACK_CMD, 2, rx.h.msgId, (uint8_t*)&a, sizeof(a), true);
  Link::log_tx(DEV, "ACK_MOTOR", ok, SELF_ID, GW_ID, ACK_CMD, 2, rx.h.msgId);
}

static bool wait_rxack(uint32_t id, uint32_t timeoutMs) {
  uint32_t t0 = millis();
  while (millis() - t0 < timeoutMs) {
    motor_tick(millis());

    Link::RxPacket rx;
    if (Link::read_enc(rx)) {
      Link::log_rx(DEV, rx);

      if (Mesh::should_forward(SELF_ID, rx)) {
        Mesh::forward_packet(DEV, SELF_ID, rx);
        continue;
      }

      if (rx.h.dst != SELF_ID) continue;

      if (rx.h.type == CMD_RELAY) { handle_motor_pkt(rx); continue; }
      if (rx.h.type == CMD_CFG_B) { handle_cfg_pkt(rx); continue; }

      if (rx.h.type == RXACK && rx.h.msgId == id) {
        Serial.print(DEV); Serial.print(" RXACK ok id="); Serial.println(id);
        return true;
      }
    }
    delay(5);
  }

  Serial.print(DEV); Serial.print(" RXACK timeout id="); Serial.println(id);
  return false;
}

static void rx_window(uint16_t rxwinMs) {
  uint32_t t0 = millis();
  while (millis() - t0 < rxwinMs) {
    motor_tick(millis());

    Link::RxPacket rx;
    if (!Link::read_enc(rx)) { delay(5); continue; }

    Link::log_rx(DEV, rx);

    if (Mesh::should_forward(SELF_ID, rx)) {
      Mesh::forward_packet(DEV, SELF_ID, rx);
      continue;
    }

    if (rx.h.dst != SELF_ID) continue;

    if (rx.h.type == CMD_RELAY) { handle_motor_pkt(rx); continue; }
    if (rx.h.type == CMD_CFG_B) { handle_cfg_pkt(rx); continue; }
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("NB ready");

  analogReadResolution(12);
  PowerPolicy::begin("nodeB");

  pinMode(PIN_RELAY, OUTPUT);
  gpio_hold_dis((gpio_num_t)PIN_RELAY);
  gpio_set_level((gpio_num_t)PIN_RELAY, motor_level_for(false)); // OFF
  g_motorOn = false;
  g_targetOn = false;
  g_rampActive = false;

  pinMode(PIN_FLOW, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_FLOW), isr_flow, RISING);

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("NB LORA init fail");
    while (true) delay(1000);
  }

  LoRa.setTxPower(17);
  LoRa.enableCrc();
  LoRa.receive();
}

void loop() {
  motor_tick(millis());
  gpio_hold_dis((gpio_num_t)PIN_RELAY);

  uint16_t period_s = PowerPolicy::get_period_s(DEF_PERIOD_S);
  uint16_t rxwin_ms = PowerPolicy::get_rxwin_ms(DEF_RXWIN_MS);
  if (period_s < 5) period_s = 5;

  uint32_t p0 = g_flowPulses;
  delay(200);
  uint32_t p1 = g_flowPulses;
  uint32_t dp = p1 - p0;

  TelemetryB tb;
  tb.ver = 1;
  tb.soil_raw = SensorsB::read_adc_u16(PIN_SOIL);
  tb.flow_hz_x10 = (uint16_t)(dp * 5 * 10);
  tb.batt_mV = SensorsB::read_batt_mV(PIN_BATT, BATT_DIVIDER_RATIO);
  tb.relay_state = g_targetOn ? 1 : 0;

  Serial.print(DEV); Serial.print(" DATA soil="); Serial.print(tb.soil_raw);
  Serial.print(" flowx10="); Serial.print(tb.flow_hz_x10);
  Serial.print(" battmV="); Serial.print(tb.batt_mV);
  Serial.print(" motor="); Serial.println(tb.relay_state);

  uint32_t id = g_msgId++;
  bool sent = Link::send_enc(SELF_ID, GW_ID, TELEMETRY, 2, id, (uint8_t*)&tb, sizeof(tb), true);
  Link::log_tx(DEV, "TEL_B", sent, SELF_ID, GW_ID, TELEMETRY, 2, id);

  if (sent) wait_rxack(id, 1500);

  rx_window(rxwin_ms);

  // Do not sleep while ramping
  if (g_rampActive) {
    Serial.print(DEV);
    Serial.println(" SLEEP skipped: ramp active");
    delay(10);
    return;
  }

  // Hold final steady level during sleep
  gpio_set_level((gpio_num_t)PIN_RELAY, motor_level_for(g_motorOn));
  gpio_hold_en((gpio_num_t)PIN_RELAY);

  uint32_t sleepMs = (uint32_t)period_s * 1000UL;
  Serial.print(DEV); Serial.print(" SLEEP ms="); Serial.println(sleepMs);

  Serial.flush();
  delay(50);
  PowerPolicy::sleep_ms(sleepMs);
}