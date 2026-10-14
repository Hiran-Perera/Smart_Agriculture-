#pragma once
#include <Arduino.h>

namespace Payload {

static const uint8_t GW_ID   = 0x00;
static const uint8_t NODE_A  = 0x01;
static const uint8_t NODE_B  = 0x02;

enum MsgType : uint8_t {
  TELEMETRY = 0x01,

  CMD_RELAY = 0x02,   // GW -> NodeB
  ACK_CMD   = 0x03,   // Node -> GW (relay ack, cfg ack)

  RXACK     = 0x04,   // GW -> Node (telemetry received)

  CMD_CFG_A = 0x10,   // GW -> NodeA sleep config
  CMD_CFG_B = 0x11,   // GW -> NodeB sleep config
  CMD_SENSOR_CFG = 0x12, // GW -> Node sensor/pin assignment (ack = AckSensorCfg)

  TELEMETRY_X = 0x05  // Node -> GW extra (user assigned) sensors, no RXACK
};

#pragma pack(push, 1)
struct TelemetryA {
  uint8_t  ver;         // 1
  int16_t  tempC_x100;  // 0x8000 if NA
  int16_t  hum_x100;    // 0x8000 if NA
  uint16_t light_raw;
  uint16_t batt_mV;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct TelemetryB {
  uint8_t  ver;         // 1
  uint16_t soil_raw;
  uint16_t flow_hz_x10;
  uint16_t batt_mV;
  uint8_t  relay_state; // 0/1
};
#pragma pack(pop)

#pragma pack(push, 1)
struct CmdCfgSleepA {
  uint16_t period_s;
  uint16_t rxwin_ms;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct AckCfgSleepA {
  uint16_t applied_period_s;
  uint16_t applied_rxwin_ms;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct CmdCfgSleepB {
  uint16_t period_s;
  uint16_t rxwin_ms;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct AckCfgSleepB {
  uint16_t applied_period_s;
  uint16_t applied_rxwin_ms;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct CmdRelay {
  uint8_t relay_on; // 0/1
};
#pragma pack(pop)

#pragma pack(push, 1)
struct AckRelay {
  uint8_t applied_state; // 0/1
};
#pragma pack(pop)

// ---- Sensor assignment ----
static const uint8_t EXT_SLOTS = 3;     // user assignable sensor slots per node
static const uint8_t PIN_NONE  = 0xFF;

enum SensorType : uint8_t {
  SENS_NONE    = 0,
  SENS_ANALOG  = 1,  // raw 12-bit ADC (ADC capable pins only)
  SENS_DIGITAL = 2,  // 0/1 (internal pull-up enabled)
  SENS_PULSE   = 3   // pulse counter, reported as Hz x10
};

#pragma pack(push, 1)
struct SensorSlotCfg {
  uint8_t type;   // SensorType
  uint8_t pin;    // GPIO, PIN_NONE if type == SENS_NONE
};
#pragma pack(pop)

#pragma pack(push, 1)
struct CmdSensorCfg {
  uint8_t ver;                       // 1
  uint8_t flow_pin;                  // NodeB flow sensor GPIO, PIN_NONE on NodeA
  SensorSlotCfg slot[EXT_SLOTS];
};
#pragma pack(pop)

#pragma pack(push, 1)
struct AckSensorCfg {
  uint8_t ver;   // 1
  uint8_t ok;    // 1 applied, 0 rejected
};
#pragma pack(pop)

#pragma pack(push, 1)
struct ExtSlotVal {
  uint8_t  type;
  uint8_t  pin;
  uint16_t value;   // ANALOG raw, DIGITAL 0/1, PULSE Hz x10
};
#pragma pack(pop)

#pragma pack(push, 1)
struct TelemetryX {
  uint8_t ver;      // 1
  ExtSlotVal s[EXT_SLOTS];
};
#pragma pack(pop)

} // namespace Payload