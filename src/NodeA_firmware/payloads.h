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
  CMD_EXT_OUT    = 0x13, // GW -> Node pump/valve outputs on EXT slots (ack = AckExtOut)

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
static const uint8_t EXT_SLOTS = 4;     // user assignable sensor slots per node (one per free GPIO)
static const uint8_t PIN_NONE  = 0xFF;
static const int16_t VAL_NA    = (int16_t)0x8000;

enum SensorType : uint8_t {
  SENS_NONE  = 0,
  SENS_DHT11 = 1,  // v1 = temp C x10, v2 = humidity % x10
  SENS_LDR   = 2,  // v1 = raw ADC (ADC pin)
  SENS_SOIL  = 3,  // v1 = raw ADC (ADC pin), higher = drier
  SENS_FLOW  = 4,  // v1 = pulse frequency Hz x10
  SENS_TDS   = 5,  // v1 = ppm (ADC pin)
  SENS_PH    = 6,  // v1 = pH x100 (ADC pin)
  SENS_PUMP  = 7,  // digital output, v1 = 0/1
  SENS_VALVE = 8,  // digital output, v1 = 0/1
  SENS_TYPE_COUNT
};

#pragma pack(push, 1)
struct SensorSlotCfg {
  uint8_t type;   // SensorType
  uint8_t pin;    // GPIO, PIN_NONE if type == SENS_NONE
};
#pragma pack(pop)

#pragma pack(push, 1)
struct CmdSensorCfg {
  uint8_t ver;                       // 2
  SensorSlotCfg slot[EXT_SLOTS];
};
#pragma pack(pop)

#pragma pack(push, 1)
struct AckSensorCfg {                // 2 bytes (ACK_CMD is told apart by length)
  uint8_t ver;
  uint8_t ok;    // 1 applied, 0 rejected
};
#pragma pack(pop)

// Pump / valve outputs on EXT slots. Bit i = slot i on.
#pragma pack(push, 1)
struct CmdExtOut {
  uint8_t ver;     // 1
  uint8_t states;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct AckExtOut {                   // 3 bytes
  uint8_t ver;
  uint8_t states;  // states actually applied
  uint8_t outMask; // which slots are outputs
};
#pragma pack(pop)

#pragma pack(push, 1)
struct ExtSlotVal {
  uint8_t type;
  uint8_t pin;
  int16_t v1;      // see SensorType, VAL_NA if not available
  int16_t v2;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct TelemetryX {
  uint8_t ver;     // 2
  ExtSlotVal s[EXT_SLOTS];
};
#pragma pack(pop)

} // namespace Payload
