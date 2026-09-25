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
  CMD_CFG_B = 0x11    // GW -> NodeB sleep config
};

#pragma pack(push, 1)
struct TelemetryA {
  uint8_t  ver;         // 1
  int16_t  tempC_x100;  // 0x8000 if unused
  int16_t  hum_x100;    // 0x8000 if unused
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

} // namespace Payload