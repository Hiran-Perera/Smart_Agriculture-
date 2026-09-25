#pragma once
#include <Arduino.h>
#include "lora_link.h"
#include "payloads.h"

namespace Mesh {

// never forward packets destined to gateway or gateway acks.
// prevents duplicates at gateway.
static inline bool should_forward(uint8_t selfId, const Link::RxPacket &p) {
  if (p.h.dst == selfId) return false;
  if (p.h.ttl == 0) return false;

  if (p.h.dst == Payload::GW_ID) return false;
  if (p.h.type == Payload::RXACK) return false;
  if (p.h.type == Payload::ACK_CMD) return false;
  if (p.h.src == selfId) return false;

  return true;
}

static inline bool forward_packet(const char *dev, uint8_t selfId, const Link::RxPacket &p) {
  Link::RxPacket tmp = p;
  tmp.h.ttl = (uint8_t)(tmp.h.ttl - 1);

  bool ok = Link::send_enc(tmp.h.src, tmp.h.dst, tmp.h.type, tmp.h.ttl, tmp.h.msgId, tmp.plain, tmp.h.len, true);

  Serial.print(dev); Serial.print(" FWD ok="); Serial.print(ok ? 1 : 0);
  Serial.print(" via="); Serial.print(selfId);
  Serial.print(" src="); Serial.print(tmp.h.src);
  Serial.print(" dst="); Serial.print(tmp.h.dst);
  Serial.print(" type="); Serial.print(tmp.h.type);
  Serial.print(" id="); Serial.print(tmp.h.msgId);
  Serial.print(" ttl="); Serial.println(tmp.h.ttl);

  return ok;
}

} // namespace Mesh