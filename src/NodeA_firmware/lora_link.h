#pragma once
#include <Arduino.h>
#include <LoRa.h>
#include "crypto_ccm.h"

namespace Link {

static const uint8_t MAGIC = 0xC3;

#pragma pack(push, 1)
struct FrameHdr {
  uint8_t  magic;
  uint8_t  src;
  uint8_t  dst;
  uint8_t  type;
  uint8_t  ttl;
  uint32_t msgId;
  uint8_t  nonce12[12];
  uint8_t  tag8[8];
  uint8_t  len;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct RxPacket {
  FrameHdr h;
  int      rssi;
  float    snr;
  uint8_t  plain[64];
};
#pragma pack(pop)

static inline void aad_make(const FrameHdr &h, uint8_t out[9]) {
  out[0] = h.magic;
  out[1] = h.src;
  out[2] = h.dst;
  out[3] = h.type;
  out[4] = h.ttl;
  out[5] = (uint8_t)(h.msgId >> 24);
  out[6] = (uint8_t)(h.msgId >> 16);
  out[7] = (uint8_t)(h.msgId >> 8);
  out[8] = (uint8_t)(h.msgId);
}

static inline bool send_enc(uint8_t src, uint8_t dst, uint8_t type, uint8_t ttl, uint32_t msgId,
                            const uint8_t *plain, uint8_t plainLen, bool async = true) {
  FrameHdr h;
  h.magic = MAGIC;
  h.src = src;
  h.dst = dst;
  h.type = type;
  h.ttl = ttl;
  h.msgId = msgId;
  nonce12_make(h.nonce12, h.src, h.dst, h.type, h.msgId);

  uint8_t aad[9];
  aad_make(h, aad);

  uint8_t cipher[64];
  if (plainLen > sizeof(cipher)) return false;

  if (!aead_encrypt_ccm(plain, plainLen, aad, sizeof(aad), h.nonce12, cipher, h.tag8)) return false;
  h.len = plainLen;

  LoRa.beginPacket();
  LoRa.write((uint8_t*)&h, sizeof(h));
  LoRa.write(cipher, h.len);

  int ok = LoRa.endPacket(async);

  // Always go back to RX after TX
  LoRa.receive();

  return ok == 1;
}

static inline bool read_enc(RxPacket &out) {
  int p = LoRa.parsePacket();
  if (p <= 0) return false;

  out.rssi = LoRa.packetRssi();
  out.snr  = LoRa.packetSnr();

  if (p < (int)sizeof(FrameHdr)) {
    while (LoRa.available()) LoRa.read();
    return false;
  }

  LoRa.readBytes((uint8_t*)&out.h, sizeof(FrameHdr));
  if (out.h.magic != MAGIC) {
    while (LoRa.available()) LoRa.read();
    return false;
  }

  if (out.h.len > sizeof(out.plain)) {
    while (LoRa.available()) LoRa.read();
    return false;
  }

  uint8_t cipher[64];
  int got = LoRa.readBytes(cipher, out.h.len);
  if (got != out.h.len) {
    while (LoRa.available()) LoRa.read();
    return false;
  }

  uint8_t aad[9];
  aad_make(out.h, aad);

  if (!aead_decrypt_ccm(cipher, out.h.len, aad, sizeof(aad), out.h.nonce12, out.h.tag8, out.plain)) {
    return false;
  }

  return true;
}

static inline void log_rx(const char *dev, const RxPacket &p) {
  Serial.print(dev); Serial.print(" RX ");
  Serial.print("src="); Serial.print(p.h.src);
  Serial.print(" dst="); Serial.print(p.h.dst);
  Serial.print(" type="); Serial.print(p.h.type);
  Serial.print(" ttl="); Serial.print(p.h.ttl);
  Serial.print(" id="); Serial.print(p.h.msgId);
  Serial.print(" len="); Serial.print(p.h.len);
  Serial.print(" rssi="); Serial.print(p.rssi);
  Serial.print(" snr="); Serial.println(p.snr, 1);
}

static inline void log_tx(const char *dev, const char *name, bool ok, uint8_t src, uint8_t dst, uint8_t type, uint8_t ttl, uint32_t id) {
  Serial.print(dev); Serial.print(" TX "); Serial.print(name);
  Serial.print(" ok="); Serial.print(ok ? 1 : 0);
  Serial.print(" src="); Serial.print(src);
  Serial.print(" dst="); Serial.print(dst);
  Serial.print(" type="); Serial.print(type);
  Serial.print(" ttl="); Serial.print(ttl);
  Serial.print(" id="); Serial.println(id);
}

} // namespace Link