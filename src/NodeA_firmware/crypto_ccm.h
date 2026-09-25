#pragma once
#include <Arduino.h>
#include "mbedtls/ccm.h"

// Use the SAME key on GW + both nodes.
static const uint8_t AES_KEY_128[16] = {
  0x10,0x22,0x33,0x44,0x55,0x66,0x77,0x88,
  0x99,0xAA,0xBB,0xCC,0xDD,0xEE,0xF0,0x0F
};

static inline void nonce12_make(uint8_t out12[12], uint8_t src, uint8_t dst, uint8_t type, uint32_t msgId) {
  uint32_t r0 = (uint32_t)esp_random();
  uint32_t r1 = (uint32_t)esp_random();
  out12[0]  = src;
  out12[1]  = dst;
  out12[2]  = type;
  out12[3]  = 0xA7;
  out12[4]  = (uint8_t)(msgId >> 24);
  out12[5]  = (uint8_t)(msgId >> 16);
  out12[6]  = (uint8_t)(msgId >> 8);
  out12[7]  = (uint8_t)(msgId);
  out12[8]  = (uint8_t)(r0);
  out12[9]  = (uint8_t)(r0 >> 8);
  out12[10] = (uint8_t)(r1);
  out12[11] = (uint8_t)(r1 >> 8);
}

static inline bool aead_encrypt_ccm(
  const uint8_t *plain, size_t plainLen,
  const uint8_t *aad, size_t aadLen,
  const uint8_t nonce12[12],
  uint8_t *cipherOut,
  uint8_t tagOut[8]
) {
  mbedtls_ccm_context ctx;
  mbedtls_ccm_init(&ctx);
  if (mbedtls_ccm_setkey(&ctx, MBEDTLS_CIPHER_ID_AES, AES_KEY_128, 128) != 0) {
    mbedtls_ccm_free(&ctx);
    return false;
  }
  int rc = mbedtls_ccm_encrypt_and_tag(&ctx, plainLen, nonce12, 12, aad, aadLen, plain, cipherOut, tagOut, 8);
  mbedtls_ccm_free(&ctx);
  return rc == 0;
}

static inline bool aead_decrypt_ccm(
  const uint8_t *cipher, size_t cipherLen,
  const uint8_t *aad, size_t aadLen,
  const uint8_t nonce12[12],
  const uint8_t tag8[8],
  uint8_t *plainOut
) {
  mbedtls_ccm_context ctx;
  mbedtls_ccm_init(&ctx);
  if (mbedtls_ccm_setkey(&ctx, MBEDTLS_CIPHER_ID_AES, AES_KEY_128, 128) != 0) {
    mbedtls_ccm_free(&ctx);
    return false;
  }
  int rc = mbedtls_ccm_auth_decrypt(&ctx, cipherLen, nonce12, 12, aad, aadLen, cipher, plainOut, tag8, 8);
  mbedtls_ccm_free(&ctx);
  return rc == 0;
}