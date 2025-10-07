#ifndef ARDUINO_BASE64_COMPAT_H
#define ARDUINO_BASE64_COMPAT_H

#include <stddef.h>
#include <stdint.h>

static inline bool gw_base64_is_valid_char(char c) {
  return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
         (c >= '0' && c <= '9') || c == '+' || c == '/';
}

static inline uint8_t gw_base64_decode_char(char c) {
  if (c >= 'A' && c <= 'Z') return (uint8_t)(c - 'A');
  if (c >= 'a' && c <= 'z') return (uint8_t)(c - 'a' + 26);
  if (c >= '0' && c <= '9') return (uint8_t)(c - '0' + 52);
  if (c == '+') return 62;
  if (c == '/') return 63;
  return 0;
}

static inline int base64_enc_len(int plainLen) {
  if (plainLen <= 0) {
    return 0;
  }
  return ((plainLen + 2) / 3) * 4 + 1; // Reserve space for '\0'
}

static inline int base64_dec_len(const char *input, int inputLen) {
  if (!input || inputLen <= 0) {
    return 0;
  }
  int padding = 0;
  if (inputLen >= 1 && input[inputLen - 1] == '=') padding++;
  if (inputLen >= 2 && input[inputLen - 2] == '=') padding++;
  return ((inputLen / 4) * 3) - padding;
}

static inline int base64_encode(char *output, const char *input, int inputLen) {
  if (!output || !input || inputLen <= 0) {
    if (output) {
      *output = '\0';
    }
    return 0;
  }

  int outIndex = 0;
  for (int i = 0; i < inputLen;) {
    int remaining = inputLen - i;
    uint32_t octet_a = (uint8_t)input[i++];
    uint32_t octet_b = remaining > 1 ? (uint8_t)input[i++] : 0;
    uint32_t octet_c = remaining > 2 ? (uint8_t)input[i++] : 0;
    uint32_t triple = (octet_a << 16) | (octet_b << 8) | octet_c;

    output[outIndex++] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/"[(triple >> 18) & 0x3F];
    output[outIndex++] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/"[(triple >> 12) & 0x3F];
    output[outIndex++] = (remaining > 1)
                             ? "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/"[(triple >> 6) & 0x3F]
                             : '=';
    output[outIndex++] = (remaining > 2)
                             ? "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/"[triple & 0x3F]
                             : '=';
  }

  output[outIndex] = '\0';
  return outIndex;
}

static inline int base64_decode(char *output, const char *input, int inputLen) {
  if (!output || !input || inputLen <= 0) {
    if (output) {
      *output = '\0';
    }
    return 0;
  }

  int outIndex = 0;
  uint8_t block[4];
  int blockLen = 0;

  for (int i = 0; i < inputLen; ++i) {
    char c = input[i];
    if (c == '=') {
      block[blockLen++] = 0;
    } else if (gw_base64_is_valid_char(c)) {
      block[blockLen++] = gw_base64_decode_char(c);
    } else {
      continue; // Skip whitespace or invalid characters
    }

    if (blockLen == 4) {
      output[outIndex++] = (char)((block[0] << 2) | (block[1] >> 4));
      if (input[i - 1] != '=') {
        output[outIndex++] = (char)((block[1] << 4) | (block[2] >> 2));
      }
      if (input[i] != '=') {
        output[outIndex++] = (char)((block[2] << 6) | block[3]);
      }
      blockLen = 0;
    }
  }

  output[outIndex] = '\0';
  return outIndex;
}

#endif // ARDUINO_BASE64_COMPAT_H
