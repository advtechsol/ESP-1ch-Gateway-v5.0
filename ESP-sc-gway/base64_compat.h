#pragma once

#include <stddef.h>
#include <stdint.h>

inline int base64_enc_len(int plainLen) {
    return ((plainLen + 2) / 3) * 4;
}

inline int base64_dec_len(const char *input, int inputLen) {
    if (!input || inputLen <= 0) {
        return 0;
    }

    int usefulChars = 0;
    int padding = 0;
    for (int i = 0; i < inputLen; ++i) {
        char c = input[i];
        if (c == '\0') {
            break;
        }
        if (c == '\r' || c == '\n' || c == ' ' || c == '\t') {
            continue;
        }
        ++usefulChars;
        if (c == '=') {
            ++padding;
        } else {
            padding = 0;
        }
    }

    if (usefulChars == 0) {
        return 0;
    }

    int groups = (usefulChars + 3) / 4;
    int length = groups * 3 - padding;
    return (length < 0) ? 0 : length;
}

inline int8_t base64_lookup(char c) {
    if (c >= 'A' && c <= 'Z') {
        return static_cast<int8_t>(c - 'A');
    }
    if (c >= 'a' && c <= 'z') {
        return static_cast<int8_t>(c - 'a' + 26);
    }
    if (c >= '0' && c <= '9') {
        return static_cast<int8_t>(c - '0' + 52);
    }
    if (c == '+') {
        return 62;
    }
    if (c == '/') {
        return 63;
    }
    if (c == '=') {
        return -2; // padding
    }
    if (c == '\r' || c == '\n' || c == ' ' || c == '\t') {
        return -3; // whitespace, ignore
    }
    return -1; // invalid
}

inline int base64_decode(char *output, const char *input, int inputLen) {
    if (!output || !input || inputLen <= 0) {
        if (output) {
            output[0] = '\0';
        }
        return 0;
    }

    int outIndex = 0;
    int val = 0;
    int valb = -8;
    for (int i = 0; i < inputLen; ++i) {
        char c = input[i];
        if (c == '\0') {
            break;
        }
        int8_t decoded = base64_lookup(c);
        if (decoded == -3) {
            continue;
        }
        if (decoded == -2) {
            break; // padding reached, decoding complete
        }
        if (decoded < 0) {
            break; // invalid char
        }
        val = (val << 6) + decoded;
        valb += 6;
        if (valb >= 0) {
            output[outIndex++] = static_cast<char>((val >> valb) & 0xFF);
            valb -= 8;
        }
    }

    output[outIndex] = '\0';
    return outIndex;
}

inline int base64_encode(char *output, const char *input, int inputLen) {
    static const char kAlphabet[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

    if (!output || !input || inputLen <= 0) {
        if (output) {
            output[0] = '\0';
        }
        return 0;
    }

    int outIndex = 0;
    int i = 0;
    while (i < inputLen) {
        uint32_t octet_a = static_cast<uint8_t>(input[i++]);
        bool have_b = i < inputLen;
        uint32_t octet_b = have_b ? static_cast<uint8_t>(input[i++]) : 0;
        bool have_c = i < inputLen;
        uint32_t octet_c = have_c ? static_cast<uint8_t>(input[i++]) : 0;

        uint32_t triple = (octet_a << 16) | (octet_b << 8) | octet_c;

        output[outIndex++] = kAlphabet[(triple >> 18) & 0x3F];
        output[outIndex++] = kAlphabet[(triple >> 12) & 0x3F];
        output[outIndex++] = have_b ? kAlphabet[(triple >> 6) & 0x3F] : '=';
        output[outIndex++] = have_c ? kAlphabet[triple & 0x3F] : '=';
    }

    output[outIndex] = '\0';
    return outIndex;
}
