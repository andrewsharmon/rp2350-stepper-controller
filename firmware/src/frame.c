#include "frame.h"

#include <string.h>

uint16_t frame_crc16(const uint8_t *data, size_t len) {
    uint16_t crc = 0xffff;
    while (len--) {
        crc ^= (uint16_t)(*data++ << 8);
        for (int b = 0; b < 8; b++)
            crc = crc & 0x8000 ? (uint16_t)(crc << 1 ^ 0x1021) : (uint16_t)(crc << 1);
    }
    return crc;
}

size_t frame_cobs_encode(const uint8_t *in, size_t len, uint8_t *out) {
    size_t code_at = 0, o = 1;
    uint8_t code = 1;
    for (size_t i = 0; i < len; i++) {
        if (in[i] == 0) {
            out[code_at] = code;
            code_at = o++;
            code = 1;
            continue;
        }
        out[o++] = in[i];
        if (++code == 0xff) {
            out[code_at] = code;
            if (i + 1 == len)
                return o;  // canonical: no empty block after a full one at the end
            code_at = o++;
            code = 1;
        }
    }
    out[code_at] = code;
    return o;
}

int frame_cobs_decode(const uint8_t *in, size_t len, uint8_t *out, size_t out_max) {
    size_t i = 0, o = 0;
    while (i < len) {
        uint8_t code = in[i++];
        if (code == 0)
            return -1;
        for (uint8_t k = 1; k < code; k++) {
            if (i >= len || in[i] == 0 || o >= out_max)
                return -1;
            out[o++] = in[i++];
        }
        if (code != 0xff && i < len) {
            if (o >= out_max)
                return -1;
            out[o++] = 0;
        }
    }
    return (int)o;
}

size_t frame_build(uint8_t type, uint16_t seq, const uint8_t *payload, size_t len, uint8_t *out) {
    if (len > FRAME_MAX_PAYLOAD)
        return 0;
    uint8_t raw[FRAME_MAX_RAW];
    raw[0] = type;
    raw[1] = (uint8_t)seq;
    raw[2] = (uint8_t)(seq >> 8);
    memcpy(raw + 3, payload, len);
    uint16_t crc = frame_crc16(raw, 3 + len);
    raw[3 + len] = (uint8_t)crc;
    raw[4 + len] = (uint8_t)(crc >> 8);
    out[0] = 0;
    size_t n = frame_cobs_encode(raw, 5 + len, out + 1);
    out[1 + n] = 0;
    return n + 2;
}

int frame_parse(const uint8_t *in, size_t len, uint8_t *type, uint16_t *seq, uint8_t *payload) {
    uint8_t raw[FRAME_MAX_RAW];
    int n = frame_cobs_decode(in, len, raw, sizeof raw);
    if (n < 5)
        return -1;
    uint16_t crc = (uint16_t)(raw[n - 2] | raw[n - 1] << 8);
    if (frame_crc16(raw, (size_t)n - 2) != crc)
        return -1;
    *type = raw[0];
    *seq = (uint16_t)(raw[1] | raw[2] << 8);
    memcpy(payload, raw + 3, (size_t)n - 5);
    return n - 5;
}
