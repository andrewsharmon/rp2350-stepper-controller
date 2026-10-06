#pragma once

// Binary frame layer shared with the host tool (tools/stepperctl).
//
// On the wire: 0x00, COBS(type, seq_lo, seq_hi, payload..., crc_lo, crc_hi), 0x00.
// COBS removes every zero byte, so frames can share the USB serial port with
// the text console: a 0x00 starts a frame and the next 0x00 ends it. The CRC
// is CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over type, seq and payload.
//
// Pure logic (no SDK), so it runs in the host tests.

#include <stddef.h>
#include <stdint.h>

#define FRAME_MAX_PAYLOAD 240
#define FRAME_MAX_RAW     (3 + FRAME_MAX_PAYLOAD + 2)   // type, seq, payload, crc
#define FRAME_MAX_WIRE    (FRAME_MAX_RAW + FRAME_MAX_RAW / 254 + 1 + 2)

uint16_t frame_crc16(const uint8_t *data, size_t len);

// COBS. Encode returns the encoded length (no delimiters); decode returns
// the decoded length, or -1 if the input is malformed.
size_t frame_cobs_encode(const uint8_t *in, size_t len, uint8_t *out);
int frame_cobs_decode(const uint8_t *in, size_t len, uint8_t *out, size_t out_max);

// Build a complete wire frame (with both delimiters) into `out`
// (FRAME_MAX_WIRE bytes). Returns its length, or 0 if the payload is too big.
size_t frame_build(uint8_t type, uint16_t seq, const uint8_t *payload, size_t len, uint8_t *out);

// Decode the bytes between two delimiters. On success fills type/seq and
// copies the payload to `payload` (FRAME_MAX_PAYLOAD bytes) and returns the
// payload length; returns -1 on a malformed frame or bad CRC.
int frame_parse(const uint8_t *in, size_t len, uint8_t *type, uint16_t *seq, uint8_t *payload);
