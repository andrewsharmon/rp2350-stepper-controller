#pragma once

// Cam tables in flash: one 4 KB sector per table, below the show slots.
// Record: magic u32 "CAMT", version u16, n u16, cyclic u8, pad[3], crc32 u32
// (of the whole record with this field zero), then x f32 x n, y f32 x n.

#include <stdbool.h>
#include <stdint.h>
#include "cam.h"

#define CAM_STORE_SLOTS   4
#define CAM_RECORD_MAX    (16 + 8 * CAM_MAX_POINTS)

// Pure helpers (host-tested). pack returns the record length.
uint32_t cam_store_pack(uint8_t *out, const float *x, const float *y, uint32_t n, bool cyclic);
bool cam_store_unpack(const uint8_t *rec, uint32_t max_len, float *x, float *y, uint32_t *n,
                      bool *cyclic);

// Flash (target only). read returns false for an empty or invalid slot.
// write erases the slot and, if n > 0, stores the table; it only touches
// flash if the stored record differs. Axes must be at rest (it pauses core 1).
bool cam_store_read(uint32_t slot, float *x, float *y, uint32_t *n, bool *cyclic);
bool cam_store_write(uint32_t slot, const float *x, const float *y, uint32_t n, bool cyclic);
