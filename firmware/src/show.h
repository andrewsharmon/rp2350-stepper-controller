#pragma once

// Shows: keyframe tracks for axes and LED pixels on one timeline, stored as
// a binary blob (compiled by tools/stepperctl from JSON). Layout, all
// little-endian:
//
//   header  magic u32 "SHOW", version u16, header_size u16, total_len u32,
//           crc32 u32 (of the whole blob with this field zero), name char[16],
//           duration_ms u32 (loop period; 0 = until the last key), flags u8
//           (bit0 loop), n_tracks u8, reserved u16
//   track   type u8 (1 axis, 2 LED), channel u8 (axis 0-9 / pixel), n_keys u16, keys:
//           axis key  t_ms u32, pos f32 (full steps), vel f32 (NaN = automatic)
//           LED key   t_ms u32, r u8, g u8, b u8, pad u8
//
// Keys are in increasing time. Axis tracks are played as PVT points (cubic
// Hermite), so motion passes exactly through every key; automatic speeds
// come from the neighbouring keys (wrapping at the loop seam), with zero at
// the ends of a non-looping show. LED colours fade linearly between keys.
//
// Pure logic (no SDK), so it runs in the host tests.

#include <stdbool.h>
#include <stdint.h>

#define SHOW_MAGIC        0x574f4853u  // "SHOW"
#define SHOW_VERSION      1
#define SHOW_HEADER_SIZE  40
#define SHOW_MAX_SIZE     16384
#define SHOW_MAX_TRACKS   32
#define SHOW_NAME_LEN     16

enum { SHOW_TRACK_AXIS = 1, SHOW_TRACK_LED = 2 };
#define SHOW_FLAG_LOOP 1

typedef struct {
    uint8_t type, channel;
    uint16_t n_keys;
    const uint8_t *keys;  // into the blob
} show_track_t;

typedef struct {
    const uint8_t *blob;
    uint32_t len;
    char name[SHOW_NAME_LEN + 1];
    uint32_t duration_ms;  // effective: the header value, or the last key time
    bool loop;
    uint32_t n_tracks;
    show_track_t track[SHOW_MAX_TRACKS];
} show_t;

uint32_t show_crc32(const uint8_t *blob, uint32_t len);   // with the crc field taken as zero

// Parse and validate a blob (bounds, CRC, key order, channels). Returns
// NULL on success or a description of the problem.
const char *show_parse(show_t *s, const uint8_t *blob, uint32_t len, uint32_t n_axes,
                       uint32_t n_pixels);

// Axis keyframe k of a track: time (ms, unwrapped by `cycle` loops) and
// position; speed resolved (automatic speeds computed from neighbours).
typedef struct {
    uint32_t t_ms;
    float pos, vel;
} show_point_t;

show_point_t show_axis_key(const show_t *s, const show_track_t *tr, uint32_t k, uint32_t cycle);

// Number of PVT points one pass of the track produces (looping shows add
// the seam segment back to the first key).
uint32_t show_axis_points(const show_t *s, const show_track_t *tr);

// Point i (i >= 0, any number of cycles for a looping show) of the stream.
show_point_t show_axis_point(const show_t *s, const show_track_t *tr, uint64_t i);

// LED colour of a track at show time t_ms (wrapped for a looping show).
void show_led_at(const show_t *s, const show_track_t *tr, uint32_t t_ms, uint8_t rgb[3]);

// Check every axis segment against limits (sampled peak speed and
// acceleration of the cubic). Returns NULL, or fills `msg` and returns it.
const char *show_check_limits(const show_t *s, const float *vmax, const float *amax,
                              char *msg, uint32_t msg_len);
