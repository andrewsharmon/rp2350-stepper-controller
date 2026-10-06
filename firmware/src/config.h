#pragma once

// Persistent configuration in the last two 4 KB flash sectors, written
// alternately. Each record carries a generation counter and a CRC-32, so a
// power cut mid-save leaves the previous record intact; loading picks the
// newest valid one.

#include <stdbool.h>
#include <stdint.h>

#define CONFIG_AXES 10

// Per-axis drive: automatic amplitude curve (fractions of full PWM span) and
// wiring fixes.
#define CONFIG_FLAG_REVERSE   1   // reverse the direction of rotation
#define CONFIG_FLAG_SWAP_COILS 2  // coil A on the B pins and vice versa

typedef struct {
    float amp_low, amp_high, amp_hold;    // run at low speed, run at high speed, hold
    float low_speed, high_speed;          // full steps/s: flat below, capped above
    uint8_t flags;
} config_drive_t;

typedef struct {
    float vmax, amax;                     // full steps/s, full steps/s^2
    uint8_t profile;                      // motion_profile_t
    uint16_t jerk_ms;
    config_drive_t drive;
} config_axis_t;

typedef struct {
    uint32_t magic;
    uint16_t version, size;
    uint32_t generation;
    config_axis_t axis[CONFIG_AXES];
    uint16_t hold_delay_ms;
    uint8_t boot_show;                    // 0xff: none
    uint8_t reserved;
    uint32_t crc;                         // CRC-32 of everything before it
} config_t;

void config_defaults(config_t *c);

// Pure helpers (host-tested).
uint32_t config_crc32(const void *data, uint32_t len);
bool config_valid(const config_t *c);
void config_seal(config_t *c);   // set magic/version/size and the CRC
// Of two stored records (either may be invalid), the index of the newest
// valid one, or -1.
int config_pick(const config_t *a, const config_t *b);

// Flash (target only). config_load falls back to defaults. config_save
// must only run with every axis at rest: it stalls both cores for tens of
// ms. Returns false if the write didn't verify.
bool config_load(config_t *c);
bool config_save(config_t *c);
