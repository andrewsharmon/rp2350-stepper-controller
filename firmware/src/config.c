#include "config.h"

#include <stddef.h>
#include <string.h>

#define CONFIG_MAGIC   0x43505453u   // "STPC"
#define CONFIG_VERSION 1

void config_defaults(config_t *c) {
    memset(c, 0, sizeof *c);
    for (int i = 0; i < CONFIG_AXES; i++) {
        config_axis_t *a = &c->axis[i];
        // Bench 8 mm stepper at 5 V (see README / milestone 2 notes).
        a->vmax = 1500.0f;
        a->amax = 2000.0f;
        a->profile = 1;  // scurve
        a->jerk_ms = 30;
        a->drive = (config_drive_t){
            .amp_low = 0.40f, .amp_high = 0.60f, .amp_hold = 0.25f,
            .low_speed = 300.0f, .high_speed = 1600.0f, .flags = 0,
        };
    }
    c->hold_delay_ms = 500;
    c->boot_show = 0xff;
    config_seal(c);
}

uint32_t config_crc32(const void *data, uint32_t len) {
    const uint8_t *p = data;
    uint32_t crc = 0xffffffffu;
    while (len--) {
        crc ^= *p++;
        for (int b = 0; b < 8; b++)
            crc = crc & 1 ? crc >> 1 ^ 0xedb88320u : crc >> 1;
    }
    return ~crc;
}

void config_seal(config_t *c) {
    c->magic = CONFIG_MAGIC;
    c->version = CONFIG_VERSION;
    c->size = sizeof *c;
    c->crc = config_crc32(c, offsetof(config_t, crc));
}

bool config_valid(const config_t *c) {
    return c->magic == CONFIG_MAGIC && c->version == CONFIG_VERSION && c->size == sizeof *c &&
           c->crc == config_crc32(c, offsetof(config_t, crc));
}

int config_pick(const config_t *a, const config_t *b) {
    bool va = config_valid(a), vb = config_valid(b);
    if (va && vb)
        return (int32_t)(b->generation - a->generation) > 0 ? 1 : 0;
    return va ? 0 : vb ? 1 : -1;
}

#ifndef CONFIG_HOST_TEST
#include "hardware/flash.h"
#include "pico/flash.h"

// Last two sectors of flash.
#define SECTOR_OFFSET(k) (PICO_FLASH_SIZE_BYTES - (2u - (k)) * FLASH_SECTOR_SIZE)

static const config_t *stored(int k) {
    return (const config_t *)(XIP_BASE + SECTOR_OFFSET(k));
}

bool config_load(config_t *c) {
    int k = config_pick(stored(0), stored(1));
    if (k < 0) {
        config_defaults(c);
        return false;
    }
    memcpy(c, stored(k), sizeof *c);
    return true;
}

typedef struct {
    uint32_t offset;
    const uint8_t *data;
} write_job_t;

static void do_write(void *param) {
    const write_job_t *job = param;
    flash_range_erase(job->offset, FLASH_SECTOR_SIZE);
    flash_range_program(job->offset, job->data, FLASH_SECTOR_SIZE / 4);  // 1 KB covers the record
}

_Static_assert(sizeof(config_t) <= FLASH_SECTOR_SIZE / 4, "config record outgrew the write size");

bool config_save(config_t *c) {
    // Overwrite the older (or invalid) sector; the newest stays intact.
    int newest = config_pick(stored(0), stored(1));
    int target = newest == 0 ? 1 : 0;
    c->generation = newest < 0 ? 1 : stored(newest)->generation + 1;
    config_seal(c);

    static uint8_t page[FLASH_SECTOR_SIZE / 4];
    memset(page, 0xff, sizeof page);
    memcpy(page, c, sizeof *c);
    write_job_t job = {SECTOR_OFFSET(target), page};
    if (flash_safe_execute(do_write, &job, 1000) != PICO_OK)
        return false;
    return memcmp(stored(target), c, sizeof *c) == 0;
}
#endif
