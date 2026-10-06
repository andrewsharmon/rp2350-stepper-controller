#include "cam_store.h"

#include <string.h>

#define MAGIC   0x544d4143u   // "CAMT"
#define VERSION 1

static uint32_t crc32(const uint8_t *p, uint32_t len) {
    uint32_t crc = 0xffffffffu;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= i >= 12 && i < 16 ? 0 : p[i];  // the crc field counts as zero
        for (int b = 0; b < 8; b++)
            crc = crc & 1 ? crc >> 1 ^ 0xedb88320u : crc >> 1;
    }
    return ~crc;
}

static void put32(uint8_t *p, uint32_t v) {
    for (int i = 0; i < 4; i++)
        p[i] = (uint8_t)(v >> (8 * i));
}

static uint32_t get32(const uint8_t *p) {
    return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}

uint32_t cam_store_pack(uint8_t *out, const float *x, const float *y, uint32_t n, bool cyclic) {
    memset(out, 0, 16);
    put32(out, MAGIC);
    out[4] = VERSION;
    out[6] = (uint8_t)n;
    out[7] = (uint8_t)(n >> 8);
    out[8] = cyclic;
    memcpy(out + 16, x, n * 4);
    memcpy(out + 16 + n * 4, y, n * 4);
    uint32_t len = 16 + n * 8;
    put32(out + 12, crc32(out, len));
    return len;
}

bool cam_store_unpack(const uint8_t *rec, uint32_t max_len, float *x, float *y, uint32_t *n,
                      bool *cyclic) {
    if (max_len < 16 || get32(rec) != MAGIC || (rec[4] | rec[5] << 8) != VERSION)
        return false;
    uint32_t cnt = (uint32_t)(rec[6] | rec[7] << 8);
    if (cnt < 2 || cnt > CAM_MAX_POINTS || 16 + cnt * 8 > max_len)
        return false;
    if (get32(rec + 12) != crc32(rec, 16 + cnt * 8))
        return false;
    memcpy(x, rec + 16, cnt * 4);
    memcpy(y, rec + 16 + cnt * 4, cnt * 4);
    *n = cnt;
    *cyclic = rec[8] != 0;
    return true;
}

#ifndef CONFIG_HOST_TEST
#include "hardware/flash.h"
#include "pico/flash.h"
#include "show_store.h"

// Below the show slots, which sit below the two config sectors.
#define CAM_OFFSET(k) (PICO_FLASH_SIZE_BYTES - 2 * FLASH_SECTOR_SIZE - SHOW_SLOTS * SHOW_MAX_SIZE - \
                       (CAM_STORE_SLOTS - (k)) * FLASH_SECTOR_SIZE)

_Static_assert(CAM_RECORD_MAX <= FLASH_SECTOR_SIZE, "a cam record must fit one sector");

static const uint8_t *stored(uint32_t k) {
    return (const uint8_t *)(XIP_BASE + CAM_OFFSET(k));
}

bool cam_store_read(uint32_t slot, float *x, float *y, uint32_t *n, bool *cyclic) {
    return slot < CAM_STORE_SLOTS && cam_store_unpack(stored(slot), FLASH_SECTOR_SIZE, x, y, n, cyclic);
}

typedef struct {
    uint32_t offset, len;
    const uint8_t *data;
} job_t;

static void do_write(void *param) {
    const job_t *job = param;
    flash_range_erase(job->offset, FLASH_SECTOR_SIZE);
    if (job->len)
        flash_range_program(job->offset, job->data, job->len);
}

bool cam_store_write(uint32_t slot, const float *x, const float *y, uint32_t n, bool cyclic) {
    static uint8_t page[(CAM_RECORD_MAX + FLASH_PAGE_SIZE - 1) / FLASH_PAGE_SIZE * FLASH_PAGE_SIZE];
    if (slot >= CAM_STORE_SLOTS)
        return false;
    uint32_t len = 0;
    if (n) {
        memset(page, 0xff, sizeof page);
        len = cam_store_pack(page, x, y, n, cyclic);
        if (memcmp(stored(slot), page, len) == 0)
            return true;  // unchanged: spare the flash
    } else {
        static float tx[CAM_MAX_POINTS], ty[CAM_MAX_POINTS];  // off the stack
        uint32_t tn;
        bool tc;
        if (!cam_store_unpack(stored(slot), FLASH_SECTOR_SIZE, tx, ty, &tn, &tc))
            return true;  // already empty
    }
    uint32_t padded = (len + FLASH_PAGE_SIZE - 1) / FLASH_PAGE_SIZE * FLASH_PAGE_SIZE;
    job_t job = {CAM_OFFSET(slot), padded, page};
    if (flash_safe_execute(do_write, &job, 1000) != PICO_OK)
        return false;
    return len == 0 || memcmp(stored(slot), page, len) == 0;
}
#endif
