#include "show_store.h"

#include <string.h>
#include "hardware/flash.h"
#include "pico/flash.h"

// Slots sit just below the two configuration sectors at the end of flash.
#define CONFIG_SECTORS  2
#define SLOT_OFFSET(k)  (PICO_FLASH_SIZE_BYTES - CONFIG_SECTORS * FLASH_SECTOR_SIZE - \
                         (SHOW_SLOTS - (k)) * SHOW_MAX_SIZE)

_Static_assert(SHOW_MAX_SIZE % FLASH_SECTOR_SIZE == 0, "show slots must be whole sectors");

const uint8_t *show_store_blob(uint32_t slot, uint32_t *len) {
    if (slot >= SHOW_SLOTS)
        return NULL;
    const uint8_t *p = (const uint8_t *)(XIP_BASE + SLOT_OFFSET(slot));
    uint32_t n = (uint32_t)p[8] | (uint32_t)p[9] << 8 | (uint32_t)p[10] << 16 | (uint32_t)p[11] << 24;
    if (n < SHOW_HEADER_SIZE || n > SHOW_MAX_SIZE)
        return NULL;  // empty (erased) or garbage; show_parse checks the rest
    *len = n;
    return p;
}

typedef struct {
    uint32_t offset, len;
    const uint8_t *data;
} job_t;

static void do_write(void *param) {
    const job_t *job = param;
    flash_range_erase(job->offset, SHOW_MAX_SIZE);
    if (job->len)
        flash_range_program(job->offset, job->data, job->len);
}

bool show_store_write(uint32_t slot, const uint8_t *data, uint32_t len) {
    if (slot >= SHOW_SLOTS || len > SHOW_MAX_SIZE)
        return false;
    // Program whole pages; the caller's buffer is padded to SHOW_MAX_SIZE.
    uint32_t padded = (len + FLASH_PAGE_SIZE - 1) / FLASH_PAGE_SIZE * FLASH_PAGE_SIZE;
    job_t job = {SLOT_OFFSET(slot), padded, data};
    if (flash_safe_execute(do_write, &job, 2000) != PICO_OK)
        return false;
    return len == 0 || memcmp((const void *)(XIP_BASE + SLOT_OFFSET(slot)), data, len) == 0;
}
