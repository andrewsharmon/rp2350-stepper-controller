#include "tmf8829.h"

#include <string.h>
#include "pico/stdlib.h"

#include "tmf8829_image.h"

// Registers shared by the bootloader and the application.
#define REG_APP_ID     0x00
#define REG_CMD_STAT   0x08
#define REG_SERIAL     0x1c   // u32, application
#define REG_CID_RID    0x20   // loaded page / preset
#define REG_CFG_FIRST  TMF8829_CFG_FIRST
// Host interface registers (always there while EN is high).
#define REG_INT_STATUS 0xe1
#define REG_INT_ENAB   0xe2
#define REG_ID         0xe3   // chip id, then revision
#define REG_RESET      0xf7
#define REG_ENABLE     0xf8
#define REG_FIFOSTATUS 0xfa   // then SYSTICK_0..3, then the FIFO
#define REG_FIFO       0xff

#define ENABLE_CPU_READY      0x80
#define ENABLE_POWERUP_SELECT 0x30
#define ENABLE_SELECT_BOOTMON 0x10   // ignore the fuses and wait in the bootloader
#define ENABLE_SELECT_RAM     0x20   // restart the application in RAM
#define ENABLE_POFF           0x08
#define ENABLE_PON            0x04
#define RESET_SOFT            0x40

#define APP_ID_APPLICATION 0x01
#define APP_ID_BOOTLOADER  0x80

// Bootloader commands; status 0 = ready.
#define BL_CMD_START_RAM_APP 0x16
#define BL_CMD_SPI_OFF       0x20
#define BL_CMD_FIFO_BOTH     0x45   // address u32, words u16: the FIFO feeds both CPU RAMs

// Application commands and status.
#define APP_CMD_MEASURE      0x10
#define APP_CMD_WRITE_PAGE   0x15
#define APP_CMD_LOAD_CONFIG  0x16
#define APP_CMD_STOP         0xff
#define APP_STAT_OK          0x00
#define APP_STAT_ACCEPTED    0x01   // a measurement is running

// The reference driver waits 3 ms for most of these and 25 ms for a stop;
// a little more here costs nothing when the chip answers on time.
#define CPU_READY_MS     10
#define BL_CMD_MS        10
#define APP_CMD_MS       10
#define APP_MEASURE_MS   10
#define APP_STOP_MS      50

#define WRITE_MAX 256   // longest register write (image chunks)

void tmf8829_init(tmf8829_t *t, i2c_inst_t *i2c) {
    memset(t, 0, sizeof *t);
    t->i2c = i2c;
    t->addr = TMF8829_ADDR;
}

// Generous enough for 100 kHz (90 us per byte).
static uint32_t timeout_us(size_t n) {
    return 2000u + 100u * (uint32_t)n;
}

int tmf8829_read(tmf8829_t *t, uint8_t reg, uint8_t *buf, size_t n) {
    if (i2c_write_timeout_us(t->i2c, t->addr, &reg, 1, true, timeout_us(1)) != 1)
        return TMF8829_ERR_NACK;
    if (i2c_read_timeout_us(t->i2c, t->addr, buf, n, false, timeout_us(n)) != (int)n)
        return TMF8829_ERR_NACK;
    return TMF8829_OK;
}

int tmf8829_write(tmf8829_t *t, uint8_t reg, const uint8_t *buf, size_t n) {
    static uint8_t tx[1 + WRITE_MAX];
    if (n > WRITE_MAX)
        return TMF8829_ERR_STATE;
    tx[0] = reg;
    memcpy(tx + 1, buf, n);
    if (i2c_write_timeout_us(t->i2c, t->addr, tx, n + 1, false, timeout_us(n + 1)) != (int)(n + 1))
        return TMF8829_ERR_NACK;
    return TMF8829_OK;
}

static int write_u8(tmf8829_t *t, uint8_t reg, uint8_t v) {
    return tmf8829_write(t, reg, &v, 1);
}

// Poll `reg` until it reads `want`. The application reports a failed
// command as a status of 2-15; stop waiting on one of those.
static int wait_reg(tmf8829_t *t, uint8_t reg, uint8_t want, uint32_t ms, bool app_status) {
    absolute_time_t until = make_timeout_time_ms(ms);
    for (;;) {
        uint8_t v;
        int err = tmf8829_read(t, reg, &v, 1);
        if (!err && v == want)
            return TMF8829_OK;
        if (!err && app_status && v >= 0x02 && v < 0x10)
            return TMF8829_ERR_STATE;
        if (time_reached(until))
            return err ? err : TMF8829_ERR_TIMEOUT;
        sleep_us(100);
    }
}

// The chip may NACK while it resets or wakes; keep asking.
static int wait_cpu_ready(tmf8829_t *t) {
    absolute_time_t until = make_timeout_time_ms(CPU_READY_MS);
    for (;;) {
        uint8_t en;
        int err = tmf8829_read(t, REG_ENABLE, &en, 1);
        if (!err && (en & ENABLE_CPU_READY))
            return TMF8829_OK;
        if (time_reached(until))
            return err ? err : TMF8829_ERR_TIMEOUT;
        sleep_us(200);
    }
}

int tmf8829_power_up(tmf8829_t *t, bool to_bootloader) {
    uint8_t en;
    int err = tmf8829_read(t, REG_ENABLE, &en, 1);
    if (err)
        return err;
    if (!(en & ENABLE_CPU_READY)) {
        // From cold, powerup_select is 0 and PON starts the bootloader. After
        // a standby of ours it selects RAM, and PON restarts the application.
        uint8_t v = (en & ENABLE_POWERUP_SELECT) == ENABLE_SELECT_RAM ? (uint8_t)((en & ~ENABLE_POFF) | ENABLE_PON)
                                                                       : ENABLE_PON;
        if ((err = write_u8(t, REG_ENABLE, v)) || (err = wait_cpu_ready(t)))
            return err;
    }
    if ((err = tmf8829_read(t, REG_APP_ID, &t->app_id, 1)))
        return err;
    if (to_bootloader && t->app_id != APP_ID_BOOTLOADER) {
        // Point the next start at the bootloader, then soft-reset into it.
        if ((err = write_u8(t, REG_ENABLE, ENABLE_PON | ENABLE_SELECT_BOOTMON)) ||
            (err = write_u8(t, REG_RESET, RESET_SOFT)))
            return err;
        sleep_ms(1);
        if ((err = wait_cpu_ready(t)) || (err = tmf8829_read(t, REG_APP_ID, &t->app_id, 1)))
            return err;
        if (t->app_id != APP_ID_BOOTLOADER)
            return TMF8829_ERR_STATE;
    }
    return TMF8829_OK;
}

int tmf8829_download_begin(tmf8829_t *t) {
    if (t->app_id != APP_ID_BOOTLOADER)
        return TMF8829_ERR_STATE;
    // Only I2C is wired. (The reference driver ignores this one's status too.)
    write_u8(t, REG_CMD_STAT, BL_CMD_SPI_OFF);
    wait_reg(t, REG_CMD_STAT, 0, BL_CMD_MS, false);

    uint32_t a = TMF8829_IMAGE_START;
    uint16_t words = (uint16_t)((TMF8829_IMAGE_LENGTH + 3) / 4);
    uint8_t cmd[8] = {BL_CMD_FIFO_BOTH, 6, (uint8_t)a, (uint8_t)(a >> 8), (uint8_t)(a >> 16), (uint8_t)(a >> 24),
                      (uint8_t)words, (uint8_t)(words >> 8)};
    int err = tmf8829_write(t, REG_CMD_STAT, cmd, sizeof cmd);
    if (!err)
        err = wait_reg(t, REG_CMD_STAT, 0, BL_CMD_MS, false);
    t->download_pos = 0;
    return err;
}

int tmf8829_download_step(tmf8829_t *t, size_t chunk) {
    size_t left = TMF8829_IMAGE_LENGTH - t->download_pos;
    size_t n = chunk < left ? chunk : left;
    if (n > WRITE_MAX)
        n = WRITE_MAX;
    int err = tmf8829_write(t, REG_FIFO, tmf8829_image + t->download_pos, n);
    if (err)
        return err;
    t->download_pos += (uint32_t)n;
    return t->download_pos >= TMF8829_IMAGE_LENGTH ? 1 : 0;
}

uint8_t tmf8829_download_pct(const tmf8829_t *t) {
    return (uint8_t)(100u * t->download_pos / TMF8829_IMAGE_LENGTH);
}

int tmf8829_download_finish(tmf8829_t *t) {
    int err = write_u8(t, REG_CMD_STAT, BL_CMD_START_RAM_APP);
    if (!err)
        err = wait_reg(t, REG_APP_ID, APP_ID_APPLICATION, BL_CMD_MS + 10, false);
    if (err)
        return err;
    t->app_id = APP_ID_APPLICATION;
    // Wake-ups from standby restart the application in RAM from now on.
    uint8_t en;
    if ((err = tmf8829_read(t, REG_ENABLE, &en, 1)) ||
        (err = write_u8(t, REG_ENABLE, (uint8_t)((en & ~ENABLE_POWERUP_SELECT) | ENABLE_SELECT_RAM))))
        return err;
    return tmf8829_read_device_info(t);
}

int tmf8829_read_device_info(tmf8829_t *t) {
    uint8_t id[2], sn[4];
    int err;
    if ((err = tmf8829_read(t, REG_ID, id, 2)) || (err = tmf8829_read(t, REG_APP_ID, t->app_version, 4)))
        return err;
    t->chip_id = id[0];
    t->rev_id = id[1] & 0x07;
    t->app_id = t->app_version[0];
    if (t->app_id != APP_ID_APPLICATION)
        return TMF8829_ERR_STATE;
    if ((err = tmf8829_read(t, REG_SERIAL, sn, 4)))
        return err;
    t->serial = (uint32_t)sn[0] | (uint32_t)sn[1] << 8 | (uint32_t)sn[2] << 16 | (uint32_t)sn[3] << 24;
    return TMF8829_OK;
}

static int command(tmf8829_t *t, uint8_t cmd, uint8_t want, uint32_t ms) {
    int err = write_u8(t, REG_CMD_STAT, cmd);
    return err ? err : wait_reg(t, REG_CMD_STAT, want, ms, true);
}

int tmf8829_load_preset(tmf8829_t *t, uint8_t preset) {
    if (preset < TMF8829_PRESET_FIRST || preset > TMF8829_PRESET_LAST)
        return TMF8829_ERR_STATE;
    int err = command(t, preset, APP_STAT_OK, APP_CMD_MS);
    return err ? err : wait_reg(t, REG_CID_RID, preset, APP_CMD_MS, false);
}

static int load_config_page(tmf8829_t *t) {
    int err = command(t, APP_CMD_LOAD_CONFIG, APP_STAT_OK, APP_CMD_MS);
    return err ? err : wait_reg(t, REG_CID_RID, APP_CMD_LOAD_CONFIG, APP_CMD_MS, false);
}

int tmf8829_config_read(tmf8829_t *t, uint8_t page[TMF8829_CFG_SIZE]) {
    int err = load_config_page(t);
    return err ? err : tmf8829_read(t, REG_CFG_FIRST, page, TMF8829_CFG_SIZE);
}

int tmf8829_config_write(tmf8829_t *t, const uint8_t page[TMF8829_CFG_SIZE]) {
    // The page must be loaded before it can be written back.
    int err = load_config_page(t);
    if (!err)
        err = tmf8829_write(t, REG_CFG_FIRST, page, TMF8829_CFG_SIZE);
    return err ? err : command(t, APP_CMD_WRITE_PAGE, APP_STAT_OK, APP_CMD_MS);
}

uint8_t tmf8829_loaded_preset(tmf8829_t *t) {
    uint8_t v = 0;
    tmf8829_read(t, REG_CID_RID, &v, 1);
    return v;
}

int tmf8829_start(tmf8829_t *t) {
    int err;
    if ((err = write_u8(t, REG_INT_STATUS, 0xff)) ||
        (err = write_u8(t, REG_INT_ENAB, TMF8829_INT_RESULTS | TMF8829_INT_HISTOGRAMS)))
        return err;
    return command(t, APP_CMD_MEASURE, APP_STAT_ACCEPTED, APP_MEASURE_MS);
}

int tmf8829_stop(tmf8829_t *t) {
    // The timed standby power mode sleeps between measurements: wake it first.
    uint8_t en;
    int err = tmf8829_read(t, REG_ENABLE, &en, 1);
    if (!err && !(en & ENABLE_CPU_READY)) {
        write_u8(t, REG_ENABLE, (uint8_t)((en & ~ENABLE_POFF) | ENABLE_PON));
        err = wait_cpu_ready(t);
    }
    if (!err)
        err = command(t, APP_CMD_STOP, APP_STAT_OK, APP_STOP_MS);
    write_u8(t, REG_INT_ENAB, 0);
    write_u8(t, REG_INT_STATUS, 0xff);
    return err;
}

int tmf8829_standby(tmf8829_t *t) {
    uint8_t en;
    int err = tmf8829_read(t, REG_ENABLE, &en, 1);
    if (err || !(en & ENABLE_CPU_READY))
        return err;
    return write_u8(t, REG_ENABLE, en | ENABLE_POFF);
}

int tmf8829_take_interrupts(tmf8829_t *t, uint8_t mask, uint8_t *set) {
    uint8_t v;
    int err = tmf8829_read(t, REG_INT_STATUS, &v, 1);
    *set = err ? 0 : (uint8_t)(v & mask);
    if (*set)
        err = write_u8(t, REG_INT_STATUS, *set);  // write 1 to clear
    return err;
}

int tmf8829_read_frame_header(tmf8829_t *t, uint8_t out[TMF8829_PRE_HEADER_SIZE + TMF8829_FRAME_HEADER_SIZE]) {
    return tmf8829_read(t, REG_FIFOSTATUS, out, TMF8829_PRE_HEADER_SIZE + TMF8829_FRAME_HEADER_SIZE);
}

int tmf8829_read_fifo(tmf8829_t *t, uint8_t *buf, size_t n) {
    return tmf8829_read(t, REG_FIFO, buf, n);
}

int tmf8829_read_fifo_status(tmf8829_t *t, uint8_t *status) {
    return tmf8829_read(t, REG_FIFOSTATUS, status, 1);
}
