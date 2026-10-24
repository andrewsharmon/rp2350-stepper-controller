#include "tof.h"

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"

#include "board_pins.h"
#include "i2c_target.h"
#include "protocol.h"
#include "tof_frame.h"

#define POLL_US        1000     // interrupt status reads while measuring
#define CHUNK_MAX      224      // FIFO bytes (or image bytes) per tof_poll pass...
#define CHUNK_US       2000     // ...or about this long on the bus, whichever is less
#define BUS_ERRORS_MAX 20       // in a row while measuring, then give up
#define TEXT_LINE_US   200000   // console summary lines: at most 5 a second
#define FRAME_MAX      32768    // sanity limit on a frame's payload (histograms ~25 KB)

#define HDR(k) (fr.hdr[TMF8829_PRE_HEADER_SIZE + (k)])

static tmf8829_t dev;
static tof_state_t state;
static tof_error_t error;
static uint32_t bus_khz = TOF_BUS_KHZ_DEFAULT;
static bool bus_is_host;
static bool clock_correction = true;
static tof_stream_t stream;
static uint8_t config[TMF8829_CFG_SIZE];
static bool user_config;   // `config` was written by the user: re-apply it after a restart
static uint8_t preset;     // last preset loaded
static tof_clock_t clk;
static uint32_t frames, bad_frames, bus_errors, errors_in_row;
static uint32_t last_check_us, last_line_us, frame_read_us;
static bool fifo_more;     // a frame just ended: the next one may already be queued

// The frame being read.
static struct {
    bool active, results;
    uint8_t hdr[TMF8829_PRE_HEADER_SIZE + TMF8829_FRAME_HEADER_SIZE];
    uint8_t layout;
    uint32_t record;   // bytes per zone record (result frames)
    uint32_t body;     // bytes between the header and the footer
    uint32_t offset;   // bytes read after the header
    uint32_t left;     // bytes still in the FIFO
    uint8_t footer[TMF8829_FRAME_FOOTER_SIZE];
    uint32_t start_us;
} fr;
static tof_summary_t summary;

static const char *const state_names[] = {"off", "starting", "idle", "measuring", "error"};
static const char *const error_names[] = {
    "none",
    "no Qwiic I2C on this board",
    "no answer at 0x41 (sensor power, EN high, pull-ups, wiring)",
    "sensor CPU never got ready",
    "bootloader not reached or refused a command (power-cycle the sensor)",
    "firmware download failed",
    "sensor application didn't start",
    "preset or configuration refused",
    "measurement wouldn't start or stop",
    "repeated bus errors while measuring",
};

const char *tof_state_name(tof_state_t s) {
    return (unsigned)s < sizeof state_names / sizeof *state_names ? state_names[s] : "?";
}

const char *tof_error_name(tof_error_t e) {
    return (unsigned)e < sizeof error_names / sizeof *error_names ? error_names[e] : "?";
}

// --- the Qwiic port: I2C target or sensor host ---------------------------------------

#ifdef BOARD_I2C_SDA_GPIO

bool tof_available(void) {
    return true;
}

static void bus_take(void) {
    if (bus_is_host) {
        i2c_set_baudrate(i2c1, bus_khz * 1000u);
        return;
    }
    i2c_target_release();
    gpio_set_function(BOARD_I2C_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(BOARD_I2C_SCL_GPIO, GPIO_FUNC_I2C);
    i2c_init(i2c1, bus_khz * 1000u);
    bus_is_host = true;
}

static void bus_release(void) {
    if (!bus_is_host)
        return;
    i2c_deinit(i2c1);
    bus_is_host = false;
    i2c_target_init();
}

#else

bool tof_available(void) {
    return false;
}

static void bus_take(void) {
}

static void bus_release(void) {
}

#endif

// --- start-up -----------------------------------------------------------------------

static void fail(tof_error_t e) {
    error = e;
    state = TOF_ERROR;
    fr.active = false;
    printf("tof: %s\n", tof_error_name(e));
}

// Bytes per pass: ~CHUNK_US on the bus (9 bit times a byte), whole zone
// records so the clock correction never splits one.
static uint32_t chunk_bytes(uint32_t record) {
    uint32_t n = bus_khz * CHUNK_US / 9000u;
    if (n < 16)
        n = 16;
    if (n > CHUNK_MAX)
        n = CHUNK_MAX;
    if (record > n)
        return record;
    return record ? n - n % record : n;
}

// The application runs: load the preset (and the user's page on top of it,
// after a restart), then read the page back.
static void configure(void) {
    tmf8829_stop(&dev);  // it may still be measuring from before a board reset
    if (!preset)
        preset = TMF8829_PRESET_8X8;
    int err = tmf8829_load_preset(&dev, preset);
    if (!err && user_config)
        err = tmf8829_config_write(&dev, config);
    if (!err)
        err = tmf8829_config_read(&dev, config);
    if (err) {
        fail(TOF_ERR_CONFIG);
        return;
    }
    state = TOF_IDLE;
    printf("tof: ready, application %u.%u.%u.%u, chip %u rev %u, serial %08lx\n", dev.app_version[0],
           dev.app_version[1], dev.app_version[2], dev.app_version[3], dev.chip_id, dev.rev_id,
           (unsigned long)dev.serial);
}

const char *tof_on(bool reload_firmware) {
    if (!tof_available())
        return tof_error_name(TOF_ERR_NO_PORT);
    if (state == TOF_BOOTING)
        return "already starting";
    if ((state == TOF_IDLE || state == TOF_MEASURING) && !reload_firmware)
        return NULL;
    fr.active = false;
    error = TOF_ERR_NONE;
    bus_take();
    tmf8829_init(&dev, i2c1);
    int err = tmf8829_power_up(&dev, reload_firmware);
    if (err) {
        fail(err == TMF8829_ERR_NACK ? TOF_ERR_NO_ANSWER : err == TMF8829_ERR_TIMEOUT ? TOF_ERR_CPU
                                                                                     : TOF_ERR_BOOTLOADER);
        return tof_error_name(error);
    }
    if (dev.app_id == 0x01) {
        // The application survived a board reset: no download needed.
        if (tmf8829_read_device_info(&dev)) {
            fail(TOF_ERR_APP);
            return tof_error_name(error);
        }
        configure();
        return state == TOF_IDLE ? NULL : tof_error_name(error);
    }
    if (tmf8829_download_begin(&dev)) {
        fail(TOF_ERR_BOOTLOADER);
        return tof_error_name(error);
    }
    state = TOF_BOOTING;
    return NULL;
}

void tof_off(void) {
    if (state == TOF_OFF)
        return;
    if (state == TOF_MEASURING)
        tmf8829_stop(&dev);
    if (state == TOF_IDLE || state == TOF_MEASURING)
        tmf8829_standby(&dev);
    state = TOF_OFF;
    error = TOF_ERR_NONE;
    fr.active = false;
    bus_release();
}

// --- measuring ------------------------------------------------------------------------

static const char *not_ready(void) {
    switch (state) {
    case TOF_OFF:     return "sensor is off (tof on)";
    case TOF_BOOTING: return "sensor still starting";
    case TOF_ERROR:   return tof_error_name(error);
    default:          return NULL;
    }
}

const char *tof_measure(bool on, tof_stream_t s) {
    if (!on) {
        if (state != TOF_MEASURING)
            return NULL;
        fr.active = false;
        state = TOF_IDLE;
        return tmf8829_stop(&dev) ? "stop not confirmed by the sensor" : NULL;
    }
    stream = s;
    if (state == TOF_MEASURING)
        return NULL;
    const char *why = not_ready();
    if (why)
        return why;
    tof_clock_reset(&clk);
    errors_in_row = 0;
    fifo_more = false;
    if (tmf8829_start(&dev))
        return "sensor refused to start measuring";
    state = TOF_MEASURING;
    last_check_us = time_us_32();
    return NULL;
}

// Preset and page changes need the measurement stopped; it restarts after.
static const char *pause(bool *was_measuring) {
    const char *why = not_ready();
    if (why)
        return why;
    *was_measuring = state == TOF_MEASURING;
    tof_measure(false, stream);
    return NULL;
}

static const char *resume(bool was_measuring, const char *result) {
    if (was_measuring && !result)
        return tof_measure(true, stream);
    return result;
}

const char *tof_preset(uint8_t p) {
    if (p < TMF8829_PRESET_FIRST || p > TMF8829_PRESET_LAST)
        return "no such preset";
    bool was;
    const char *why = pause(&was);
    if (why)
        return why;
    int err = tmf8829_load_preset(&dev, p);
    if (!err) {
        preset = p;
        user_config = false;
    }
    if (tmf8829_config_read(&dev, config) && !err)
        err = TMF8829_ERR_STATE;
    return resume(was, err ? "preset refused" : NULL);
}

const char *tof_config_write(const uint8_t page[TMF8829_CFG_SIZE]) {
    bool was;
    const char *why = pause(&was);
    if (why)
        return why;
    int err = tmf8829_config_write(&dev, page);
    if (!err)
        user_config = true;
    tmf8829_config_read(&dev, config);   // whatever the chip kept
    return resume(was, err ? "configuration refused by the sensor" : NULL);
}

const char *tof_set_bus_khz(uint32_t khz) {
    if (khz < 100 || khz > 1000)
        return "bus speed 100-1000 kHz";
    bus_khz = khz;
    if (bus_is_host)
        i2c_set_baudrate(i2c1, bus_khz * 1000u);
    return NULL;
}

void tof_set_clock_correction(bool on) {
    clock_correction = on;
    tof_clock_reset(&clk);
}

const uint8_t *tof_config(void) {
    return config;
}

// --- frames ---------------------------------------------------------------------------

static void bus_error(void) {
    bus_errors++;
    fr.active = false;
    if (++errors_in_row >= BUS_ERRORS_MAX)
        fail(TOF_ERR_BUS);
}

static uint32_t get32(const uint8_t *p) {
    return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}

// Sub-frame index and frame count of the result frame in fr.
static void frame_position(uint32_t *sub, uint32_t *count, uint32_t *cols, uint32_t *rows) {
    uint32_t rows_per_frame;
    tof_grid(HDR(0) & 0x0f, cols, &rows_per_frame, count);
    *sub = fr.layout & TOF_LAYOUT_SUBFRAME ? 1 : 0;
    *rows = rows_per_frame * *count;
}

static void frame_begin(uint32_t now) {
    if (tmf8829_read_frame_header(&dev, fr.hdr)) {
        bus_error();
        return;
    }
    uint32_t host_us = time_us_32();
    errors_in_row = 0;
    uint8_t fid = HDR(0) & TMF8829_FID_MASK;
    uint32_t payload = (uint32_t)(HDR(2) | HDR(3) << 8);
    if (fid != TMF8829_FID_RESULTS && fid != TMF8829_FID_HISTOGRAMS) {
        if (!(fr.hdr[0] & TMF8829_FIFOSTATUS_TX_EMPTY))
            bad_frames++;  // something, but not a frame we know
        return;
    }
    if (payload < TMF8829_FRAME_HEADER_SIZE - 4 + TMF8829_FRAME_FOOTER_SIZE || payload > FRAME_MAX) {
        bad_frames++;
        return;
    }
    fr.results = fid == TMF8829_FID_RESULTS;
    fr.layout = HDR(1);
    fr.record = fr.results ? tof_record_size(fr.layout) : 0;
    fr.left = payload - (TMF8829_FRAME_HEADER_SIZE - 4);
    fr.body = fr.left - TMF8829_FRAME_FOOTER_SIZE;
    fr.offset = 0;
    fr.start_us = now;
    fr.active = true;
    if (fr.results) {
        tof_clock_add(&clk, host_us, get32(fr.hdr + 1), TMF8829_TICKS_PER_MS);
        uint32_t sub, count, cols, rows;
        frame_position(&sub, &count, &cols, &rows);
        if (sub == 0)
            tof_summary_reset(&summary);
    }
    if (stream == TOF_STREAM_BINARY)
        protocol_send_tof_header(host_us, fr.hdr, clock_correction ? clk.ratio : 1.0f);
}

static void print_summary(uint32_t now) {
    uint32_t sub, count, cols, rows;
    frame_position(&sub, &count, &cols, &rows);
    if (sub + 1 < count || now - last_line_us < TEXT_LINE_US)
        return;
    last_line_us = now;
    float unit = fr.footer[9] & TOF_FOOTER_MM ? 1.0f : 0.25f;
    printf("tof %lu  %lux%lu  %lu/%lu zones", (unsigned long)get32(&HDR(4)), (unsigned long)cols,
           (unsigned long)rows, (unsigned long)summary.valid, (unsigned long)summary.zones);
    if (summary.valid)
        printf("  min %.0f  mean %.0f  max %.0f mm", (double)(summary.min_raw * unit),
               (double)(summary.sum_raw * unit / summary.valid), (double)(summary.max_raw * unit));
    printf("  %d C\n", (int8_t)HDR(8));
}

static void frame_end(uint32_t now) {
    fr.active = false;
    bool eof = (fr.footer[10] | fr.footer[11] << 8) == TMF8829_FRAME_EOF;
    if (eof)
        frames++;
    else
        bad_frames++;
    if (fr.results)
        frame_read_us = now - fr.start_us;
    if (stream == TOF_STREAM_BINARY)
        protocol_send_tof_end(eof ? 0 : 1, fr.offset, now - fr.start_us);
    else if (fr.results && eof)
        print_summary(now);
    fifo_more = true;
}

static void frame_step(uint32_t now) {
    static uint8_t buf[CHUNK_MAX];
    uint32_t n = chunk_bytes(fr.record);
    if (n > fr.left)
        n = fr.left;
    if (tmf8829_read_fifo(&dev, buf, n)) {
        // The FIFO is out of step now: restart the measurement to resync.
        bad_frames++;
        bus_error();
        if (state == TOF_MEASURING) {
            tmf8829_stop(&dev);
            if (tmf8829_start(&dev))
                fail(TOF_ERR_MEASURE);
        }
        return;
    }
    uint32_t body_left = fr.offset < fr.body ? fr.body - fr.offset : 0;
    uint32_t in_body = n < body_left ? n : body_left;
    if (fr.results) {
        if (clock_correction)
            tof_correct_distances(buf, in_body, fr.layout, clk.ratio);
        tof_summary_add(&summary, buf, in_body, fr.layout);
    }
    for (uint32_t k = in_body; k < n; k++)
        fr.footer[fr.offset + k - fr.body] = buf[k];
    if (stream == TOF_STREAM_BINARY)
        protocol_send_tof_data((uint16_t)fr.offset, buf, n);
    fr.offset += n;
    fr.left -= n;
    if (fr.left == 0)
        frame_end(now);
}

void tof_poll(uint32_t now) {
    if (state == TOF_BOOTING) {
        int r = tmf8829_download_step(&dev, chunk_bytes(0));
        if (r < 0)
            fail(TOF_ERR_DOWNLOAD);
        else if (r == 1 && tmf8829_download_finish(&dev))
            fail(TOF_ERR_APP);
        else if (r == 1)
            configure();
        return;
    }
    if (state != TOF_MEASURING)
        return;
    if (fr.active) {
        frame_step(now);
        return;
    }
    if (fifo_more) {
        // Both sub-frames can land before one status poll; the second one
        // shares the interrupt already cleared.
        fifo_more = false;
        uint8_t fs;
        if (!tmf8829_read_fifo_status(&dev, &fs) && !(fs & TMF8829_FIFOSTATUS_TX_EMPTY))
            frame_begin(now);
        return;
    }
    if (now - last_check_us < POLL_US)
        return;
    last_check_us = now;
    uint8_t set;
    if (tmf8829_take_interrupts(&dev, TMF8829_INT_RESULTS | TMF8829_INT_HISTOGRAMS, &set)) {
        bus_error();
        return;
    }
    errors_in_row = 0;
    if (set)
        frame_begin(now);
}

// --- status ---------------------------------------------------------------------------

void tof_get_info(tof_info_t *o) {
    memset(o, 0, sizeof *o);
    o->state = state;
    o->error = error;
    memcpy(o->app_version, dev.app_version, sizeof o->app_version);
    o->chip_id = dev.chip_id;
    o->rev_id = dev.rev_id;
    o->serial = dev.serial;
    o->bus_khz = bus_khz;
    o->clock_correction = clock_correction;
    o->binary = stream == TOF_STREAM_BINARY;
    o->preset = preset;
    o->download_pct = state == TOF_BOOTING ? tmf8829_download_pct(&dev) : state == TOF_OFF ? 0 : 100;
    o->frames = frames;
    o->bad_frames = bad_frames;
    o->bus_errors = bus_errors;
    o->clock_ratio = clk.ratio;
    o->frame_read_us = frame_read_us;
}

void tof_print_status(void) {
    printf("tof: %s", tof_state_name(state));
    if (state == TOF_ERROR)
        printf(" (%s)", tof_error_name(error));
    if (state == TOF_OFF) {
        printf(", Qwiic port is the I2C target\n");
        return;
    }
    printf(", bus %lu kHz, clock correction %s (x%.5f)\n", (unsigned long)bus_khz, clock_correction ? "on" : "off",
           (double)clk.ratio);
    if (dev.app_id == 0x01)
        printf("  application %u.%u.%u.%u, chip %u rev %u, serial %08lx, preset 0x%02x\n", dev.app_version[0],
               dev.app_version[1], dev.app_version[2], dev.app_version[3], dev.chip_id, dev.rev_id,
               (unsigned long)dev.serial, preset);
    uint32_t cols, rows, nf;
    tof_grid(config[TMF8829_CFG_FP_MODE] & 7, &cols, &rows, &nf);
    printf("  %lux%lu zones, period %u ms, %u k iterations, %u peak(s)\n", (unsigned long)cols,
           (unsigned long)(rows * nf), config[TMF8829_CFG_PERIOD_MS] | config[TMF8829_CFG_PERIOD_MS + 1] << 8,
           config[TMF8829_CFG_KILO_ITERATIONS] | config[TMF8829_CFG_KILO_ITERATIONS + 1] << 8,
           config[TMF8829_CFG_RESULT_FORMAT] & TOF_LAYOUT_PEAKS);
    printf("  %lu frames, %lu bad, %lu bus errors, last result frame read in %lu us\n", (unsigned long)frames,
           (unsigned long)bad_frames, (unsigned long)bus_errors, (unsigned long)frame_read_us);
}
