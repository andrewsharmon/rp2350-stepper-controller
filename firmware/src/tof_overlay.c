#include "tof_overlay.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"

#include "board_pins.h"
#include "control.h"
#include "tof.h"

#define DEFAULT_KILO_ITERATIONS 10   // the preset's 586 is for the full range
#define DEFAULT_PERIOD_MS       20
#define DEFAULT_SMOOTH_MS       100
#define DEFAULT_DEADBAND        4.0f
#define DEFAULT_MIN_CONF        1

typedef enum { OFF, STARTING, RUNNING } state_t;

static tof_overlay_cfg_t cfg;
static bool cfg_ready;
static state_t state;
static bool kicked;              // tof_on called (or the sensor was already starting)
static bool started_measuring;   // measuring because of us, with no stream
static uint32_t axes_mask;       // axes sent moves since starting
static float filt_mm[TOF_MAP_MAX_AXES];  // per slice
static float sent[TOF_MAP_MAX_AXES];     // last target sent per slice, full steps
static bool have_filt;
static uint32_t last_us;
static uint32_t measurements, moves_sent;
static float interval_s;          // smoothed time between measurements
static uint32_t last_cols, last_rows;

#ifdef BOARD_AXIS_ROW
static const uint8_t board_row[] = BOARD_AXIS_ROW;
#endif

static void ensure_cfg(void);

void tof_overlay_board_order(uint8_t n) {
    ensure_cfg();
    if (n > NUM_MOTORS)
        n = NUM_MOTORS;
    uint8_t k = 0;
#ifdef BOARD_AXIS_ROW
    for (uint32_t j = 0; j < sizeof board_row && k < TOF_MAP_MAX_AXES; j++)
        if (board_row[j] < n)
            cfg.map.order[k++] = board_row[j];
#endif
    if (k < n) {
        // No row for the board, or it doesn't list every axis: in order.
        k = 0;
        for (uint8_t a = 0; a < n && a < TOF_MAP_MAX_AXES; a++)
            cfg.map.order[k++] = a;
    }
    cfg.map.axes = k;
}

static void ensure_cfg(void) {
    if (cfg_ready)
        return;
    cfg_ready = true;
    tof_map_defaults(&cfg.map, NUM_MOTORS);
    tof_overlay_board_order(NUM_MOTORS);
    cfg.preset = TMF8829_PRESET_16X16;
    cfg.kilo_iterations = DEFAULT_KILO_ITERATIONS;
    cfg.period_ms = DEFAULT_PERIOD_MS;
    cfg.smooth_ms = DEFAULT_SMOOTH_MS;
    cfg.deadband = DEFAULT_DEADBAND;
    cfg.min_conf = DEFAULT_MIN_CONF;
}

tof_overlay_cfg_t *tof_overlay_cfg(void) {
    ensure_cfg();
    return &cfg;
}

bool tof_overlay_active(void) {
    return state != OFF;
}

bool tof_overlay_running(void) {
    return state == RUNNING;
}

const char *tof_overlay_state(void) {
    return state == OFF ? "off" : state == STARTING ? "starting sensor" : "running";
}

static void on_measurement(const tof_measurement_t *m) {
    if (state != RUNNING)
        return;
    uint32_t now = time_us_32();
    float dt = have_filt ? (float)(now - last_us) * 1e-6f : 0.0f;
    last_us = now;
    last_cols = m->cols;
    last_rows = m->rows;
    measurements++;
    if (have_filt)
        interval_s = interval_s > 0.0f ? interval_s + 0.1f * (dt - interval_s) : dt;

    float mm[TOF_MAP_MAX_AXES];
    tof_map_distances(&cfg.map, m->dist, m->cols, m->rows, m->unit_mm, mm);
    float alpha = !have_filt || cfg.smooth_ms == 0 ? 1.0f : 1.0f - expf(-dt * 1000.0f / (float)cfg.smooth_ms);
    have_filt = true;
    for (uint32_t i = 0; i < cfg.map.axes; i++) {
        filt_mm[i] += alpha * (mm[i] - filt_mm[i]);
        float target = tof_map_steps(&cfg.map, filt_mm[i]);
        if (cfg.map.order[i] >= NUM_MOTORS)
            continue;
        uint32_t bit = 1u << cfg.map.order[i];
        if ((axes_mask & bit) && fabsf(target - sent[i]) < cfg.deadband)
            continue;
        control_cmd_t c = {.type = CMD_MOVE, .axes = (uint16_t)bit, .pos = motion_steps_to_units(target)};
        if (!control_post(&c))
            break;  // queue full: the next measurement catches up
        sent[i] = target;
        axes_mask |= bit;
        moves_sent++;
    }
}

const char *tof_overlay_apply_sensor(void) {
    ensure_cfg();
    tof_info_t t;
    tof_get_info(&t);
    if (t.state != TOF_IDLE && t.state != TOF_MEASURING)
        return state == STARTING ? NULL : "sensor not ready";
    const char *err = tof_preset(cfg.preset);
    if (err)
        return err;
    uint8_t page[TMF8829_CFG_SIZE];
    memcpy(page, tof_config(), sizeof page);
    page[TMF8829_CFG_PERIOD_MS] = (uint8_t)cfg.period_ms;
    page[TMF8829_CFG_PERIOD_MS + 1] = (uint8_t)(cfg.period_ms >> 8);
    page[TMF8829_CFG_KILO_ITERATIONS] = (uint8_t)cfg.kilo_iterations;
    page[TMF8829_CFG_KILO_ITERATIONS + 1] = (uint8_t)(cfg.kilo_iterations >> 8);
    return tof_config_write(page);
}

const char *tof_overlay_start(void) {
    ensure_cfg();
    if (!tof_available())
        return "no Qwiic I2C on this board";
    if (!control_outputs_on)
        return "outputs are stopped (c to clear)";
    if (state != OFF)
        return NULL;
    static control_snapshot_t s;  // ~1 KB: off core 0's stack
    control_snapshot(&s);
    for (uint32_t i = 0; i < cfg.map.axes; i++) {
        uint8_t a = cfg.map.order[i];
        if (a < NUM_MOTORS && (s.mode[a] == MODE_GROUP || s.mode[a] == MODE_CAM))
            return "an axis is in a group or following a cam";
    }
    tof_info_t t;
    tof_get_info(&t);
    kicked = t.state == TOF_BOOTING;
    started_measuring = false;
    axes_mask = 0;
    have_filt = false;
    measurements = moves_sent = 0;
    interval_s = 0.0f;
    state = STARTING;
    return NULL;
}

void tof_overlay_stop(const char *why) {
    if (state == OFF)
        return;
    tof_set_listener(NULL, 0);
    if (axes_mask) {
        control_cmd_t c = {.type = CMD_STOP, .axes = (uint16_t)axes_mask};
        control_post(&c);
    }
    tof_info_t t;
    tof_get_info(&t);
    if (started_measuring && t.state == TOF_MEASURING && t.stream == TOF_STREAM_NONE)
        tof_measure(false, TOF_STREAM_NONE);
    state = OFF;
    if (why)
        printf("tof map off: %s\n", why);
}

static void fail(const char *what, const char *why) {
    static char msg[96];
    snprintf(msg, sizeof msg, "%s%s%s", what, why ? ": " : "", why ? why : "");
    tof_overlay_stop(msg);
}

void tof_overlay_poll(uint32_t now) {
    (void)now;
    if (state == OFF)
        return;
    tof_info_t t;
    tof_get_info(&t);
    if (state == RUNNING) {
        if (t.state != TOF_MEASURING)
            fail("sensor", t.state == TOF_ERROR ? tof_error_name(t.error) : "stopped measuring");
        return;
    }
    switch (t.state) {
    case TOF_OFF:
    case TOF_ERROR: {
        if (kicked) {
            fail("sensor", t.state == TOF_ERROR ? tof_error_name(t.error) : "switched off");
            return;
        }
        kicked = true;
        const char *err = tof_on(false);
        if (err)
            fail("sensor", err);
        return;
    }
    case TOF_BOOTING:
        kicked = true;
        return;
    case TOF_IDLE:
    case TOF_MEASURING:
        break;
    }
    const char *err = tof_overlay_apply_sensor();
    if (!err && t.state == TOF_IDLE) {
        err = tof_measure(true, TOF_STREAM_NONE);
        started_measuring = !err;
    }
    if (err) {
        fail("sensor set-up", err);
        return;
    }
    tof_set_listener(on_measurement, cfg.min_conf);
    state = RUNNING;
    printf("tof map running: %.0f-%.0f mm -> %.0f..%.0f steps on %u axes\n", (double)cfg.map.near_mm,
           (double)cfg.map.far_mm, (double)cfg.map.steps_near, (double)cfg.map.steps_far, cfg.map.axes);
}

void tof_overlay_print_status(void) {
    ensure_cfg();
    const tof_map_cfg_t *m = &cfg.map;
    printf("tof map: %s\n", tof_overlay_state());
    printf("  range %.0f-%.0f mm -> %.0f..%.0f full steps\n", (double)m->near_mm, (double)m->far_mm,
           (double)m->steps_near, (double)m->steps_far);
    printf("  axes %s:", m->mirror ? "right to left (mirrored)" : "left to right");
    for (uint32_t i = 0; i < m->axes; i++)
        printf("%s%u", i ? "," : " ", m->order[i] + 1);
    putchar('\n');
    printf("  band %.0f%% of the rows x %.0f%% of the columns", (double)(m->band_height * 100.0f),
           (double)(m->band_width * 100.0f));
    if (last_cols) {
        uint32_t c0, c1, r0, r1;
        tof_map_band(m, last_cols, last_rows, &c0, &c1, &r0, &r1);
        printf(": rows %lu-%lu, columns %lu-%lu of %lux%lu", (unsigned long)r0, (unsigned long)r1,
               (unsigned long)c0, (unsigned long)c1, (unsigned long)last_cols, (unsigned long)last_rows);
    }
    printf("\n  sensor preset 0x%02x, %u k iterations, period %u ms, min confidence %u\n", cfg.preset,
           cfg.kilo_iterations, cfg.period_ms, cfg.min_conf);
    printf("  smoothing %u ms, deadband %.1f steps\n", cfg.smooth_ms, (double)cfg.deadband);
    if (state != RUNNING)
        return;
    printf("  %lu measurements (%.1f/s), %lu moves sent\n", (unsigned long)measurements,
           interval_s > 0.0f ? (double)(1.0f / interval_s) : 0.0, (unsigned long)moves_sent);
    if (!have_filt)
        return;
    printf("  ax:");
    for (uint32_t i = 0; i < m->axes; i++)
        printf(" %5u", m->order[i] + 1);
    printf("\n  mm:");
    for (uint32_t i = 0; i < m->axes; i++)
        printf(" %5.0f", (double)filt_mm[i]);
    printf("\n  to:");
    for (uint32_t i = 0; i < m->axes; i++)
        printf(" %5.0f", (double)sent[i]);
    putchar('\n');
}
