#include "tof_frame.h"

#include <string.h>

uint32_t tof_record_size(uint8_t layout) {
    uint32_t peaks = layout & TOF_LAYOUT_PEAKS;
    uint32_t peak = 3 + (layout & TOF_LAYOUT_SIGNAL ? 2 : 0);
    return peaks * peak + (layout & TOF_LAYOUT_NOISE ? 2 : 0) + (layout & TOF_LAYOUT_XTALK ? 2 : 0);
}

void tof_grid(uint8_t fp_mode, uint32_t *cols, uint32_t *rows_per_frame, uint32_t *frames) {
    switch (fp_mode) {
    case 0:  // 8x8A
    case 1:  // 8x8B
        *cols = 8, *rows_per_frame = 8, *frames = 1;
        break;
    case 2:
        *cols = 16, *rows_per_frame = 16, *frames = 1;
        break;
    case 3:  // 32x32, 2 SPADs per zone
    case 4:  // 32x32s, 1 SPAD
        *cols = 32, *rows_per_frame = 16, *frames = 2;
        break;
    default:  // 48x32
        *cols = 48, *rows_per_frame = 16, *frames = 2;
        break;
    }
}

void tof_clock_reset(tof_clock_t *c) {
    memset(c, 0, sizeof *c);
    c->ratio = 1.0f;
}

void tof_clock_add(tof_clock_t *c, uint32_t host_us, uint32_t chip_ticks, uint32_t ticks_per_ms) {
    c->idx = (c->idx + 1) % TOF_CLOCK_PAIRS;
    c->host_us[c->idx] = host_us;
    c->ticks[c->idx] = chip_ticks;
    if (c->n < TOF_CLOCK_PAIRS)
        c->n++;
    if (c->n < TOF_CLOCK_PAIRS)
        return;
    uint32_t oldest = (c->idx + 1) % TOF_CLOCK_PAIRS;
    uint32_t dh = c->host_us[c->idx] - c->host_us[oldest];   // wraps cleanly
    uint32_t dt = c->ticks[c->idx] - c->ticks[oldest];
    if (dh < 1000 || dt < ticks_per_ms)
        return;  // under a millisecond apart: too coarse to judge
    float r = (float)dh * (float)ticks_per_ms / ((float)dt * 1000.0f);
    if (r >= 0.5f && r <= 1.5f)
        c->ratio = r;
}

static uint16_t get16(const uint8_t *p) {
    return (uint16_t)(p[0] | p[1] << 8);
}

void tof_correct_distances(uint8_t *records, size_t n, uint8_t layout, float ratio) {
    uint32_t size = tof_record_size(layout);
    uint32_t peaks = layout & TOF_LAYOUT_PEAKS;
    uint32_t first = (layout & TOF_LAYOUT_NOISE ? 2 : 0) + (layout & TOF_LAYOUT_XTALK ? 2 : 0);
    uint32_t step = 3 + (layout & TOF_LAYOUT_SIGNAL ? 2 : 0);
    if (size == 0 || ratio == 1.0f)
        return;
    for (size_t at = 0; at + size <= n; at += size) {
        for (uint32_t k = 0; k < peaks; k++) {
            uint8_t *p = records + at + first + k * step;
            float d = (float)get16(p) * ratio + 0.5f;
            uint32_t v = d >= 65535.0f ? 65535u : (uint32_t)d;
            p[0] = (uint8_t)v;
            p[1] = (uint8_t)(v >> 8);
        }
    }
}

void tof_summary_reset(tof_summary_t *s) {
    memset(s, 0, sizeof *s);
    s->min_raw = UINT32_MAX;
}

void tof_summary_add(tof_summary_t *s, const uint8_t *records, size_t n, uint8_t layout) {
    uint32_t size = tof_record_size(layout);
    if (size == 0 || (layout & TOF_LAYOUT_PEAKS) == 0)
        return;
    uint32_t first = (layout & TOF_LAYOUT_NOISE ? 2 : 0) + (layout & TOF_LAYOUT_XTALK ? 2 : 0);
    for (size_t at = 0; at + size <= n; at += size) {
        const uint8_t *p = records + at + first;
        s->zones++;
        if (p[2] == 0)
            continue;  // confidence 0: no target in this zone
        uint32_t d = get16(p);
        s->valid++;
        s->sum_raw += d;
        if (d < s->min_raw)
            s->min_raw = d;
        if (d > s->max_raw)
            s->max_raw = d;
    }
}

void tof_zones_add(uint16_t *grid, uint32_t cols, uint32_t rows, uint32_t frames, uint32_t sub, uint32_t first,
                   const uint8_t *records, size_t n, uint8_t layout, uint8_t min_conf) {
    uint32_t size = tof_record_size(layout);
    if (size == 0 || (layout & TOF_LAYOUT_PEAKS) == 0 || cols == 0)
        return;
    if (min_conf < 1)
        min_conf = 1;
    uint32_t skip = (layout & TOF_LAYOUT_NOISE ? 2 : 0) + (layout & TOF_LAYOUT_XTALK ? 2 : 0);
    uint32_t z = first;
    for (size_t at = 0; at + size <= n; at += size, z++) {
        const uint8_t *p = records + at + skip;
        uint32_t y = z / cols;
        uint32_t row = frames == 2 ? 2 * y + sub : y;
        if (row >= rows)
            return;
        uint16_t d = get16(p);
        grid[row * cols + z % cols] = p[2] >= min_conf ? (d ? d : 1) : 0;
    }
}
