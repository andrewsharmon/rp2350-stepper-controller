#include "tof_map.h"

#include <math.h>

#define DEFAULT_NEAR_MM  150.0f
#define DEFAULT_FAR_MM   300.0f
#define DEFAULT_STEPS    1000.0f   // each way from the centre, as the built-in programs
#define DEFAULT_HEIGHT   0.25f     // middle quarter of the rows
#define DEFAULT_WIDTH    1.0f

void tof_map_defaults(tof_map_cfg_t *c, uint8_t axes) {
    c->near_mm = DEFAULT_NEAR_MM;
    c->far_mm = DEFAULT_FAR_MM;
    c->steps_near = DEFAULT_STEPS;
    c->steps_far = -DEFAULT_STEPS;
    c->axes = axes > TOF_MAP_MAX_AXES ? TOF_MAP_MAX_AXES : axes;
    for (uint32_t k = 0; k < TOF_MAP_MAX_AXES; k++)
        c->order[k] = (uint8_t)k;
    c->band_height = DEFAULT_HEIGHT;
    c->band_width = DEFAULT_WIDTH;
    c->mirror = false;
}

// `frac` of `n` zones, centred: first and last.
static void centred(float frac, uint32_t n, uint32_t *first, uint32_t *last) {
    uint32_t k = (uint32_t)lroundf(frac * (float)n);
    if (k < 1)
        k = 1;
    if (k > n)
        k = n;
    *first = (n - k) / 2;
    *last = *first + k - 1;
}

void tof_map_band(const tof_map_cfg_t *c, uint32_t cols, uint32_t rows, uint32_t *c0, uint32_t *c1,
                  uint32_t *r0, uint32_t *r1) {
    centred(c->band_width, cols, c0, c1);
    centred(c->band_height, rows, r0, r1);
}

static float clampf(float v, float lo, float hi) {
    return v < lo ? lo : v > hi ? hi : v;
}

void tof_map_distances(const tof_map_cfg_t *c, const uint16_t *raw, uint32_t cols, uint32_t rows,
                       float unit_mm, float *mm) {
    uint32_t c0, c1, r0, r1;
    tof_map_band(c, cols, rows, &c0, &c1, &r0, &r1);
    float lo = fminf(c->near_mm, c->far_mm), hi = fmaxf(c->near_mm, c->far_mm);
    float width = (float)(c1 - c0 + 1);
    uint32_t n = c->axes;
    for (uint32_t i = 0; i < n; i++) {
        // The slice, in columns from the band's left edge. A column belongs
        // to the slice its centre falls in; a slice narrower than a column
        // takes the column under its middle.
        float x0 = width * (float)i / (float)n, x1 = width * (float)(i + 1) / (float)n;
        uint32_t first = (uint32_t)ceilf(x0 - 0.5f), last = (uint32_t)ceilf(x1 - 0.5f);
        if (last <= first) {
            first = (uint32_t)((x0 + x1) * 0.5f);
            last = first + 1;
        }
        uint32_t best = UINT32_MAX;
        for (uint32_t r = r0; r <= r1; r++)
            for (uint32_t k = first; k < last && c0 + k <= c1; k++) {
                uint16_t d = raw[r * cols + c0 + k];
                if (d && d < best)
                    best = d;
            }
        float v = best == UINT32_MAX ? hi : clampf((float)best * unit_mm, lo, hi);
        mm[c->mirror ? n - 1 - i : i] = v;
    }
}

float tof_map_steps(const tof_map_cfg_t *c, float mm) {
    float span = c->far_mm - c->near_mm;
    if (span == 0.0f)
        return c->steps_near;
    float u = clampf((mm - c->near_mm) / span, 0.0f, 1.0f);
    return c->steps_near + u * (c->steps_far - c->steps_near);
}
