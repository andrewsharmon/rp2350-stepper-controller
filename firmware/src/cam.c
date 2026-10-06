#include "cam.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "motion.h"

#define UNITS ((double)MOTION_UNITS_PER_STEP)

const char *cam_build(cam_table_t *t, const float *x, const float *y, uint32_t n, bool cyclic) {
    if (n < 2 || n > CAM_MAX_POINTS)
        return "a cam table needs 2 to 128 points";
    for (uint32_t i = 0; i < n; i++) {
        if (!isfinite(x[i]) || !isfinite(y[i]))
            return "cam point is not a number";
        if (i > 0 && !(x[i] > x[i - 1]))
            return "cam x values must increase";
    }
    memset(t, 0, sizeof *t);
    t->n = (uint16_t)n;
    t->cyclic = cyclic;
    memcpy(t->x, x, n * sizeof *x);
    memcpy(t->y, y, n * sizeof *y);
    double period = (double)x[n - 1] - x[0], rise = (double)y[n - 1] - y[0];
    t->x0_units = (int64_t)llround(x[0] * UNITS);
    t->period_units = (int64_t)llround(period * UNITS);
    t->rise_units = cyclic ? (int64_t)llround(rise * UNITS) : 0;

    // Catmull-Rom slopes. Cyclic: neighbours wrap (shifted by the period
    // and rise), and the two ends describe the same point, so they share a
    // slope. One-shot: zero at the ends, matching the held value outside.
    for (uint32_t i = 0; i < n; i++) {
        double xp, yp, xn, yn;
        if (i > 0) {
            xp = x[i - 1]; yp = y[i - 1];
        } else if (cyclic) {
            xp = x[n - 2] - period; yp = y[n - 2] - rise;
        } else {
            t->m[i] = 0.0f;
            continue;
        }
        if (i + 1 < n) {
            xn = x[i + 1]; yn = y[i + 1];
        } else if (cyclic) {
            xn = x[1] + period; yn = y[1] + rise;
        } else {
            t->m[i] = 0.0f;
            continue;
        }
        t->m[i] = (float)((yn - yp) / (xn - xp));
    }
    return NULL;
}

// Segment containing u (x[0] <= u <= x[n-1]).
static uint32_t segment(const cam_table_t *t, float u) {
    uint32_t lo = 0, hi = t->n - 1;
    while (hi - lo > 1) {
        uint32_t mid = (lo + hi) / 2;
        if (t->x[mid] <= u)
            lo = mid;
        else
            hi = mid;
    }
    return lo;
}

// Hermite evaluation at u (steps) within the table range; y relative to y[0].
static float eval_local(const cam_table_t *t, float u, float *dydx, float *d2ydx2) {
    uint32_t i = segment(t, u);
    float h = t->x[i + 1] - t->x[i], s = (u - t->x[i]) / h;
    float y0 = t->y[i] - t->y[0], y1 = t->y[i + 1] - t->y[0];
    float m0 = t->m[i] * h, m1 = t->m[i + 1] * h;
    float s2 = s * s, s3 = s2 * s;
    float y = (2 * s3 - 3 * s2 + 1) * y0 + (s3 - 2 * s2 + s) * m0 + (-2 * s3 + 3 * s2) * y1 + (s3 - s2) * m1;
    *dydx = ((6 * s2 - 6 * s) * y0 + (3 * s2 - 4 * s + 1) * m0 + (-6 * s2 + 6 * s) * y1 + (3 * s2 - 2 * s) * m1) / h;
    *d2ydx2 = ((12 * s - 6) * y0 + (6 * s - 4) * m0 + (-12 * s + 6) * y1 + (6 * s - 2) * m1) / (h * h);
    return y;
}

int64_t cam_eval(const cam_table_t *t, int64_t leader, float *dydx, float *d2ydx2) {
    int64_t rel = leader - t->x0_units, cycles = 0;
    if (t->cyclic) {
        // Floor division: negative leader positions count cycles downward.
        cycles = rel / t->period_units;
        rel -= cycles * t->period_units;
        if (rel < 0) {
            rel += t->period_units;
            cycles--;
        }
    } else if (rel <= 0 || rel >= t->period_units) {
        *dydx = *d2ydx2 = 0.0f;
        uint32_t k = rel <= 0 ? 0 : t->n - 1;
        return (int64_t)llround((double)t->y[k] * UNITS);
    }
    // Within one period: u is at most the period, so float keeps ~1e-7
    // relative precision; the integer cycle count carries the rest.
    float u = t->x[0] + (float)((double)rel / UNITS);
    float y = eval_local(t, u, dydx, d2ydx2);
    return (int64_t)llround(((double)t->y[0] + y) * UNITS) + cycles * t->rise_units;
}

const char *cam_check(const cam_table_t *t, float lead_vmax, float lead_amax,
                      float vmax, float amax, char *msg, uint32_t msg_len) {
    for (uint32_t i = 0; i + 1 < t->n; i++) {
        for (int k = 0; k <= 16; k++) {
            float u = t->x[i] + (t->x[i + 1] - t->x[i]) * (float)k / 16.0f, d1, d2;
            eval_local(t, u, &d1, &d2);
            float v = fabsf(d1) * lead_vmax;
            float a = fabsf(d2) * lead_vmax * lead_vmax + fabsf(d1) * lead_amax;
            if (v > vmax * 1.001f || a > amax * 1.01f) {
                snprintf(msg, msg_len, "at leader x = %.1f the follower would need %.0f steps/s, "
                         "%.0f steps/s^2 (limits %.0f, %.0f) with the leader at %.0f steps/s",
                         (double)u, (double)v, (double)a, (double)vmax, (double)amax, (double)lead_vmax);
                return msg;
            }
        }
    }
    return NULL;
}
