#include "programs.h"

#include <math.h>
#include <stdio.h>

#define A PROGRAM_AMP
#define TWO_PI (2.0 * M_PI)

// Axis position along the row, 0 (axis 1) to 1 (the last axis).
static double row(int i, int n) {
    return n > 1 ? (double)i / (double)(n - 1) : 0.5;
}

static double wave(double cycles) {
    return sin(TWO_PI * cycles);
}

// Minimum-jerk step 0 -> 1 over u = 0..1 (flat outside).
static double ease(double u) {
    if (u <= 0.0)
        return 0.0;
    if (u >= 1.0)
        return 1.0;
    return u * u * u * (u * (6.0 * u - 15.0) + 10.0);
}

static double wrap(double t, double period) {
    t = fmod(t, period);
    if (t < 0.0)
        t += period;
    return t < period ? t : 0.0;  // a tiny negative t rounds up to `period`
}

// 1. All axes rise and fall together, slowly.
static double breathe(int i, int n, double t) {
    (void)i, (void)n;
    return A * wave(t / 8.0);
}

// 2. One wavelength across the row, travelling from axis 1 to the last.
static double travel(int i, int n, double t) {
    return A * wave(t / 6.0 - (double)i / (double)n);
}

// 3. Standing wave: ends and middle in antiphase, nodes between.
static double standing(int i, int n, double t) {
    return A * cos(TWO_PI * row(i, n)) * wave(t / 6.0);
}

// 4. Rings spreading out from the middle of the row.
static double ripple(int i, int n, double t) {
    double d = fabs(2.0 * row(i, n) - 1.0);
    return A * wave(t / 6.0 - 0.75 * d);
}

// 5. The row as a straight line, rocking about its middle.
static double seesaw(int i, int n, double t) {
    return A * (2.0 * row(i, n) - 1.0) * wave(t / 6.0);
}

// 6. Odd and even axes in antiphase, then in phase, drifting between.
static double zipper(int i, int n, double t) {
    (void)n;
    double phase = (i & 1) ? 0.25 + 0.25 * wave(t / 32.0) : 0.0;
    return A * wave(t / 8.0 + phase);
}

// 7. Each axis in turn rises the full stroke, holds, then falls in turn.
static double domino(int i, int n, double t) {
    double t0 = 2.7 * row(i, n);
    return A * (-1.0 + 2.0 * (ease((t - t0) / 3.2) - ease((t - 6.0 - t0) / 3.2)));
}

// 8. Up and down a staircase of five levels, each axis a beat behind the last.
static double stairs(int i, int n, double t) {
    static const double level[8] = {-1.0, -0.5, 0.0, 0.5, 1.0, 0.5, 0.0, -0.5};
    double tl = wrap(t - 1.2 * row(i, n), 16.0);
    int k = (int)(tl / 2.0);
    double from = level[k], to = level[(k + 1) % 8];
    return A * (from + (to - from) * ease((tl - 2.0 * k) / 1.4));
}

// 9. A hump sweeping back and forth along the row, running off each end.
static double scan(int i, int n, double t) {
    double c = 0.5 - 0.65 * cos(TWO_PI * t / 18.0);
    double d = (row(i, n) - c) / 0.35;
    return A * (-0.75 + 1.5 * exp(-d * d));
}

// 10. Bumps chasing each other along the row and wrapping round.
static double chase(int i, int n, double t) {
    double u = wrap(t - 7.5 * (double)i / (double)n, 7.5) / 4.0;
    double s = u < 1.0 ? sin(M_PI * u) : 0.0;
    return A * (-0.6 + 1.2 * s * s);
}

// 11. Two waves of different speed travelling opposite ways.
static double interference(int i, int n, double t) {
    double x = row(i, n);
    return 0.5 * A * (wave(t / 6.0 - x) + wave(t / 5.0 + 1.5 * x));
}

// 12. Each axis wanders on its own: three incommensurate-looking waves per
// axis (they all repeat every 60 s) with per-axis phases.
static double drift(int i, int n, double t) {
    (void)n;
    double g = 0.618034 * (double)(i + 1);
    return A * (0.5 * wave(7.0 * t / 60.0 + g) + 0.3 * wave(11.0 * t / 60.0 + 2.0 * g + 0.3) +
                0.2 * wave(17.0 * t / 60.0 + 3.0 * g + 0.7));
}

const program_t programs[PROGRAM_COUNT] = {
    {"breathe", 8.0, 850, 700, breathe},
    {"travel", 6.0, 1100, 1200, travel},
    {"standing", 6.0, 1100, 1200, standing},
    {"ripple", 6.0, 1100, 1200, ripple},
    {"seesaw", 6.0, 1100, 1200, seesaw},
    {"zipper", 32.0, 1150, 1300, zipper},
    {"domino", 12.0, 1250, 1250, domino},
    {"stairs", 16.0, 750, 1600, stairs},
    {"scan", 18.0, 900, 1350, scan},
    {"chase", 7.5, 1000, 1600, chase},
    {"interference", 30.0, 1250, 1450, interference},
    {"drift", 60.0, 1150, 1350, drift},
};

// Pattern time after t seconds: the clock rate follows ease() from 0 to 1
// over the warm-up, so this is its integral (W * (u^6 - 3u^5 + 2.5u^4)),
// then runs at full rate.
static double warp(double t) {
    const double W = PROGRAM_WARMUP_S;
    if (t <= 0.0)
        return 0.0;
    if (t >= W)
        return t - 0.5 * W;
    double u = t / W;
    return W * u * u * u * u * (u * (u - 3.0) + 2.5);
}

double program_pos(const program_t *p, int axis, int n_axes, double t) {
    return p->pos(axis, n_axes, wrap(warp(t), p->period_s));
}

show_point_t program_point(const program_t *p, int axis, int n_axes, uint64_t i) {
    const double h = 1e-3;
    double t = (double)i * (PROGRAM_POINT_MS / 1000.0);
    double vel = (program_pos(p, axis, n_axes, t + h) - program_pos(p, axis, n_axes, t - h)) / (2.0 * h);
    return (show_point_t){
        .t_ms = (uint32_t)(i * PROGRAM_POINT_MS),  // wraps; the player sends differences
        .pos = (float)program_pos(p, axis, n_axes, t),
        .vel = (float)vel,
    };
}

const char *program_check_limits(const program_t *p, int n_axes, const float *vmax,
                                 const float *amax, char *msg, uint32_t msg_len) {
    for (int i = 0; i < n_axes; i++) {
        if (p->vpeak > vmax[i] || p->apeak > amax[i]) {
            snprintf(msg, msg_len, "%s needs vmax %.0f, amax %.0f; axis %d has %.0f, %.0f",
                     p->name, (double)p->vpeak, (double)p->apeak, i + 1, (double)vmax[i],
                     (double)amax[i]);
            return msg;
        }
    }
    return NULL;
}
