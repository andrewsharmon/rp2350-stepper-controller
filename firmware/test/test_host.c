// Host-side checks for the PWM period encoder and microstep math.
//
//   cc -std=c11 -O1 -Wall -Wextra -I../src test_host.c ../src/hbridge_encode.c ../src/microstep.c ../src/sine_lut.c ../src/ladder.c ../src/motion.c ../src/group.c ../src/frame.c -lm -o test_host && ./test_host
//
// The PIO program is simulated per segment: each 16-bit half-word holds
// pattern bits [3:0] and length [15:4], and lasts length + 3 SM clocks.

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "hbridge_encode.h"
#include "microstep.h"
#include "ladder.h"
#include "motion.h"
#include "group.h"
#include "frame.h"
#include "trig.h"

#define PERIOD_CLOCKS 3750  // 150 MHz / 2 / 20 kHz
#define SPAN (PERIOD_CLOCKS - 4 * HBRIDGE_SEGMENT_OVERHEAD)

typedef struct {
    int total;      // SM clocks in the period
    int a_fwd, a_rev, b_fwd, b_rev;  // clocks each coil is driven each way
    unsigned last;  // pattern held at the end of the period
} sim_t;

static int failures;

#define CHECK(cond, ...)                                            \
    do {                                                            \
        if (!(cond)) {                                              \
            failures++;                                             \
            printf("FAIL %s:%d: ", __FILE__, __LINE__);             \
            printf(__VA_ARGS__);                                    \
            printf("\n");                                           \
        }                                                           \
    } while (0)

static sim_t simulate(uint32_t w0, uint32_t w1) {
    sim_t s = {0};
    uint32_t words[2] = {w0, w1};
    for (int i = 0; i < 4; i++) {
        uint32_t half = (words[i / 2] >> (16 * (i % 2))) & 0xffff;
        unsigned pat = half & 0xf;
        int clocks = (int)(half >> 4) + HBRIDGE_SEGMENT_OVERHEAD;
        unsigned a = pat & 3, b = (pat >> 2) & 3;
        if (a == 1) s.a_fwd += clocks;
        if (a == 2) s.a_rev += clocks;
        if (b == 1) s.b_fwd += clocks;
        if (b == 2) s.b_rev += clocks;
        CHECK(a != 0 && b != 0, "segment %d leaves a coil floating (pattern %x)", i, pat);
        s.total += clocks;
        s.last = pat;
    }
    return s;
}

static void check_period(int32_t da, int32_t db) {
    uint32_t w0, w1;
    hbridge_encode_period(SPAN, da, db, &w0, &w1);
    sim_t s = simulate(w0, w1);

    CHECK(s.total == PERIOD_CLOCKS, "duty %d/%d: period %d clocks", da, db, s.total);
    CHECK(s.last == 0xf, "duty %d/%d: period ends with pattern %x, not brake", da, db, s.last);

    int ea = abs(da) > SPAN ? SPAN : abs(da);
    int eb = abs(db) > SPAN ? SPAN : abs(db);
    int got_a = da >= 0 ? s.a_fwd : s.a_rev;
    int got_b = db >= 0 ? s.b_fwd : s.b_rev;
    int wrong_a = da >= 0 ? s.a_rev : s.a_fwd;
    int wrong_b = db >= 0 ? s.b_rev : s.b_fwd;
    CHECK(wrong_a == 0 && wrong_b == 0, "duty %d/%d: coil driven the wrong way", da, db);
    // Segment overhead adds up to 2 x 3 clocks (80 ns) of drive per coil.
    CHECK(ea == 0 ? got_a == 0 : got_a >= ea && got_a <= ea + 6, "duty %d: coil A driven %d", da, got_a);
    CHECK(eb == 0 ? got_b == 0 : got_b >= eb && got_b <= eb + 6, "duty %d: coil B driven %d", db, got_b);
}

static void test_encoder(void) {
    int32_t samples[] = {0, 1, 37, 38, 500, 1873, SPAN - 1, SPAN, SPAN + 100};
    int n = sizeof samples / sizeof samples[0];
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++)
            for (int sa = -1; sa <= 1; sa += 2)
                for (int sb = -1; sb <= 1; sb += 2)
                    check_period(sa * samples[i], sb * samples[j]);
    for (int k = 0; k < 2000; k++)
        check_period(rand() % (2 * SPAN + 1) - SPAN, rand() % (2 * SPAN + 1) - SPAN);
}

static void test_phase_inc(void) {
    // 100 full steps/s = 25 electrical cycles/s; 20 kHz periods.
    uint32_t inc = microstep_phase_inc(100.0f, 20000);
    double cycles = (double)inc * 20000.0 / 4294967296.0;
    CHECK(fabs(cycles - 25.0) < 1e-3, "100 steps/s -> %f cycles/s", cycles);
    uint32_t rev = microstep_phase_inc(-100.0f, 20000);
    CHECK(rev == (uint32_t)(0u - inc), "reverse increment is not the negation");
}

static void test_microstep(void) {
    const int32_t min_duty = 38;  // 500 ns at 75 MHz
    microstep_t ms = {0};
    ms.amplitude = 0.5f;

    // Peak duty at phase 0 (coil A = cos) and quarter cycle (coil B = sin).
    int32_t a, b;
    microstep_duties(&ms, SPAN, min_duty, &a, &b);
    CHECK(abs(a - SPAN / 2) <= 1 && b == 0, "phase 0: %d/%d", a, b);
    ms.phase = 0x40000000u;
    microstep_duties(&ms, SPAN, min_duty, &a, &b);
    CHECK(abs(b - SPAN / 2) <= 1 && abs(a) < min_duty, "phase 90: %d/%d", a, b);

    // Small duties: every emitted pulse is 0 or >= min_duty, and the long-run
    // average matches the request.
    microstep_t small = {0};
    small.amplitude = 0.005f;  // ~18 counts peak, below min_duty
    small.phase = 0x10000000u;
    double want = 0, got = 0;
    for (int k = 0; k < 20000; k++) {
        microstep_duties(&small, SPAN, min_duty, &a, &b);
        CHECK(a == 0 || abs(a) >= min_duty, "emitted sub-minimum pulse %d", a);
        got += a;
        want += 0.005 * SPAN * cos(2 * M_PI * 0x10000000u / 4294967296.0);
    }
    CHECK(fabs(got - want) / want < 0.01, "dithered average %f vs %f", got, want);
}

static void test_ladder(void) {
    // Nominal levels from the resistor values, +/- 5%.
    struct { uint32_t mv; ladder_level_t want; } levels[] = {
        {3300, LADDER_ESTOP}, {3135, LADDER_ESTOP},
        {2200, LADDER_IDLE}, {2090, LADDER_IDLE}, {2310, LADDER_IDLE},
        {1320, LADDER_BTN1}, {1254, LADDER_BTN1}, {1386, LADDER_BTN1},
        {730, LADDER_BTN2}, {694, LADDER_BTN2}, {767, LADDER_BTN2},
        {600, LADDER_BTN2},  // both buttons: reads as Btn2
        {0, LADDER_FAULT}, {120, LADDER_FAULT},
    };
    for (unsigned i = 0; i < sizeof levels / sizeof levels[0]; i++)
        CHECK(ladder_classify(levels[i].mv) == levels[i].want, "%u mV -> %s",
              levels[i].mv, ladder_level_name(ladder_classify(levels[i].mv)));

    ladder_t l;
    uint32_t t = 0;
    ladder_reset(&l, t);

    // A single noisy stop sample must not trip.
    CHECK(!ladder_update(&l, 3300, t += 10), "one-sample spike tripped");
    CHECK(!ladder_update(&l, 2200, t += 10), "tripped on idle");
    CHECK(!l.stop_latched, "spike latched");

    // A sustained stop trips on the LADDER_STOP_SAMPLES-th sample (<100 us).
    uint32_t start = t;
    int tripped = 0;
    for (int k = 0; k < 10 && !tripped; k++)
        tripped = ladder_update(&l, 3300, t += 10);
    CHECK(tripped && t - start <= 100, "e-stop latency %u us", t - start);
    CHECK(l.stop_cause == LADDER_ESTOP, "stop cause %s", ladder_level_name(l.stop_cause));

    // Cannot clear while still open; can once the line is back to idle.
    CHECK(!ladder_clear_stop(&l), "cleared while e-stop open");
    ladder_update(&l, 2200, t += 10);
    CHECK(ladder_clear_stop(&l), "could not clear at idle");

    // Fault also latches.
    for (int k = 0; k < 3; k++)
        ladder_update(&l, 0, t += 10);
    CHECK(l.stop_latched && l.stop_cause == LADDER_FAULT, "fault not latched");
    ladder_update(&l, 2200, t += 10);
    ladder_clear_stop(&l);

    // Buttons: a 5 ms blip is ignored; a 30 ms press counts once.
    for (uint32_t end = t + 5000; t < end; t += 10)
        ladder_update(&l, 1320, t);
    for (uint32_t end = t + 50000; t < end; t += 10)
        ladder_update(&l, 2200, t);
    CHECK(l.btn1_presses == 0, "blip counted as press");
    for (uint32_t end = t + 30000; t < end; t += 10)
        ladder_update(&l, 730, t);
    for (uint32_t end = t + 30000; t < end; t += 10)
        ladder_update(&l, 2200, t);
    CHECK(l.btn2_presses == 1 && l.btn1_presses == 0, "btn2 presses %u, btn1 %u",
          l.btn2_presses, l.btn1_presses);
    CHECK(!l.stop_latched, "button latched a stop");
}

typedef struct {
    int ticks;
    float max_v, max_a, max_j;  // magnitudes seen on the output
    float overshoot;            // steps past the target (direction of travel)
} run_stats_t;

// Tick until settled (or max_ticks); track limits on the output.
static run_stats_t run_until_settled(motion_axis_t *ax, int max_ticks) {
    run_stats_t st = {0};
    float dir = 0.0f, a_prev = 0.0f;
    int64_t target = ax->target;
    for (; st.ticks < max_ticks && !(st.ticks > 0 && ax->settled); st.ticks++) {
        float v0 = ax->vel;
        motion_tick(ax);
        float a = (ax->vel - v0) * MOTION_TICK_HZ;
        if (!ax->settled) {  // the final tick to rest is a snap, not a ramp
            if (fabsf(a) > st.max_a) st.max_a = fabsf(a);
            float j = fabsf(a - a_prev) * MOTION_TICK_HZ;
            if (st.ticks > 0 && j > st.max_j) st.max_j = j;
        }
        a_prev = a;
        if (fabsf(ax->vel) > st.max_v) st.max_v = fabsf(ax->vel);
        if (dir == 0.0f && ax->vel != 0.0f) dir = ax->vel > 0 ? 1.0f : -1.0f;
        float d = motion_units_to_steps(target - ax->pos);
        if (dir * -d > st.overshoot) st.overshoot = dir * -d;
    }
    return st;
}

static float ideal_time(motion_profile_t p, float d, float vmax, float amax, float tj) {
    switch (p) {
    case PROFILE_COSINE:
        return fmaxf(1.5708f * d / vmax, sqrtf(4.9348f * d / amax));
    case PROFILE_QUINTIC:
        return fmaxf(1.875f * d / vmax, sqrtf(5.7735f * d / amax));
    default: {
        float t = d * amax <= vmax * vmax ? 2.0f * sqrtf(d / amax) : d / vmax + vmax / amax;
        return t + (p == PROFILE_SCURVE ? tj : p == PROFILE_SMOOTH ? 2.0f * tj : 0.0f);
    }
    }
}

static void check_move_profile(motion_profile_t prof, float from, float to, float vmax, float amax) {
    const uint32_t jerk_ms = 30;
    motion_axis_t ax;
    motion_init(&ax, vmax, amax);
    motion_set_profile(&ax, prof, jerk_ms);
    motion_set_position(&ax, motion_steps_to_units(from));
    motion_move_to(&ax, motion_steps_to_units(to));
    run_stats_t st = run_until_settled(&ax, 10000000);

    const char *n = motion_profile_name(prof);
    float d = fabsf(to - from);
    float ideal = ideal_time(prof, d, vmax, amax, jerk_ms / 1000.0f);
    float t = (float)st.ticks / MOTION_TICK_HZ;
    CHECK(ax.settled && ax.pos == motion_steps_to_units(to),
          "%s %g->%g: ended at %f (settled %d)", n, from, to, motion_units_to_steps(ax.pos), ax.settled);
    CHECK(st.max_v <= vmax * 1.002f, "%s %g->%g: speed %f > vmax", n, from, to, st.max_v);
    CHECK(st.max_a <= amax * 1.01f, "%s %g->%g: accel %f > amax", n, from, to, st.max_a);
    CHECK(st.overshoot <= 1e-3f, "%s %g->%g: overshoot %f steps", n, from, to, st.overshoot);
    CHECK(t <= ideal * 1.03f + 0.005f, "%s %g->%g: %.3f s vs ideal %.3f s", n, from, to, t, ideal);
    if (prof == PROFILE_SCURVE || prof == PROFILE_SMOOTH) {
        // One moving average turns each acceleration step into a ramp of
        // jerk amax/Tj; a direct +a to -a reversal (triangle moves) is a
        // step of 2*amax.
        float jmax = 2.0f * amax / (jerk_ms / 1000.0f);
        CHECK(st.max_j <= jmax * 1.1f + 2.0f * MOTION_TICK_HZ, "%s %g->%g: jerk %.0f > %.0f",
              n, from, to, st.max_j, jmax);
    }
}

static void check_move(float from, float to, float vmax, float amax) {
    for (int p = 0; p < PROFILE_COUNT; p++)
        check_move_profile((motion_profile_t)p, from, to, vmax, amax);
}

static void check_cobs(const uint8_t *in, size_t n, const uint8_t *want, size_t wn) {
    uint8_t enc[600], dec[600];
    size_t e = frame_cobs_encode(in, n, enc);
    CHECK(e == wn && memcmp(enc, want, wn) == 0, "cobs encode of %zu bytes", n);
    int d = frame_cobs_decode(enc, e, dec, sizeof dec);
    CHECK(d == (int)n && memcmp(dec, in, n) == 0, "cobs round trip of %zu bytes", n);
}

static void test_frame(void) {
    CHECK(frame_crc16((const uint8_t *)"123456789", 9) == 0x29b1, "crc16 check value");

    // Published COBS examples.
    check_cobs((const uint8_t[]){0x00}, 1, (const uint8_t[]){0x01, 0x01}, 2);
    check_cobs((const uint8_t[]){0x00, 0x00}, 2, (const uint8_t[]){0x01, 0x01, 0x01}, 3);
    check_cobs((const uint8_t[]){0x11, 0x22, 0x00, 0x33}, 4, (const uint8_t[]){0x03, 0x11, 0x22, 0x02, 0x33}, 5);
    check_cobs((const uint8_t[]){0x11, 0x22, 0x33, 0x44}, 4, (const uint8_t[]){0x05, 0x11, 0x22, 0x33, 0x44}, 5);
    check_cobs((const uint8_t[]){0x11, 0x00, 0x00, 0x00}, 4, (const uint8_t[]){0x02, 0x11, 0x01, 0x01, 0x01}, 5);
    uint8_t in[255], want[258];
    for (int i = 0; i < 254; i++) in[i] = (uint8_t)(i + 1);       // 01..FE
    want[0] = 0xff;
    memcpy(want + 1, in, 254);
    check_cobs(in, 254, want, 255);
    in[254] = 0xff;                                                // 01..FF
    want[255] = 0x02;
    want[256] = 0xff;
    check_cobs(in, 255, want, 257);

    // Golden frame, also checked by the Python tool's tests.
    uint8_t wire[FRAME_MAX_WIRE], payload[FRAME_MAX_PAYLOAD], type;
    uint16_t seq;
    const uint8_t pl[] = {0x00, 0x01, 0x02, 0xff};
    size_t n = frame_build(0x10, 0x1234, pl, sizeof pl, wire);
    // Computed independently (bit-by-bit CRC, reference COBS in Python).
    const uint8_t golden[] = {0x00, 0x04, 0x10, 0x34, 0x12, 0x06, 0x01, 0x02, 0xff, 0xe3, 0xe0, 0x00};
    CHECK(n == sizeof golden && memcmp(wire, golden, n) == 0, "golden frame (got %zu bytes)", n);
    if (n != sizeof golden || memcmp(wire, golden, n) != 0) {
        printf("  frame:");
        for (size_t i = 0; i < n; i++) printf(" %02x", wire[i]);
        printf("\n");
    }

    // Random round trips; corruption is rejected.
    srand(7);
    for (int k = 0; k < 2000; k++) {
        uint8_t p[FRAME_MAX_PAYLOAD];
        size_t len = (size_t)(rand() % (FRAME_MAX_PAYLOAD + 1));
        for (size_t i = 0; i < len; i++) p[i] = (uint8_t)(rand() % 4 == 0 ? 0 : rand());
        n = frame_build((uint8_t)k, (uint16_t)(k * 31), p, len, wire);
        CHECK(n >= 4 && wire[0] == 0 && wire[n - 1] == 0 && memchr(wire + 1, 0, n - 2) == NULL,
              "frame %d has a zero inside", k);
        int got = frame_parse(wire + 1, n - 2, &type, &seq, payload);
        CHECK(got == (int)len && type == (uint8_t)k && seq == (uint16_t)(k * 31) &&
              memcmp(payload, p, len) == 0, "frame %d round trip", k);
        wire[1 + (size_t)rand() % (n - 2)] ^= (uint8_t)(1 + rand() % 255);
        got = frame_parse(wire + 1, n - 2, &type, &seq, payload);
        CHECK(got < 0 || !(type == (uint8_t)k && memcmp(payload, p, len) == 0 && got == (int)len),
              "frame %d corruption accepted", k);
    }
}

static void test_trig(void) {
    double worst = 0, worst_d = 0;
    for (int k = -20000; k <= 20000; k++) {
        float x = (float)k * 0.001f;  // +/- 20 rad, through every quadrant boundary
        double e = fabs((double)trig_sinf(x) - sin((double)x));
        double ec = fabs((double)trig_cosf(x) - cos((double)x));
        if (e > worst) worst = e;
        if (ec > worst) worst = ec;
    }
    for (int k = 0; k <= 1000; k++) {
        double u = k / 1000.0;
        double e = fabs(trig_cos_pi_d(u) - cos(TRIG_PI * u));
        if (e > worst_d) worst_d = e;
    }
    // Float range reduction by 2pi dominates far from 0; motion code calls
    // trig_sin_core directly on [-pi/2, pi/2].
    CHECK(worst < 2e-6, "trig_sinf/cosf error %g", worst);
    CHECK(worst_d < 1e-12, "trig_cos_pi_d error %g", worst_d);
}

static void test_pvt(void) {
    // Stream A(1 - cos wt) (starts at rest; 200 steps, 1 Hz) as 20 ms points.
    const float A = 200.0f, w = 2.0f * (float)M_PI;
    motion_axis_t ax;
    motion_init(&ax, 1500, 2000);  // s-curve profile: PVT must bypass it
    int k = 1;
    for (; k <= 20; k++) {
        float t = k * 0.02f;
        CHECK(motion_pvt_push(&ax, motion_steps_to_units(A * (1 - cosf(w * t))), A * w * sinf(w * t), 20),
              "push %d refused", k);
    }
    CHECK(motion_pvt_start(&ax), "pvt start refused");
    CHECK(!motion_pvt_start(&ax), "pvt start allowed while moving");
    float max_err = 0, max_a = 0, v_prev = 0;
    for (int n = 1; n <= 3000; n++) {  // 3 s, refilling as we go
        motion_tick(&ax);
        if (ax.pvt_count < 16 && k <= 150) {
            float t = k * 0.02f;
            motion_pvt_push(&ax, motion_steps_to_units(A * (1 - cosf(w * t))), A * w * sinf(w * t), 20);
            k++;
        }
        float t = n / 1000.0f;
        float err = fabsf(motion_units_to_steps(ax.pos) - A * (1 - cosf(w * t)));
        if (n <= 2900 && err > max_err) max_err = err;
        float a = fabsf(ax.vel - v_prev) * 1000.0f;
        if (n > 1 && a > max_a) max_a = a;
        v_prev = ax.vel;
    }
    // Cubic Hermite through exact samples of a sine: error ~ (h w)^4 A / 384.
    CHECK(max_err < 0.01f, "pvt tracking error %f steps", max_err);
    CHECK(max_a < A * w * w * 1.1f, "pvt accel %f vs %f", max_a, A * w * w);
    // Stream ended at t = 3.0 s, at rest by coincidence; add a moving end.
    motion_init(&ax, 1500, 2000);
    motion_pvt_push(&ax, motion_steps_to_units(20), 400, 100);
    motion_pvt_start(&ax);
    for (int n = 0; n < 100; n++)  // the segment itself accelerates at 4000
        motion_tick(&ax);
    run_stats_t st = run_until_settled(&ax, 10000);
    CHECK(ax.settled && ax.pvt_underruns == 1, "underrun: settled %d, count %u", ax.settled, ax.pvt_underruns);
    CHECK(st.max_a <= 2000.0f * 1.01f, "underrun brake accel %f", st.max_a);

    // A stream ending at v = 0 stops exactly on its last point.
    motion_init(&ax, 1500, 2000);
    motion_pvt_push(&ax, motion_steps_to_units(10), 300, 50);
    motion_pvt_push(&ax, motion_steps_to_units(25.5f), 0, 100);
    motion_pvt_start(&ax);
    run_until_settled(&ax, 1000);
    CHECK(ax.settled && ax.pos == motion_steps_to_units(25.5f) && ax.pvt_underruns == 0,
          "pvt end at %f, underruns %u", motion_units_to_steps(ax.pos), ax.pvt_underruns);

    // Queue capacity.
    motion_init(&ax, 1500, 2000);
    unsigned pushed = 0;
    while (motion_pvt_push(&ax, 0, 0, 10) && pushed < 1000)
        pushed++;
    CHECK(pushed == MOTION_PVT_QUEUE, "queue took %u points", pushed);

    // Another command abandons the stream.
    motion_pvt_start(&ax);
    motion_tick(&ax);
    motion_move_to(&ax, motion_steps_to_units(5));
    run_until_settled(&ax, 2000);
    CHECK(ax.pos == motion_steps_to_units(5) && ax.pvt_count == 0, "move after pvt ended at %f",
          motion_units_to_steps(ax.pos));
}

// Run a group to completion on trapezoid axes (no filter, so the path itself
// is checked). Tracks per-axis speed / acceleration excess and the largest
// per-tick speed jump.
typedef struct {
    int ticks;
    float max_v_ratio, max_a_ratio;   // worst |v|/vmax, |a|/amax over axes
    float min_v_mid;                  // lowest path speed seen at a junction crossing
} group_stats_t;

static group_stats_t run_group(group_t *g, motion_axis_t *axes, int max_ticks) {
    group_stats_t st = {0};
    st.min_v_mid = INFINITY;
    float vprev[GROUP_MAX_AXES];
    for (uint32_t k = 0; k < g->n; k++)
        vprev[k] = axes[g->axis[k]].vel;  // may start mid-move
    uint32_t done = g->segments_done;
    for (; st.ticks < max_ticks; st.ticks++) {
        group_tick(g, axes);
        for (uint32_t k = 0; k < g->n; k++)
            motion_tick(&axes[g->axis[k]]);
        for (uint32_t k = 0; k < g->n; k++) {
            motion_axis_t *ax = &axes[g->axis[k]];
            float vr = fabsf(ax->vel) / ax->vmax;
            float ar = fabsf(ax->vel - vprev[k]) * MOTION_TICK_HZ / ax->amax;
            if (vr > st.max_v_ratio) st.max_v_ratio = vr;
            if (ar > st.max_a_ratio) st.max_a_ratio = ar;
            vprev[k] = ax->vel;
        }
        if (g->segments_done != done && g->count > 0 && g->v < st.min_v_mid)
            st.min_v_mid = g->v;  // just crossed into the next segment
        done = g->segments_done;
        bool settled = true;
        for (uint32_t k = 0; k < g->n; k++)
            settled &= axes[g->axis[k]].settled;
        if (group_idle(g) && settled)
            break;
    }
    return st;
}

static void group_axes_init(motion_axis_t *axes, int n) {
    for (int i = 0; i < n; i++) {
        motion_init(&axes[i], 1500, 2000);
        motion_set_profile(&axes[i], PROFILE_TRAP, 0);
    }
}

static void test_group(void) {
    motion_axis_t axes[3];
    group_t g;
    const uint8_t xy[2] = {0, 1}, xyz[3] = {0, 1, 2};
    const float corner = 0.02f;  // default corner time

    // Square: corners land exactly; on each side the other axis stays put.
    group_axes_init(axes, 2);
    CHECK(group_create(&g, 0, axes, xy, 2, 1000), "create refused");
    int64_t sq[4][2] = {{1000, 0}, {1000, 1000}, {0, 1000}, {0, 0}};
    for (int k = 0; k < 4; k++) {
        int64_t t[2] = {motion_steps_to_units(sq[k][0]), motion_steps_to_units(sq[k][1])};
        group_line(&g, axes, t, 0);
    }
    int64_t y_side1_max = 0;
    for (int n = 0; n < 1000; n++) {  // first side: y must not move
        group_tick(&g, axes);
        motion_tick(&axes[0]);
        motion_tick(&axes[1]);
        int64_t y = axes[1].pos < 0 ? -axes[1].pos : axes[1].pos;
        if (y > y_side1_max) y_side1_max = y;
    }
    group_stats_t st = run_group(&g, axes, 100000);
    CHECK(y_side1_max == 0, "square: y moved %lld units on the x side", (long long)y_side1_max);
    CHECK(group_idle(&g) && axes[0].pos == 0 && axes[1].pos == 0, "square ended at %f,%f",
          motion_units_to_steps(axes[0].pos), motion_units_to_steps(axes[1].pos));
    CHECK(st.max_v_ratio <= 1.001f, "square: speed %.3f x vmax", st.max_v_ratio);
    // Square corners: a full stop is needed (90 degrees, corner time 20 ms
    // allows only amax*0.02 = 40 steps/s of jump).
    // A corner may jump each axis's speed by amax * corner (one tick), so
    // the per-tick acceleration seen is at most amax * corner * 1000 = 20x.
    CHECK(st.max_a_ratio <= corner * MOTION_TICK_HZ * 1.001f, "square: accel %.3f x amax", st.max_a_ratio);
    CHECK(g.segments_done == 4, "square: %u segments done", g.segments_done);

    // Diagonal with unequal limits: stays on the line, arrives together.
    group_axes_init(axes, 3);
    axes[1].vmax = 300;  // slow axis sets the pace
    axes[2].amax = 500;
    group_create(&g, 1, axes, xyz, 3, 5000);
    int64_t tgt[3] = {motion_steps_to_units(800), motion_steps_to_units(400), motion_steps_to_units(-200)};
    group_line(&g, axes, tgt, 0);
    float worst_off = 0;
    for (int n = 0; n < 20000 && !(group_idle(&g) && axes[0].settled); n++) {
        group_tick(&g, axes);
        for (int k = 0; k < 3; k++)
            motion_tick(&axes[k]);
        // Distance from the line through the origin and the target.
        float p[3], t[3] = {800, 400, -200}, tt = 0, pt = 0;
        for (int k = 0; k < 3; k++) {
            p[k] = motion_units_to_steps(axes[k].pos);
            tt += t[k] * t[k];
            pt += p[k] * t[k];
        }
        float off = 0;
        for (int k = 0; k < 3; k++) {
            float e = p[k] - t[k] * pt / tt;
            off += e * e;
        }
        if (sqrtf(off) > worst_off) worst_off = sqrtf(off);
        CHECK(fabsf(axes[1].vel) <= 300.0f * 1.001f, "diagonal: axis 2 at %f", axes[1].vel);
    }
    CHECK(worst_off < 0.01f, "diagonal: %f steps off the line", worst_off);
    for (int k = 0; k < 3; k++)
        CHECK(axes[k].pos == tgt[k], "diagonal: axis %d ended at %f", k + 1, motion_units_to_steps(axes[k].pos));

    // Circle of 64 chords: flows through the junctions without stopping.
    group_axes_init(axes, 2);
    group_create(&g, 2, axes, xy, 2, 800);
    for (int k = 1; k <= 64; k++) {
        float a = 2.0f * (float)M_PI * k / 64.0f;
        int64_t t[2] = {motion_steps_to_units(500.0f * (cosf(a) - 1.0f)), motion_steps_to_units(500.0f * sinf(a))};
        if (k == 64) t[0] = t[1] = 0;
        if (!group_line(&g, axes, t, 0)) {
            run_group(&g, axes, 200);  // make room
            group_line(&g, axes, t, 0);
        }
        for (int n = 0; n < 40; n++) {  // keep streaming while it runs
            group_tick(&g, axes);
            motion_tick(&axes[0]);
            motion_tick(&axes[1]);
        }
    }
    st = run_group(&g, axes, 100000);
    CHECK(axes[0].pos == 0 && axes[1].pos == 0, "circle ended at %f,%f",
          motion_units_to_steps(axes[0].pos), motion_units_to_steps(axes[1].pos));
    // Chord turn 5.6 deg: per-axis jump allowed amax*0.02 = 40 steps/s at
    // |du| ~ 0.098, so ~400 steps/s at junctions; must never stop mid-circle.
    CHECK(st.max_v_ratio <= 1.001f, "circle: speed %.3f x vmax", st.max_v_ratio);

    // Full circle as an arc: back exactly at the start, always on the
    // circle within the chord tolerance, never stopping on the way.
    group_axes_init(axes, 2);
    group_create(&g, 2, axes, xy, 2, 800);
    int64_t ctr[2] = {motion_steps_to_units(-500), 0};
    CHECK(group_arc(&g, ctr, 2.0 * M_PI, 0, 0.05f), "arc refused");
    float worst_r = 0;
    st.min_v_mid = INFINITY;
    for (int n = 0; n < 100000 && !group_idle(&g); n++) {
        group_tick(&g, axes);
        motion_tick(&axes[0]);
        motion_tick(&axes[1]);
        float x = motion_units_to_steps(axes[0].pos) + 500.0f, y = motion_units_to_steps(axes[1].pos);
        float dr = fabsf(sqrtf(x * x + y * y) - 500.0f);
        if (dr > worst_r) worst_r = dr;
        if (n > 1000 && g.arc_active && g.v < st.min_v_mid) st.min_v_mid = g.v;
    }
    CHECK(worst_r < 0.06f, "arc: %f steps off the circle", worst_r);
    CHECK(axes[0].pos == 0 && axes[1].pos == 0, "arc ended at %f,%f",
          motion_units_to_steps(axes[0].pos), motion_units_to_steps(axes[1].pos));
    CHECK(st.min_v_mid > 700.0f, "arc slowed to %f mid-circle", st.min_v_mid);

    // Collinear segments: no slowdown at the joint.
    group_axes_init(axes, 2);
    group_create(&g, 3, axes, xy, 2, 1000);
    int64_t c1[2] = {motion_steps_to_units(1000), motion_steps_to_units(1000)};
    int64_t c2[2] = {motion_steps_to_units(2000), motion_steps_to_units(2000)};
    group_line(&g, axes, c1, 0);
    group_line(&g, axes, c2, 0);
    st = run_group(&g, axes, 100000);
    CHECK(st.min_v_mid > 999.0f, "collinear: slowed to %f at the joint", st.min_v_mid);
    CHECK(axes[0].pos == c2[0] && axes[1].pos == c2[1], "collinear end");

    // Hold mid-move, resume, finish exactly. Then stop flushes the queue.
    group_axes_init(axes, 2);
    group_create(&g, 0, axes, xy, 2, 1000);
    int64_t h1[2] = {motion_steps_to_units(3000), 0};
    group_line(&g, axes, h1, 0);
    for (int n = 0; n < 800; n++) { group_tick(&g, axes); motion_tick(&axes[0]); motion_tick(&axes[1]); }
    group_hold(&g, true);
    st = run_group(&g, axes, 2000);  // runs until held (not idle): bounded
    CHECK(g.v == 0.0f && g.running && axes[0].pos < h1[0], "hold: v %f", g.v);
    // Float path distance on a 3000-step segment adds ~0.5% tick-to-tick noise.
    CHECK(st.max_a_ratio <= 1.01f, "hold decel %.3f x amax", st.max_a_ratio);
    group_hold(&g, false);
    run_group(&g, axes, 100000);
    CHECK(axes[0].pos == h1[0], "resume ended at %f", motion_units_to_steps(axes[0].pos));
    int64_t h2[2] = {0, motion_steps_to_units(3000)};
    group_line(&g, axes, h2, 0);
    for (int n = 0; n < 800; n++) { group_tick(&g, axes); motion_tick(&axes[0]); motion_tick(&axes[1]); }
    group_stop(&g);
    run_group(&g, axes, 100000);
    CHECK(group_idle(&g) && axes[1].pos > 0 && axes[1].pos < h2[1], "stop: idle %d at %f,%f",
          group_idle(&g), motion_units_to_steps(axes[0].pos), motion_units_to_steps(axes[1].pos));
    int64_t h3[2] = {0, 0};
    group_line(&g, axes, h3, 0);
    run_group(&g, axes, 100000);
    CHECK(axes[0].pos == 0 && axes[1].pos == 0, "after stop, line from current position");

    // Stop when the queue ends before the brake does: the group must not
    // be left held.
    int64_t h4[2] = {motion_steps_to_units(30), 0};
    group_line(&g, axes, h4, 0);
    for (int n = 0; n < 150; n++) { group_tick(&g, axes); motion_tick(&axes[0]); motion_tick(&axes[1]); }
    group_stop(&g);
    run_group(&g, axes, 100000);
    CHECK(group_idle(&g) && !g.hold && !g.flush_on_stop, "stop at queue end left hold %d", g.hold);
    group_line(&g, axes, h3, 0);
    run_group(&g, axes, 100000);
    CHECK(axes[0].pos == 0 && axes[1].pos == 0, "group stuck after stop at queue end");

    // Ownership: release only at rest, then axes are free again.
    CHECK(group_release(&g, axes) && axes[0].group == -1 && axes[0].mode == MODE_IDLE, "release");
    motion_set_velocity(&axes[0], 100);
    motion_tick(&axes[0]);
    CHECK(!group_create(&g, 0, axes, xy, 2, 1000), "create allowed with a moving axis");
    (void)corner;
}

static void test_motion(void) {
    check_move(0, 1000, 1500, 2000);     // trapezoid
    check_move(0, 10, 1500, 2000);       // triangle
    check_move(0, 0.01f, 1500, 2000);    // tiny
    check_move(500, -300, 800, 5000);
    check_move(0, 1e6f, 1600, 2000);     // long: float precision far from target
    check_move(-12345.678f, 12345.678f, 1500, 2000);

    // Retarget mid-move: reverse smoothly, arrive exactly, for every profile.
    for (int p = 0; p < PROFILE_COUNT; p++) {
        motion_axis_t ax;
        motion_init(&ax, 1500, 2000);
        motion_set_profile(&ax, (motion_profile_t)p, 30);
        motion_move_to(&ax, motion_steps_to_units(1000));
        for (int k = 0; k < 500; k++)
            motion_tick(&ax);
        float v_at_switch = ax.vel;
        motion_move_to(&ax, motion_steps_to_units(-200));
        run_stats_t st = run_until_settled(&ax, 100000);
        const char *n = motion_profile_name((motion_profile_t)p);
        CHECK(v_at_switch > 500.0f, "%s retarget: only %f steps/s at switch", n, v_at_switch);
        CHECK(ax.pos == motion_steps_to_units(-200), "%s retarget: ended at %f", n,
              motion_units_to_steps(ax.pos));
        CHECK(st.max_a <= 2000.0f * 1.01f, "%s retarget: accel %f", n, st.max_a);
        CHECK(st.max_v <= 1500.0f * 1.002f, "%s retarget: speed %f", n, st.max_v);
    }

    // Jog (trapezoid): runs while refreshed, then decelerates to a stop.
    motion_axis_t ax;
    motion_init(&ax, 1500, 2000);
    motion_set_profile(&ax, PROFILE_TRAP, 0);
    motion_jog(&ax, 500, 100);
    for (int k = 0; k < 300; k++)
        motion_tick(&ax);
    CHECK(ax.settled && ax.vel == 0.0f, "jog did not time out (mode %d, v %f)", ax.mode, ax.vel);
    // Ramp up is cut at 100 ms (v = 200), then 0.1 s down: 10 + 10 steps.
    float p = motion_units_to_steps(ax.pos);
    CHECK(fabsf(p - 20.0f) < 0.5f, "jog travel %f steps", p);

    // Same jog through the s-curve filter: same travel, exactly.
    motion_axis_t ax2;
    motion_init(&ax2, 1500, 2000);
    motion_jog(&ax2, 500, 100);
    run_until_settled(&ax2, 1000);
    CHECK(ax2.pos == ax.pos, "s-curve jog travel %f vs trapezoid %f",
          motion_units_to_steps(ax2.pos), p);

    // Velocity, then stop: decelerates and settles.
    motion_init(&ax, 1500, 2000);
    motion_set_profile(&ax, PROFILE_TRAP, 0);
    motion_set_velocity(&ax, -800);
    for (int k = 0; k < 1000; k++)
        motion_tick(&ax);
    CHECK(fabsf(ax.vel + 800.0f) < 1e-3f, "velocity mode at %f", ax.vel);
    motion_stop(&ax);
    run_stats_t st = run_until_settled(&ax, 10000);
    CHECK(ax.settled && st.ticks >= 399 && st.ticks <= 401, "stop took %d ticks", st.ticks);

    // Stop during a quintic move hands over to the ramp without a jump.
    motion_init(&ax, 1500, 2000);
    motion_set_profile(&ax, PROFILE_QUINTIC, 0);
    motion_move_to(&ax, motion_steps_to_units(2000));
    for (int k = 0; k < 600; k++)
        motion_tick(&ax);
    motion_stop(&ax);
    st = run_until_settled(&ax, 10000);
    CHECK(ax.settled && st.max_a <= 2000.0f * 1.01f, "quintic stop: accel %f", st.max_a);

    // Profile and position changes only while settled.
    CHECK(motion_set_position(&ax, 0), "set_position refused while settled");
    motion_set_velocity(&ax, 10);
    motion_tick(&ax);
    CHECK(!motion_set_position(&ax, 0), "set_position allowed while moving");
    CHECK(!motion_set_profile(&ax, PROFILE_TRAP, 0), "set_profile allowed while moving");
}

int main(void) {
    test_encoder();
    test_phase_inc();
    test_microstep();
    test_ladder();
    test_frame();
    test_trig();
    test_motion();
    test_pvt();
    test_group();
    if (failures) {
        printf("%d check(s) failed\n", failures);
        return 1;
    }
    printf("all host tests passed\n");
    return 0;
}
