// Host-side checks for the PWM period encoder and microstep math.
//
//   cc -std=c11 -O1 -Wall -Wextra -I../src test_host.c ../src/hbridge_encode.c ../src/microstep.c ../src/sine_lut.c ../src/ladder.c ../src/motion.c -lm -o test_host && ./test_host
//
// The PIO program is simulated per segment: each 16-bit half-word holds
// pattern bits [3:0] and length [15:4], and lasts length + 3 SM clocks.

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "hbridge_encode.h"
#include "microstep.h"
#include "ladder.h"
#include "motion.h"
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
    test_trig();
    test_motion();
    if (failures) {
        printf("%d check(s) failed\n", failures);
        return 1;
    }
    printf("all host tests passed\n");
    return 0;
}
