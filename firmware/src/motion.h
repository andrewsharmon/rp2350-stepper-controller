#pragma once

// Per-axis online trajectory generation, run at MOTION_TICK_HZ on core 1.
//
// Position is fixed point: MOTION_UNITS_PER_STEP units per full step, so the
// low 32 bits are exactly the microstep electrical phase (one electrical
// cycle = 4 full steps = 2^32). Speeds are in full steps/s.
//
// A generator (trapezoid, or a fixed-time cosine / quintic segment) produces
// the raw motion; the s-curve profiles then pass it through one or two
// moving-average filters, which limit jerk without raising peak speed or
// acceleration and without losing a single position unit.
//
// Pure logic (no SDK), so it runs in the host tests.

#include <stdbool.h>
#include <stdint.h>

#define MOTION_TICK_HZ        1000u
#define MOTION_UNITS_PER_STEP (1ll << 30)
#define MOTION_MAX_FILTER     100u   // ticks per moving-average stage (100 ms)
#define MOTION_PVT_QUEUE      32u    // streamed points buffered per axis

typedef enum {
    MODE_IDLE,      // holding position, v = 0
    MODE_POSITION,  // move to `target`, retargetable mid-move
    MODE_VELOCITY,  // ramp to `cmd_vel` and stay there
    MODE_JOG,       // like velocity, but decelerates to a stop if not
                    // refreshed within the jog timeout
    MODE_PVT,       // follow streamed position/velocity/time points
    MODE_GROUP,     // owned by a coordinated group (group.c drives gen_pos)
    MODE_CAM,       // cam follower: gen_pos is a function of a leader (control.c)
} motion_mode_t;

typedef enum {
    PROFILE_TRAP,     // trapezoid: fastest, step changes in acceleration
    PROFILE_SCURVE,   // trapezoid + one moving average: jerk = amax / Tj per
                      // acceleration change (twice that at a direct reversal)
    PROFILE_SMOOTH,   // trapezoid + two moving averages: continuous jerk
    PROFILE_COSINE,   // fixed-time cosine move (position mode only)
    PROFILE_QUINTIC,  // fixed-time minimum-jerk quintic (position mode only)
    PROFILE_COUNT,
} motion_profile_t;

// One streamed point: reach `pos` at speed `vel`, `ticks` after the
// previous point (cubic Hermite in between).
typedef struct {
    int64_t pos;
    float vel;
    uint32_t ticks;
} motion_pvt_point_t;

// Moving average over the last `len` per-tick position deltas, with the
// division remainder carried so the output total equals the input total.
typedef struct {
    int64_t ring[MOTION_MAX_FILTER];
    int64_t sum;
    int64_t carry;
    uint32_t head, len, nonzero;
} motion_filter_t;

typedef struct {
    // Output: what the motor follows.
    int64_t pos;          // units
    float vel;            // full steps/s
    motion_mode_t mode;
    bool settled;         // generator idle and filters flushed (hold current)

    // Raw generator state (before the filters). gen_last is the generator
    // position the output has already followed: a group moves gen_pos
    // before the axis ticks, so the per-tick delta is gen_pos - gen_last.
    int64_t gen_pos, gen_last;
    float gen_vel, gen_acc;

    // Command
    int64_t target;       // MODE_POSITION, units
    float cmd_vel;        // MODE_VELOCITY / MODE_JOG
    uint32_t jog_ticks_left;

    // Fixed-time segment (cosine / quintic): x relative to seg_p0, as a
    // polynomial in normalized time tau = t / seg_T (cosine: c[0] = distance).
    bool seg_active, seg_cosine;
    int64_t seg_p0;
    double seg_cd[6];     // exact, for the periodic re-sync
    float seg_c[6];       // float copy for the per-tick evaluation
    uint32_t seg_tick;    // ticks since the segment started
    float seg_T;          // duration, s
    int64_t seg_corr;     // per-tick drift correction (units)

    // Streamed PVT: queue and the segment being followed.
    motion_pvt_point_t pvt_q[MOTION_PVT_QUEUE];
    uint32_t pvt_head, pvt_count;
    int64_t pvt_p0;
    float pvt_c[3];       // x(tau) = c0 tau + c1 tau^2 + c2 tau^3, steps
    uint32_t pvt_tick, pvt_ticks;
    int64_t pvt_p1;
    float pvt_v1;
    uint32_t pvt_underruns;
    bool unfiltered;      // PVT (and its underrun ramp) bypass the filters

    // Group ownership (group.c).
    int8_t group;         // -1: not in a group
    bool ext_moving;      // a group or cam moved this axis this tick

    // Settings
    float vmax;           // full steps/s
    float amax;           // full steps/s^2
    motion_profile_t profile;
    uint32_t jerk_ticks;  // moving-average length (s-curve / smooth)
    motion_filter_t filt[2];
} motion_axis_t;

void motion_init(motion_axis_t *ax, float vmax, float amax);

// Commands. Each takes effect at the next tick and blends from the current
// state (no jumps).
void motion_move_to(motion_axis_t *ax, int64_t target);
void motion_set_velocity(motion_axis_t *ax, float vel);
void motion_jog(motion_axis_t *ax, float vel, uint32_t timeout_ms);
void motion_stop(motion_axis_t *ax);   // decelerate to rest, then IDLE

// Streamed PVT. Points queue up; motion_pvt_start begins following them
// (only from rest, so several axes can start on the same tick). Running
// out of points while moving decelerates at amax and counts an underrun.
bool motion_pvt_push(motion_axis_t *ax, int64_t pos, float vel, uint32_t ms);
bool motion_pvt_start(motion_axis_t *ax);

// Change profile / jerk time (only while settled; returns false otherwise).
bool motion_set_profile(motion_axis_t *ax, motion_profile_t profile, uint32_t jerk_ms);

// Redefine the current position (only while settled; returns false otherwise).
bool motion_set_position(motion_axis_t *ax, int64_t pos);

// Stop dead where the output is (e-stop: outputs are already off).
void motion_halt(motion_axis_t *ax);

// Advance one tick. Updates pos and vel.
void motion_tick(motion_axis_t *ax);

const char *motion_profile_name(motion_profile_t p);

static inline int64_t motion_steps_to_units(float steps) {
    return (int64_t)((double)steps * (double)MOTION_UNITS_PER_STEP);
}

static inline float motion_units_to_steps(int64_t units) {
    return (float)((double)units / (double)MOTION_UNITS_PER_STEP);
}
