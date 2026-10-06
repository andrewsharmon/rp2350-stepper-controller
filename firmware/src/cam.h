#pragma once

// Electronic cam tables: follower position as a function of leader
// position, y = f(x), so a follower retraces exactly whichever way, and at
// whatever speed, the leader moves.
//
// A table is points (x, y) in full steps with increasing x, joined by cubic
// Hermite segments with Catmull-Rom slopes (C1: follower speed is continuous
// whenever the leader's is).
//   cyclic:  f repeats with period X = x_last - x_first; the net rise
//            R = y_last - y_first accumulates per cycle (R = 0 for a closed
//            cam; R != 0 e.g. to advance a follower each revolution).
//   one-shot: outside [x_first, x_last] f holds its end value (zero slope).
// Leader and follower positions are int64 units (2^30 per full step); cycles
// are counted in exact integer arithmetic, so millions of cycles in either
// direction don't drift.
//
// Pure logic (no SDK), so it runs in the host tests.

#include <stdbool.h>
#include <stdint.h>

#define CAM_MAX_POINTS 128

typedef struct {
    uint16_t n;
    bool cyclic;
    float x[CAM_MAX_POINTS], y[CAM_MAX_POINTS], m[CAM_MAX_POINTS];  // m: slope dy/dx
    int64_t x0_units, period_units, rise_units;
} cam_table_t;

// Compute slopes and cycle constants. Returns NULL or why the table is
// unusable (too few points, x not increasing, ...).
const char *cam_build(cam_table_t *t, const float *x, const float *y, uint32_t n, bool cyclic);

// Follower position (units) for a leader position (units), plus dy/dx and
// d2y/dx2 there (for speed and acceleration: v_f = y' v_l,
// a_f = y'' v_l^2 + y' a_l).
int64_t cam_eval(const cam_table_t *t, int64_t leader, float *dydx, float *d2ydx2);

// Refuse tables the follower can't keep up with when the leader runs at up
// to `lead_vmax` / `lead_amax`. Returns NULL, or fills `msg` and returns it.
const char *cam_check(const cam_table_t *t, float lead_vmax, float lead_amax,
                      float vmax, float amax, char *msg, uint32_t msg_len);
