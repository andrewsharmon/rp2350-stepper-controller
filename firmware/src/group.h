#pragma once

// Coordinated groups: a group owns a set of axes and moves them together
// along a queue of straight segments in joint space, so every axis starts
// and finishes each segment at the same moment.
//
// Planning: path speed and acceleration on each segment are limited so no
// member axis exceeds its own vmax / amax. Junction speeds are limited so
// no axis's speed jumps by more than amax * corner_s at a corner. Entry
// speeds are planned over the whole queue (backward and forward passes), so
// the group can always stop at the end of what it has been given.
//
// Execution runs the exact discrete stopping law along the path (the same
// as the single-axis trapezoid, with a nonzero exit speed), so segments end
// exactly on their targets. The group writes each member's generator
// position; the axes' own s-curve filters then smooth all of them alike.
//
// Pure logic (no SDK), so it runs in the host tests.

#include <stdbool.h>
#include <stdint.h>
#include "motion.h"

#define GROUP_MAX_AXES  10
#define GROUP_QUEUE     32
#define GROUP_COUNT     4

typedef struct {
    int64_t end[GROUP_MAX_AXES];  // absolute target per member (units)
    float u[GROUP_MAX_AXES];      // unit direction (joint space, steps)
    float len;                    // full steps along the path
    float vmax, amax;             // path limits on this segment
    float v_junction;             // max speed entering this segment
    float v_entry;                // planned entry speed
} group_seg_t;

typedef struct {
    bool active;
    uint8_t n;                         // member count
    uint8_t axis[GROUP_MAX_AXES];      // member axis indices
    float feed;                        // default path speed limit, steps/s
    float corner_s;                    // corner time: allowed per-axis speed jump = amax * corner_s

    group_seg_t q[GROUP_QUEUE];
    uint32_t head, count;              // q[head] is executing when running
    int64_t start[GROUP_MAX_AXES];     // start of the executing segment
    int64_t tail[GROUP_MAX_AXES];      // end of the last queued segment
    float s;                           // distance along the executing segment
    float v;                           // path speed
    bool running;
    bool hold;                         // decelerate along the path and wait
    bool flush_on_stop;                // stop: drop the queue once at rest
    uint32_t segments_done;

    // Arc being fed into the queue as chords (members 0 and 1).
    bool arc_active;
    int64_t arc_c[2];                  // center (units)
    float arc_r0[2];                   // start point relative to center (steps)
    float arc_step;                    // radians per chord
    uint32_t arc_k, arc_n;             // next chord, chord count
    double arc_angle;                  // total angle, for the exact last point
    int64_t arc_p0[2];                 // exact start point
    float arc_feed;
} group_t;

// Create a group over `axes` (all must be settled, not in another group).
// Positions start from the axes' current output positions.
bool group_create(group_t *g, int8_t id, motion_axis_t *axes, const uint8_t *members,
                  uint32_t n, float feed);

// Release the group (only at rest). The axes go back to IDLE.
bool group_release(group_t *g, motion_axis_t *axes);

// Queue a straight move to `target` (one value per member, units). feed <= 0
// uses the group default. Returns false if the queue is full.
bool group_line(group_t *g, const motion_axis_t *axes, const int64_t *target, float feed);

// Queue an arc on members 0 and 1 around `center` (units) through `angle`
// radians (positive: from member 0 toward member 1), as chords no further
// than `tol` steps from the true circle. Other members hold their queued
// position. Chords are fed in as the queue drains, so arcs of any length
// fit. Returns false if another arc is still being fed.
bool group_arc(group_t *g, const int64_t *center, double angle, float feed, float tol);

void group_hold(group_t *g, bool hold);

// E-stop: drop all motion; keep the membership. Axes are already halted.
void group_halt(group_t *g, const motion_axis_t *axes);
void group_stop(group_t *g);   // hold, then drop the queue at rest

// Advance one tick: sets each member's generator position and speed.
void group_tick(group_t *g, motion_axis_t *axes);

bool group_idle(const group_t *g);
