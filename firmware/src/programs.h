#pragma once

// Built-in motion programs: patterns for a row of linear actuators that the
// buttons cycle through with no computer attached. Each is a smooth,
// periodic position function per axis, in full steps about position 0 (the
// power-up position, so power up with the actuators mid-stroke, or zero them
// there with `z`). The player streams a program to the PVT queues like a
// looping show.
//
// A program starts from rest at its first pose: its clock speeds up from 0
// to full rate over PROGRAM_WARMUP_S. That is a time warp, not an amplitude
// ramp, so the speed never exceeds the steady pattern's.
//
// Pure logic (no SDK), so it runs in the host tests.

#include <stdint.h>
#include "show.h"

#define PROGRAM_COUNT     12
#define PROGRAM_POINT_MS  50       // PVT point spacing
#define PROGRAM_WARMUP_S  4.0
#define PROGRAM_AMP       1000.0   // full steps each way from the centre

typedef struct {
    const char *name;
    double period_s;              // the pattern repeats with this period
    // Bounds on |speed| (full steps/s) and |acceleration| (full steps/s^2)
    // with ten axes, warm-up included (the host tests check them).
    float vpeak, apeak;
    double (*pos)(int axis, int n_axes, double t);  // steady pattern, 0 <= t < period
} program_t;

extern const program_t programs[PROGRAM_COUNT];

// Position of `axis` (full steps) t seconds after the start, warm-up included.
double program_pos(const program_t *p, int axis, int n_axes, double t);

// Point i of an axis's PVT stream (one every PROGRAM_POINT_MS from the start).
show_point_t program_point(const program_t *p, int axis, int n_axes, uint64_t i);

// NULL if every axis's limits cover the program's peaks; otherwise fills
// `msg` and returns it.
const char *program_check_limits(const program_t *p, int n_axes, const float *vmax,
                                 const float *amax, char *msg, uint32_t msg_len);
