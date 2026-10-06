#pragma once

// Per-axis online trajectory generation, run at MOTION_TICK_HZ on core 1.
//
// Position is fixed point: MOTION_UNITS_PER_STEP units per full step, so the
// low 32 bits are exactly the microstep electrical phase (one electrical
// cycle = 4 full steps = 2^32). Speeds are in full steps/s.
//
// Pure logic (no SDK), so it runs in the host tests.

#include <stdbool.h>
#include <stdint.h>

#define MOTION_TICK_HZ        1000u
#define MOTION_UNITS_PER_STEP (1ll << 30)

typedef enum {
    MODE_IDLE,      // holding position, v = 0
    MODE_POSITION,  // trapezoid to `target`, retargetable mid-move
    MODE_VELOCITY,  // ramp to `cmd_vel` and stay there
    MODE_JOG,       // like velocity, but decelerates to a stop if not
                    // refreshed within the jog timeout
} motion_mode_t;

typedef struct {
    // State
    int64_t pos;          // units
    float vel;            // full steps/s
    motion_mode_t mode;
    bool settled;         // in IDLE with v == 0 (for hold current)

    // Command
    int64_t target;       // MODE_POSITION, units
    float cmd_vel;        // MODE_VELOCITY / MODE_JOG
    uint32_t jog_ticks_left;

    // Limits
    float vmax;           // full steps/s
    float amax;           // full steps/s^2
} motion_axis_t;

void motion_init(motion_axis_t *ax, float vmax, float amax);

// Commands. Each takes effect at the next tick and blends from the current
// position and velocity (no jumps).
void motion_move_to(motion_axis_t *ax, int64_t target);
void motion_set_velocity(motion_axis_t *ax, float vel);
void motion_jog(motion_axis_t *ax, float vel, uint32_t timeout_ms);
void motion_stop(motion_axis_t *ax);   // decelerate to rest, then IDLE

// Redefine the current position (only while settled; returns false otherwise).
bool motion_set_position(motion_axis_t *ax, int64_t pos);

// Advance one tick. Updates pos and vel.
void motion_tick(motion_axis_t *ax);

static inline int64_t motion_steps_to_units(float steps) {
    return (int64_t)((double)steps * (double)MOTION_UNITS_PER_STEP);
}

static inline float motion_units_to_steps(int64_t units) {
    return (float)((double)units / (double)MOTION_UNITS_PER_STEP);
}
