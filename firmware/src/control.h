#pragma once

// Core 1 motion engine: runs the 1 kHz trajectory tick for every axis,
// interpolates each tick across its PWM periods, sets drive amplitude, keeps
// the DMA rings full and acts on the e-stop ladder. Core 0 talks to it only
// through the command queue, the snapshot and the few flags below.

#include <stdbool.h>
#include <stdint.h>
#include "ladder.h"
#include "motion.h"

#ifndef NUM_MOTORS
#define NUM_MOTORS 10
#endif

#define CONTROL_ALL_AXES ((1u << NUM_MOTORS) - 1)

// Defaults for new axes (bench 8 mm stepper at 5 V; holds ~1900 at 60%).
#define CONTROL_DEFAULT_VMAX 1500.0f   // full steps/s
#define CONTROL_DEFAULT_AMAX 2000.0f   // full steps/s^2

typedef enum {
    CMD_MOVE,        // pos: absolute target (units)
    CMD_MOVE_REL,    // pos: offset from the current target or position
    CMD_VELOCITY,    // f1: full steps/s
    CMD_JOG,         // f1: full steps/s, ms: watchdog timeout
    CMD_STOP,        // decelerate to rest
    CMD_SET_POS,     // pos: redefine position (axis must be settled)
    CMD_LIMITS,      // f1: vmax, f2: amax (<= 0 leaves that limit unchanged)
    CMD_PROFILE,     // ms: profile (motion_profile_t), f1: jerk time in ms
    CMD_PVT_POINT,   // pos: position (units), f1: speed, ms: time since previous point
    CMD_PVT_START,   // start following queued points (axes must be at rest)
} control_cmd_type_t;

typedef struct {
    uint8_t type;
    uint16_t axes;   // bit mask, bit 0 = axis 1
    float f1, f2;
    int64_t pos;
    uint32_t ms;
} control_cmd_t;

typedef struct {
    uint32_t tick;                 // 1 kHz tick count
    int64_t pos[NUM_MOTORS];       // units
    float vel[NUM_MOTORS];         // full steps/s
    uint8_t mode[NUM_MOTORS];      // motion_mode_t
    float amp[NUM_MOTORS];
    float vmax[NUM_MOTORS], amax[NUM_MOTORS];
    uint8_t profile[NUM_MOTORS];   // motion_profile_t
    uint16_t jerk_ms[NUM_MOTORS];
    uint8_t pvt_depth[NUM_MOTORS];  // queued PVT points
    uint32_t pvt_underruns;         // all axes
    uint32_t pvt_dropped;           // points refused: queue full
    uint32_t holding_mask;
    uint32_t rejected;             // commands refused (z / prof need the axis at rest)
} control_snapshot_t;

// Shared flags. Core 1 writes the read-only ones.
extern volatile bool control_outputs_on;    // false while a stop is latched
extern volatile bool control_clear_request; // core 0 asks to clear a stop
extern volatile bool control_clear_refused; // core 1: line still reads a stop
extern volatile bool control_energized;     // false: drive amplitude 0
extern volatile float control_manual_amp;   // < 0: automatic
extern volatile bool control_stall_test;    // watchdog test: stall core 1
extern volatile uint32_t control_heartbeat;
extern volatile uint32_t control_busy_us;   // reset by the reader
extern volatile uint32_t control_min_queued;
extern ladder_t control_ladder;             // owned by core 1, read by core 0

// Call on core 0 after the motors are initialized, before launching core 1.
void control_init(void);
void control_core1_main(void);

// Queue a command for core 1. Returns false if the queue is full.
bool control_post(const control_cmd_t *cmd);

// Consistent copy of the latest tick's state.
void control_snapshot(control_snapshot_t *snap);
