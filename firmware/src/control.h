#pragma once

// Core 1 motion engine: runs the 1 kHz trajectory tick for every axis,
// interpolates each tick across its PWM periods, sets drive amplitude, keeps
// the DMA rings full and acts on the e-stop ladder. Core 0 talks to it only
// through the command queue, the snapshot and the few flags below.

#include <stdbool.h>
#include <stdint.h>
#include "cam.h"
#include "config.h"
#include "group.h"
#include "ladder.h"
#include "motion.h"

#ifndef NUM_MOTORS
#define NUM_MOTORS 10
#endif

#define CONTROL_ALL_AXES ((1u << NUM_MOTORS) - 1)
#define CAM_TABLES       4
#define VIRTUAL_LEADERS  2   // leader ids NUM_MOTORS.. are virtual leaders


// Group commands are the contiguous range CMD_GROUP_CREATE..CMD_GROUP_STOP;
// use control_is_group_cmd rather than comparing against one end.
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
    CMD_GROUP_CREATE,  // ms: group id, axes: members, f1: feed, f2: corner time ms
    CMD_GROUP_RELEASE, // ms: group id
    CMD_GROUP_LINE,    // ms: group id, n + vec: target per member, f1: feed
    CMD_GROUP_ARC,     // ms: group id, vec[0..1]: center, d1: angle (rad), f1: feed, f2: tolerance
    CMD_GROUP_HOLD,    // ms: group id, f1: 1 hold / 0 resume
    CMD_GROUP_STOP,    // ms: group id: decelerate along the path, drop the queue
    CMD_DRIVE,         // drive: amplitude curve and wiring flags
    CMD_CAM_LOAD,      // ms: table id: copy control_cam_staging[id] in (unused tables only)
    CMD_CAM_ENGAGE,    // axes: followers, ms: table, n: leader (axis 0-9 or virtual
                       //   NUM_MOTORS + k), pos: follower offset (units), f1: blend ms
    CMD_VLEADER,       // ms: virtual leader, n: op (VL_*), f1/f2/pos: arguments
} control_cmd_type_t;

// Virtual leader operations (CMD_VLEADER n).
enum { VL_VELOCITY, VL_MOVE, VL_STOP, VL_LIMITS, VL_ZERO };

static inline bool control_is_group_cmd(int type) {
    return type >= CMD_GROUP_CREATE && type <= CMD_GROUP_STOP;
}

typedef struct {
    uint8_t type;
    uint16_t axes;   // bit mask, bit 0 = axis 1
    uint32_t seq;    // set by control_post
    float f1, f2;
    int64_t pos;
    uint32_t ms;
    uint8_t n;                    // values in vec
    double d1;
    int64_t vec[GROUP_MAX_AXES];
    config_drive_t drive;
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
    uint32_t settled_mask;         // generator idle and filters flushed
    uint16_t pvt_underrun[NUM_MOTORS];
    config_drive_t drive[NUM_MOTORS];
    uint32_t last_seq;             // sequence number of the last applied command
    int8_t cam_table[NUM_MOTORS];  // table followed, -1: not a follower
    uint8_t cam_leader[NUM_MOTORS];
    uint8_t cams_loaded;           // bit mask of tables loaded on core 1
    struct {
        int64_t pos;
        float vel, vmax, amax;
        uint8_t mode;
    } vlead[VIRTUAL_LEADERS];
    struct {
        bool active, running, hold, arc;
        uint16_t members;          // axis mask
        uint8_t queued;
        float v;                   // path speed, full steps/s
        uint32_t segments_done;
    } group[GROUP_COUNT];
    uint32_t rejected;             // commands refused (z / prof need the axis at rest)
} control_snapshot_t;

// Shared flags. Core 1 writes the read-only ones.
extern volatile bool control_outputs_on;    // false while a stop is latched
extern volatile bool control_clear_request; // core 0 asks to clear a stop
extern volatile bool control_clear_refused; // core 1: line still reads a stop
extern volatile bool control_energized;     // false: drive amplitude 0
extern volatile float control_manual_amp;   // < 0: automatic
extern volatile bool control_stall_test;    // watchdog test: stall core 1
extern volatile bool control_flash_busy;    // core 0: core 1 paused for a flash write
extern volatile uint32_t control_heartbeat;
extern volatile uint32_t control_busy_us;   // reset by the reader
extern volatile uint32_t control_min_queued;
extern ladder_t control_ladder;             // owned by core 1, read by core 0
// Core 0 builds cam tables here, then posts CMD_CAM_LOAD; it must not touch
// a staging table again until that command has been applied.
extern cam_table_t control_cam_staging[CAM_TABLES];

// Call on core 0 after the motors are initialized, before launching core 1.
// Limits, profiles and drive settings come from the configuration.
void control_init(const config_t *cfg);
void control_core1_main(void);

// Queue a command for core 1, stamping it with the next sequence number
// (core 0 only). Returns that number, or 0 if the queue is full.
uint32_t control_post(const control_cmd_t *cmd);

// Consistent copy of the latest tick's state.
void control_snapshot(control_snapshot_t *snap);

// Core 0: build a cam table and queue it for core 1. Returns NULL, or why
// it can't (bad points, the table is in use, a previous load still pending).
const char *control_cam_load(uint32_t table, const float *x, const float *y, uint32_t n, bool cyclic);

// Core 0: why engaging followers `axes` on `table` behind `leader` would be
// refused, or NULL. Call it before posting CMD_CAM_ENGAGE: this is where
// the cam is checked against the axes' limits (too slow for core 1's tick);
// core 1 only re-checks the cheap structural conditions.
const char *control_cam_engage_check(uint32_t axes, uint32_t table, uint32_t leader);
