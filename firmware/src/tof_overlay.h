#pragma once

// ToF map overlay: the ToF sensor (tof.h) drives the row of linear
// actuators through the mapping in tof_map.h. Each whole measurement gives
// every mapped axis a distance; it is smoothed, turned into a position and
// sent as a position move, which the axis's own limits and profile
// shape (s-curve by default; the fixed-time profiles restart a segment on
// every retarget, so they suit this less).
//
// Starting switches the sensor on if it is off, loads the overlay's preset
// with a low iteration count (the band is close, so a short exposure
// does) and starts measuring with no stream, unless a host or the console
// already is. It stops on tof_overlay_stop, an e-stop, or the sensor
// stopping or failing; anything else that commands the mapped axes is
// overridden by the next measurement.
//
// Core 0 only.

#include <stdbool.h>
#include <stdint.h>
#include "tof_map.h"

typedef struct {
    tof_map_cfg_t map;
    uint8_t preset;            // TMF8829_PRESET_*
    uint16_t kilo_iterations;  // per measurement
    uint16_t period_ms;
    uint16_t smooth_ms;        // time constant of the distance smoothing (0: none)
    float deadband;            // full steps: smaller target changes aren't sent
    uint8_t min_conf;          // first-peak confidence below this: no target
} tof_overlay_cfg_t;

// Lay out axes 1..n (n <= NUM_MOTORS) across the band in the board's row
// order (BOARD_AXIS_ROW in board_pins.h, else 1..n); the default is every
// axis.
void tof_overlay_board_order(uint8_t n);

// Live settings: changes apply from the next measurement. After changing
// preset, kilo_iterations or period_ms while running, call
// tof_overlay_apply_sensor.
tof_overlay_cfg_t *tof_overlay_cfg(void);
const char *tof_overlay_apply_sensor(void);

// NULL, or why it can't start (no Qwiic port, outputs stopped, an axis in
// a group or following a cam). Asynchronous: the sensor may take a moment.
const char *tof_overlay_start(void);
// Stop (the mapped axes decelerate). Prints `why` if it was running.
void tof_overlay_stop(const char *why);
bool tof_overlay_active(void);    // starting or running
bool tof_overlay_running(void);
const char *tof_overlay_state(void);   // "off", "starting sensor", "running"
void tof_overlay_print_status(void);

void tof_overlay_poll(uint32_t now_us);
