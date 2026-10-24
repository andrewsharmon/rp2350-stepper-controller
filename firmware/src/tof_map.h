#pragma once

// ToF map: distances in a band across the middle of the sensor's field of
// view become positions for a row of linear actuators. The band is split
// left to right into one slice per axis (`order` says which axis sits
// where in the row); an axis follows the nearest return in its slice, so
// a hand or object between near_mm and far_mm pushes the actuators under
// it, clamped at both ends. A slice with no
// return reads as far.
//
// Pure logic (no SDK), so it runs in the host tests.

#include <stdbool.h>
#include <stdint.h>

#define TOF_MAP_MAX_AXES 16

typedef struct {
    float near_mm, far_mm;        // the distance range mapped
    float steps_near, steps_far;  // axis positions (full steps) at those ends
    uint8_t axes;                 // slices across the band
    uint8_t order[TOF_MAP_MAX_AXES];  // axis (0-based) of each slice, left to right
    float band_height;            // band size as fractions of the field of view,
    float band_width;             //   centred on its middle
    bool mirror;                  // the first slice on the right of the sensor's grid
} tof_map_cfg_t;

// Defaults with axes 1..axes in order.
void tof_map_defaults(tof_map_cfg_t *c, uint8_t axes);

// Band in zones: columns c0..c1, rows r0..r1 (inclusive).
void tof_map_band(const tof_map_cfg_t *c, uint32_t cols, uint32_t rows, uint32_t *c0, uint32_t *c1,
                  uint32_t *r0, uint32_t *r1);

// Per-slice distance (mm, clamped to near..far) from one measurement:
// mm[k] is for the axis order[k].
// `raw` holds the first peak of each zone, row by row, in units of
// `unit_mm`; 0 means no target.
void tof_map_distances(const tof_map_cfg_t *c, const uint16_t *raw, uint32_t cols, uint32_t rows,
                       float unit_mm, float *mm);

// Axis position (full steps) for a distance.
float tof_map_steps(const tof_map_cfg_t *c, float mm);
