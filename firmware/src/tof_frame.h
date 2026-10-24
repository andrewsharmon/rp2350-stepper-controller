#pragma once

// TMF8829 result frames: pixel layout, clock correction and a per-frame
// summary. Pure logic (no SDK), so it runs in the host tests.
//
// A result frame is a 16-byte header, then one record per zone, then a
// 12-byte footer. header[1] (layout) says what a record holds: bits 0-2 the
// number of peaks, bit 3 a signal strength per peak, bit 4 a noise and bit
// 5 a crosstalk value per zone, bit 6 the sub-frame index. A record is
// [noise u16][xtalk u16] then per peak: distance u16, confidence u8,
// [signal u16]. Distances are in 0.25 mm unless footer[9] bit 0 says mm.
//
// 8x8 and 16x16 come in one frame. 32x32 and 48x32 come in two: rows
// 0, 2, 4... in sub-frame 0 and rows 1, 3, 5... in sub-frame 1.

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define TOF_LAYOUT_PEAKS    0x07
#define TOF_LAYOUT_SIGNAL   0x08
#define TOF_LAYOUT_NOISE    0x10
#define TOF_LAYOUT_XTALK    0x20
#define TOF_LAYOUT_SUBFRAME 0x40
#define TOF_FOOTER_MM       0x01   // footer[9]: distances in mm

// Bytes per zone record (0 if the layout carries nothing).
uint32_t tof_record_size(uint8_t layout);

// Zone grid of a focal plane mode: columns, rows in one frame, and the
// number of frames per measurement.
void tof_grid(uint8_t fp_mode, uint32_t *cols, uint32_t *rows_per_frame, uint32_t *frames);

// Clock correction: the chip's oscillator runs a little fast or slow, so
// its distances scale with it. Pairs of (host time, chip systick) taken as
// each result frame is read give the ratio; as in the reference driver it
// spans the last TOF_CLOCK_PAIRS frames and stays within 0.5-1.5.
#define TOF_CLOCK_PAIRS 4
typedef struct {
    uint32_t host_us[TOF_CLOCK_PAIRS];
    uint32_t ticks[TOF_CLOCK_PAIRS];
    uint32_t n, idx;
    float ratio;
} tof_clock_t;

void tof_clock_reset(tof_clock_t *c);
void tof_clock_add(tof_clock_t *c, uint32_t host_us, uint32_t chip_ticks, uint32_t ticks_per_ms);

// Scale every peak distance in `n` bytes of whole zone records.
void tof_correct_distances(uint8_t *records, size_t n, uint8_t layout, float ratio);

// Running summary of the first peak over the records of one frame.
typedef struct {
    uint32_t zones, valid;
    uint32_t min_raw, max_raw;
    uint64_t sum_raw;
} tof_summary_t;

void tof_summary_reset(tof_summary_t *s);
void tof_summary_add(tof_summary_t *s, const uint8_t *records, size_t n, uint8_t layout);

// First peak of each zone into `grid` (row by row, cols x rows, every
// sub-frame's rows in place), 0 where its confidence is under `min_conf`
// (at least 1). records[0] is zone `first` of the frame; `n` bytes of
// whole records. Zones past the grid are ignored.
void tof_zones_add(uint16_t *grid, uint32_t cols, uint32_t rows, uint32_t frames, uint32_t sub, uint32_t first,
                   const uint8_t *records, size_t n, uint8_t layout, uint8_t min_conf);
