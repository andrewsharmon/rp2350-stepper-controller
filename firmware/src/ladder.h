#pragma once

// Decoder for the analog button / e-stop / fault ladder (hardware/README.md):
//
//   3V3 --10k--+-- ADC
//              |-- E-stop (NC) --20k-- GND   open or cut  -> 3.3 V  STOP
//              |-- Btn1 ---------10k-- GND   idle         -> 2.2 V
//              |-- Btn2 ---------3.3k- GND   Btn1         -> 1.3 V
//              '-- nFAULT (open drain)       Btn2         -> 0.7 V
//                                            nFAULT       -> 0 V    STOP
//
// A held button hides an open e-stop: e-stop open + Btn1 reads 1.65 V and
// e-stop open + Btn2 reads 0.82 V, both inside the button bands. Two guards:
//   - 1.49-1.76 V (above Btn1's 1.32 V by more than resistor and ADC error)
//     held for LADDER_MASKED_US latches an e-stop. The delay rides over the
//     line passing through that band when Btn1 is released.
//   - Btn2's masked level is only 90 mV from Btn2 alone, too close to split,
//     so any button held longer than LADDER_BUTTON_MAX_US latches an e-stop.
//     This also catches a stuck button.
//
// Pure logic (no SDK), so it runs in the host tests.

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    LADDER_IDLE,
    LADDER_BTN1,
    LADDER_BTN2,
    LADDER_ESTOP,  // e-stop open or cable cut
    LADDER_FAULT,  // driver nFAULT pulled the line low
} ladder_level_t;

// Stop conditions must be seen this many consecutive evaluations (spaced at
// least one ADC round apart) before tripping: rejects single-sample noise
// while keeping stop latency well under 100 us.
#define LADDER_STOP_SAMPLES 3
// Buttons must be stable this long to register.
#define LADDER_BUTTON_US 20000u
// E-stop open while Btn1 is held: how long the masked level must persist.
#define LADDER_MASKED_US 5000u
// A button held longer than this is treated as an e-stop.
#define LADDER_BUTTON_MAX_US 3000000u

typedef struct {
    ladder_level_t raw_prev;
    uint32_t raw_count;          // consecutive evaluations at raw_prev
    uint32_t raw_since_us;       // when raw_prev was first seen
    bool masked;                 // last sample was in the e-stop + Btn1 band
    uint32_t masked_since_us;
    bool held_stop;              // a masked or over-long button level is tripping
    ladder_level_t stable;       // debounced level
    bool stop_latched;           // set on a stop level; cleared by ladder_clear_stop
    ladder_level_t stop_cause;
    uint32_t btn1_presses, btn2_presses;
} ladder_t;

// Thresholds sit midway between the nominal levels (in millivolts).
ladder_level_t ladder_classify(uint32_t mv);

void ladder_reset(ladder_t *l, uint32_t now_us);

// Feed one sample. Returns true when a stop (e-stop or fault) newly latched.
bool ladder_update(ladder_t *l, uint32_t mv, uint32_t now_us);

// Clear a latched stop. Refused (returns false) while the line still reads a
// stop level, including a masked or over-long button.
bool ladder_clear_stop(ladder_t *l);

const char *ladder_level_name(ladder_level_t level);
