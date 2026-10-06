#pragma once

#include <stdint.h>

// One electrical cycle (4 full steps) is the full uint32_t phase range.
#define MICROSTEP_LUT_BITS 10  // 1024 entries/cycle = 1/256 of a full step

typedef struct {
    uint32_t phase;
    float amplitude;  // 0..1 fraction of the full PWM drive span
    float carry_a;  // error-diffusion residue per coil, in counts
    float carry_b;
} microstep_t;

// One cycle of Q15 sine, generated offline (tools/gen_sine_lut.py).
extern const int16_t microstep_sine_lut[1 << MICROSTEP_LUT_BITS];

// Phase increment per PWM period for a given full-step rate.
uint32_t microstep_phase_inc(float full_steps_per_sec, uint32_t pwm_hz);

// Signed coil duties for the current phase. The exact (fractional) duty is
// error-diffused: rounding residue and duties shorter than min_duty carry
// into later periods, so the average matches the request.
void microstep_duties(microstep_t *ms, int32_t max_duty, int32_t min_duty,
                      int32_t *duty_a, int32_t *duty_b);
