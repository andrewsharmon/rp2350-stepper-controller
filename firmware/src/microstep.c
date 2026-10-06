#include "microstep.h"

#include <math.h>
#if __has_include("pico.h")
#include "pico.h"
#define HOT_FUNC(f) __time_critical_func(f)  // runs every PWM period: keep in RAM
#else
#define HOT_FUNC(f) f
#endif

#define LUT_SIZE (1u << MICROSTEP_LUT_BITS)

// Round to nearest without a libm call (one VCVT on the M33).
static inline int32_t round_to_int(float x) {
    return (int32_t)(x >= 0.0f ? x + 0.5f : x - 0.5f);
}

uint32_t HOT_FUNC(microstep_phase_inc)(float full_steps_per_sec, uint32_t pwm_hz) {
    // 4 full steps per electrical cycle. Single precision: the M33 FPU has no
    // double support and this runs every PWM period. Fits int32 below
    // ~40000 full steps/s at 20 kHz.
    float phase_per_period = full_steps_per_sec * (4294967296.0f / 4.0f) / (float)pwm_hz;
    return (uint32_t)round_to_int(phase_per_period);
}

static int32_t diffuse(float duty, int32_t min_duty, float *carry) {
    float want = duty + *carry;
    int32_t out = (want >= 0.0f ? want : -want) >= (float)min_duty ? round_to_int(want) : 0;
    *carry = want - (float)out;
    return out;
}

void HOT_FUNC(microstep_duties)(microstep_t *ms, int32_t max_duty, int32_t min_duty,
                      int32_t *duty_a, int32_t *duty_b) {
    uint32_t idx = ms->phase >> (32 - MICROSTEP_LUT_BITS);
    int32_t sin_q15 = microstep_sine_lut[idx];
    int32_t cos_q15 = microstep_sine_lut[(idx + LUT_SIZE / 4) & (LUT_SIZE - 1)];
    float scale = ms->amplitude * (float)max_duty / 32767.0f;

    *duty_a = diffuse((float)cos_q15 * scale, min_duty, &ms->carry_a);
    *duty_b = diffuse((float)sin_q15 * scale, min_duty, &ms->carry_b);
}
