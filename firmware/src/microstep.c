#include "microstep.h"

#include <math.h>

#define LUT_SIZE (1u << MICROSTEP_LUT_BITS)

uint32_t microstep_phase_inc(float full_steps_per_sec, uint32_t pwm_hz) {
    // 4 full steps per electrical cycle.
    double cycles_per_period = (double)full_steps_per_sec / 4.0 / (double)pwm_hz;
    return (uint32_t)(int64_t)llround(cycles_per_period * 4294967296.0);
}

static int32_t diffuse(float duty, int32_t min_duty, float *carry) {
    float want = duty + *carry;
    int32_t out = fabsf(want) >= (float)min_duty ? (int32_t)lrintf(want) : 0;
    *carry = want - (float)out;
    return out;
}

void microstep_duties(microstep_t *ms, int32_t max_duty, int32_t min_duty,
                      int32_t *duty_a, int32_t *duty_b) {
    uint32_t idx = ms->phase >> (32 - MICROSTEP_LUT_BITS);
    int32_t sin_q15 = microstep_sine_lut[idx];
    int32_t cos_q15 = microstep_sine_lut[(idx + LUT_SIZE / 4) & (LUT_SIZE - 1)];
    float scale = ms->amplitude * (float)max_duty / 32767.0f;

    *duty_a = diffuse((float)cos_q15 * scale, min_duty, &ms->carry_a);
    *duty_b = diffuse((float)sin_q15 * scale, min_duty, &ms->carry_b);
}
