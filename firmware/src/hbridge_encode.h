#pragma once

#include <stdint.h>

// SM clocks spent per segment beyond its length field (out, out, jmp).
#define HBRIDGE_SEGMENT_OVERHEAD 3

// Encode one PWM period for hbridge_pwm.pio as two FIFO words.
// `span` is the drive span in counts (period clocks minus 4 segment
// overheads); duties are clamped to +/- span. Positive duty drives xIN1,
// negative xIN2; the remainder of the period is brake (both inputs high).
void hbridge_encode_period(int32_t span, int32_t duty_a, int32_t duty_b, uint32_t *w0, uint32_t *w1);
