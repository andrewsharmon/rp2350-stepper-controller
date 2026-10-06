#pragma once

#include <stdint.h>
#include "hardware/pio.h"

#define HBRIDGE_MAX_MOTORS 10
#define HBRIDGE_PWM_HZ     20000u
// AT8833 input deglitch is ~450 ns; shorter pulses are swallowed.
#define HBRIDGE_MIN_PULSE_NS 500u

// Per-motor DMA ring: 2 words per PWM period, 64 periods = 3.2 ms at 20 kHz.
#define HBRIDGE_RING_BITS    9
#define HBRIDGE_RING_WORDS   ((1u << HBRIDGE_RING_BITS) / 4)
#define HBRIDGE_RING_PERIODS (HBRIDGE_RING_WORDS / 2)

typedef struct {
    PIO pio;
    uint sm;
    uint pin_base;
    uint dma_ch;
    uint32_t *ring;
    uint32_t wr;  // next word index to write (mod HBRIDGE_RING_WORDS)
} hbridge_t;

// Configure motor `index` (0..HBRIDGE_MAX_MOTORS-1) on GPIO 4*index..4*index+3
// and its DMA ring, pre-filled with brake. Outputs stay idle until
// hbridge_start(). Motors 0-3 use PIO0, 4-7 PIO1, 8-9 PIO2 (GPIO base 16).
void hbridge_init(hbridge_t *hb, uint index);

// Start the state machines of all `count` motors, in sync per PIO block.
void hbridge_start(hbridge_t *hbs, uint count);

// Drive span per coil in counts: duties passed to hbridge_write_period are
// clamped to +/- this value.
int32_t hbridge_max_duty(void);

// Shortest duty (counts) that survives the driver's input deglitch.
int32_t hbridge_min_duty(void);

// PWM periods that can be written without overtaking the DMA read position.
uint32_t hbridge_free_periods(const hbridge_t *hb);

// Append one PWM period to the ring. Signed duties per coil: positive drives
// xIN1 high, negative drives xIN2 high; the off-time is brake (slow decay).
// Call only when hbridge_free_periods() > 0.
void hbridge_write_period(hbridge_t *hb, int32_t duty_a, int32_t duty_b);

// Stop DMA and PIO for this motor and hand the pins to SIO, driven low
// (coast). Safe to call from either core; used when the producer stalls,
// since the ring would otherwise replay stale periods.
void hbridge_safe_off(hbridge_t *hb);
