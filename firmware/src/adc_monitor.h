#pragma once

// Free-running ADC round robin over the monitor inputs, streamed by DMA so
// the latest sample of each input is always available without interrupts.

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    MON_LADDER,
    MON_VMOT,
    MON_BOARD_ID,
    MON_CC1,
    MON_CC2,
    MON_COUNT,
} mon_input_t;

void adc_monitor_init(void);

// True if this board has the input.
bool adc_monitor_present(mon_input_t in);

// Latest reading at the ADC pin, in millivolts (3.3 V reference).
uint32_t adc_monitor_pin_mv(mon_input_t in);

// Motor supply in millivolts, scaled by the board's sense divider.
uint32_t adc_monitor_vmot_mv(void);

// Restart the round robin if the ADC FIFO overflowed, which would leave the
// samples in the wrong slots. Returns true if it had to. Call periodically.
bool adc_monitor_check(void);
