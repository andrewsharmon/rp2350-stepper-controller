#pragma once

// Status LED (WS2812) on PIO2 SM2. The LED data line also continues down the
// stack, so only the first pixel is driven here.

#include <stdint.h>

void led_init(void);

// Set the status pixel; r/g/b 0-255, scaled by a global brightness limit.
void led_set(uint8_t r, uint8_t g, uint8_t b);
