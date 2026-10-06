#pragma once

// WS2812 chain on PIO2 SM2. Pixel 0 is the general status indicator;
// pixels 1..LED_AXIS_COUNT show each motor's speed and direction. The data
// line continues down the stack, so later pixels belong to the LED output.

#include <stdint.h>

#define LED_STATUS      0
#define LED_AXIS_FIRST  1
#define LED_AXIS_COUNT  10
#define LED_COUNT       (LED_AXIS_FIRST + LED_AXIS_COUNT)

void led_init(void);

// Set one pixel in the frame buffer; r/g/b 0-255, scaled by a global
// brightness limit when shown.
void led_set(uint32_t index, uint8_t r, uint8_t g, uint8_t b);

// Send the frame (about 30 us per pixel; blocks while the FIFO is full).
// Call no faster than every ~300 us so the chain latches between frames.
void led_show(void);
