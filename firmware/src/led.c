#include "led.h"

#include "hardware/pio.h"
#include "board_pins.h"
#include "ws2812.pio.h"

// PIO2: motors 9-10 use SM0-1, the LED SM2 (hardware/README.md).
#define LED_SM 2
// Full-white WS2812 is ~60 mA and blinding at desk distance.
#define LED_BRIGHTNESS 32   // of 255

static int led_offset = -1;

void led_init(void) {
    // hbridge_init() set PIO2's GPIO base on 48-GPIO parts before this runs.
    led_offset = pio_add_program(pio2, &ws2812_program);
    pio_sm_claim(pio2, LED_SM);
    ws2812_program_init(pio2, LED_SM, (uint)led_offset, BOARD_WS2812_GPIO, 800000.0f);
    led_set(0, 0, 0);
}

static uint32_t scale(uint8_t v) {
    return ((uint32_t)v * LED_BRIGHTNESS + 127) / 255;
}

void led_set(uint8_t r, uint8_t g, uint8_t b) {
    if (led_offset < 0)
        return;
    uint32_t grb = scale(g) << 16 | scale(r) << 8 | scale(b);
    pio_sm_put(pio2, LED_SM, grb << 8);  // drop the pixel if the FIFO is full
}
