#pragma once

// Application pin map. Motors always sit on GPIO 4*n..4*n+3; boards with
// fewer GPIOs run the remaining motors without pins.

#include "pico.h"

#define ADC_NONE 0xffu

#if NUM_BANK0_GPIOS > 32
// RP2354B stepper controller (hardware/README.md).
#define BOARD_MOTORS_WITH_PINS 10
#define BOARD_CC1_GPIO        40
#define BOARD_CC2_GPIO        41
#define BOARD_VMOT_GPIO       42
#define BOARD_VMOT_RATIO      5.0f   // 1:5 divider
#define BOARD_ID_GPIO         43
#define BOARD_LADDER_GPIO     44
#define BOARD_WS2812_GPIO     45
#else
// Pico 2 bench (RP2350A, GPIO 0-29). Motors 1-5 on GPIO 0-19. The ladder and
// board ID go on the ADC header pins; VSYS/3 stands in for VMOTOR_SENSE.
#define BOARD_MOTORS_WITH_PINS 5
#define BOARD_CC1_GPIO        ADC_NONE
#define BOARD_CC2_GPIO        ADC_NONE
#define BOARD_VMOT_GPIO       29
#define BOARD_VMOT_RATIO      3.0f   // Pico 2 VSYS/3 divider
#define BOARD_ID_GPIO         28
#define BOARD_LADDER_GPIO     26
#define BOARD_WS2812_GPIO     20
#endif
