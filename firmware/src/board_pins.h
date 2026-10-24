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
#ifdef BOARD_SINGLE_SH_PROTO
#define BOARD_VMOT_RATIO      4.9f   // 39k / 10k divider
#else
#define BOARD_VMOT_RATIO      5.0f   // 1:5 divider
#endif
#define BOARD_ID_GPIO         43
#define BOARD_LADDER_GPIO     44
#define BOARD_WS2812_GPIO     45
// Qwiic: I2C1 target (hardware/README.md). External pull-ups (RP2350-E9).
#define BOARD_I2C_SDA_GPIO    46
#define BOARD_I2C_SCL_GPIO    47
#define BOARD_I2C_TARGET_ADDR 0x42
#ifdef BOARD_SINGLE_SH_PROTO
// Axes in row order along the motor connectors, from the far end to the
// USB end: J13 M5 M6, J14 M7 M8, J15 M9 M10, J11 M1 M2, J12 M3 M4. The ToF
// map (tof_overlay.c) lays its slices out in this order.
#define BOARD_AXIS_ROW {4, 5, 6, 7, 8, 9, 0, 1, 2, 3}
#endif
#else
// Pico 2 bench (RP2350A, GPIO 0-29). Motors 1-5 on GPIO 0-19. The ladder and
// board ID go on the ADC header pins; VSYS/3 stands in for VMOTOR_SENSE.
// No free pins for the I2C target, so it is compiled out.
#define BOARD_MOTORS_WITH_PINS 5
#define BOARD_CC1_GPIO        ADC_NONE
#define BOARD_CC2_GPIO        ADC_NONE
#define BOARD_VMOT_GPIO       29
#define BOARD_VMOT_RATIO      3.0f   // Pico 2 VSYS/3 divider
#define BOARD_ID_GPIO         28
#define BOARD_LADDER_GPIO     26
#define BOARD_WS2812_GPIO     20
#endif
