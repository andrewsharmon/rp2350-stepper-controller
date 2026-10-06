// Board header for the RP2354B stepper controller (tier 1).

// pico_cmake_set PICO_PLATFORM=rp2350

#ifndef _BOARDS_RP2354B_STEPPER_H
#define _BOARDS_RP2354B_STEPPER_H

// RP2354B: QFN-80 (48 GPIO) with 2 MB in-package flash.
#define PICO_RP2350A 0

// pico_cmake_set_default PICO_FLASH_SIZE_BYTES = (2 * 1024 * 1024)
#ifndef PICO_FLASH_SIZE_BYTES
#define PICO_FLASH_SIZE_BYTES (2 * 1024 * 1024)
#endif

#define PICO_XOSC_STARTUP_DELAY_MULTIPLIER 64

#ifndef PICO_DEFAULT_I2C
#define PICO_DEFAULT_I2C 1
#endif
#ifndef PICO_DEFAULT_I2C_SDA_PIN
#define PICO_DEFAULT_I2C_SDA_PIN 46
#endif
#ifndef PICO_DEFAULT_I2C_SCL_PIN
#define PICO_DEFAULT_I2C_SCL_PIN 47
#endif

#ifndef PICO_DEFAULT_WS2812_PIN
#define PICO_DEFAULT_WS2812_PIN 45
#endif

#ifndef PICO_RP2350_A2_SUPPORTED
#define PICO_RP2350_A2_SUPPORTED 1
#endif

#endif
