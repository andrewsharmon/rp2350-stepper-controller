#pragma once

// TMF8829 measurement application image (ams OSRAM, MIT licence: see
// LICENSE-MIT.txt). The bootloader writes it to both CPU RAMs at
// TMF8829_IMAGE_START.

#include <stdint.h>

#define TMF8829_IMAGE_START  0x00010000u
#define TMF8829_IMAGE_LENGTH 0x0000386Cu

extern const uint8_t tmf8829_image[TMF8829_IMAGE_LENGTH];
