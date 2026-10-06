#pragma once

// Show slots in flash. Writing pauses both cores (sector erases), so only
// with every axis at rest, like config_save.

#include <stdbool.h>
#include <stdint.h>
#include "show.h"

#define SHOW_SLOTS 4

// The stored blob of a slot (in XIP flash), or NULL if the slot is empty.
// Not yet validated: run it through show_parse.
const uint8_t *show_store_blob(uint32_t slot, uint32_t *len);

// Erase the slot and write `len` bytes (0 just erases). `data` must be
// readable up to len rounded up to 256 bytes.
bool show_store_write(uint32_t slot, const uint8_t *data, uint32_t len);
