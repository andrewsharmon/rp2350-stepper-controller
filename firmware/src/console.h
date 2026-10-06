#pragma once

// Line-based USB serial console. Type `help` for the command list.

#include <stdbool.h>
#include <stdint.h>
#include "config.h"

// Feed one received character (echoes it; runs the line on Enter).
void console_input(int c);

// Emit a telemetry line if one is due. Call often from core 0.
void console_telemetry(uint32_t now_us);

// Configure telemetry (text from the console, binary from protocol.c).
void console_set_telemetry(uint32_t hz, uint32_t axes, uint32_t fields, uint32_t sys,
                           bool events, bool binary);

// Provided by main.c.
void app_print_status(void);
void app_set_stress(bool on);
bool app_stress_on(void);
// NULL on success, else why the save was refused.
const char *app_save_config(void);
const config_t *app_config(void);   // as loaded / last saved
void app_set_boot_show(int slot);   // -1: none (save to keep it)
