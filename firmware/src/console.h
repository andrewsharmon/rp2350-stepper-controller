#pragma once

// Line-based USB serial console. Type `help` for the command list.

#include <stdbool.h>
#include <stdint.h>

// Feed one received character (echoes it; runs the line on Enter).
void console_input(int c);

// Emit a telemetry line if one is due. Call often from core 0.
void console_telemetry(uint32_t now_us);

// Provided by main.c.
void app_print_status(void);
void app_set_stress(bool on);
bool app_stress_on(void);
