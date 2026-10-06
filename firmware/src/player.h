#pragma once

// Show player (core 0). Running a show: the show's axes move to their first
// keyframe, then start together; each axis's keyframes are streamed to its
// PVT queue as it drains, and LED tracks override their pixels. Stops on
// player_stop, an e-stop, or the end of a non-looping show.

#include <stdbool.h>
#include <stdint.h>

// Start the show in a slot. Returns NULL, or why it can't run (empty slot,
// invalid show, an axis busy or grouped, a segment over the axis limits).
const char *player_start(uint32_t slot);
void player_stop(void);
void player_poll(uint32_t now_us);

// -1 when idle.
int player_slot(void);
const char *player_state(void);   // "idle", "moving to start", "running"

// LED override for pixel `px` while a show runs; false if the show doesn't
// drive it.
bool player_led(uint32_t px, uint8_t rgb[3]);
