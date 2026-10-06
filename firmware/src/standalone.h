#pragma once

// Standalone mode: what the ladder buttons do without a computer.
//
//   Btn1: clear a latched stop (once the e-stop line is back to idle);
//         otherwise start or stop the selected show.
//   Btn2: select the next stored show; if one is playing, switch to it.
//
// Pure logic (no SDK), so it runs in the host tests.

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    SA_NONE,
    SA_CLEAR,       // clear the latched stop
    SA_START,       // start `slot`
    SA_STOP,        // stop the running show
    SA_SWITCH,      // stop the running show, then start `slot` once at rest
    SA_SELECT,      // nothing playing: just remember `slot`
} sa_action_t;

typedef struct {
    bool stop_latched;
    bool line_idle;      // the ladder no longer reads a stop
    int running;         // slot playing, or -1
    int selected;        // slot Btn1 starts, or -1
    uint32_t valid;      // bit mask of slots holding a valid show
} sa_state_t;

typedef struct {
    sa_action_t action;
    int slot;
} sa_result_t;

// Next valid slot after `from` (wrapping), or -1 if none.
int sa_next_slot(uint32_t valid, int from, int n_slots);

sa_result_t sa_button(const sa_state_t *s, int button, int n_slots);
