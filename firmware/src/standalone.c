#include "standalone.h"

int sa_next_slot(uint32_t valid, int from, int n_slots) {
    for (int k = 1; k <= n_slots; k++) {
        int slot = ((from < 0 ? -1 : from) + k) % n_slots;
        if (valid & (1u << slot))
            return slot;
    }
    return -1;
}

sa_result_t sa_button(const sa_state_t *s, int button, int n_slots) {
    sa_result_t r = {SA_NONE, -1};
    if (button == 1) {
        if (s->stop_latched) {
            if (s->line_idle)
                r.action = SA_CLEAR;
        } else if (s->running >= 0) {
            r.action = SA_STOP;
        } else {
            int slot = s->selected >= 0 && (s->valid & (1u << s->selected))
                     ? s->selected : sa_next_slot(s->valid, -1, n_slots);
            if (slot >= 0) {
                r.action = SA_START;
                r.slot = slot;
            }
        }
    } else if (button == 2 && !s->stop_latched) {
        int from = s->running >= 0 ? s->running : s->selected;
        int slot = sa_next_slot(s->valid, from, n_slots);
        if (slot >= 0) {
            r.slot = slot;
            r.action = s->running >= 0 ? (slot == s->running ? SA_NONE : SA_SWITCH) : SA_SELECT;
        }
    }
    return r;
}
