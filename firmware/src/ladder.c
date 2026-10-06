#include "ladder.h"

// Nominal: STOP 3300, idle 2200, Btn1 1320, Btn2 730, fault 0 (mV).
#define TH_ESTOP_MV 2750
#define TH_IDLE_MV  1760
#define TH_BTN1_MV  1025
#define TH_BTN2_MV  365
// E-stop open + Btn1 reads 1650 mV, inside the Btn1 band; split it off midway.
#define TH_BTN1_MASKED_MV 1490

ladder_level_t ladder_classify(uint32_t mv) {
    if (mv >= TH_ESTOP_MV) return LADDER_ESTOP;
    if (mv >= TH_IDLE_MV)  return LADDER_IDLE;
    if (mv >= TH_BTN1_MV)  return LADDER_BTN1;
    if (mv >= TH_BTN2_MV)  return LADDER_BTN2;
    return LADDER_FAULT;
}

static bool is_stop(ladder_level_t level) {
    return level == LADDER_ESTOP || level == LADDER_FAULT;
}

void ladder_reset(ladder_t *l, uint32_t now_us) {
    *l = (ladder_t){0};
    l->raw_prev = LADDER_IDLE;
    l->raw_since_us = now_us;
    l->stable = LADDER_IDLE;
}

bool ladder_update(ladder_t *l, uint32_t mv, uint32_t now_us) {
    ladder_level_t raw = ladder_classify(mv);
    if (raw != l->raw_prev) {
        l->raw_prev = raw;
        l->raw_count = 0;
        l->raw_since_us = now_us;
    }
    if (l->raw_count < UINT32_MAX)
        l->raw_count++;

    bool masked = raw == LADDER_BTN1 && mv >= TH_BTN1_MASKED_MV;
    if (masked && !l->masked)
        l->masked_since_us = now_us;
    l->masked = masked;

    ladder_level_t stop = raw;
    l->held_stop = false;
    if (!is_stop(raw)) {
        bool button = raw == LADDER_BTN1 || raw == LADDER_BTN2;
        l->held_stop = (masked && now_us - l->masked_since_us >= LADDER_MASKED_US) ||
                       (button && now_us - l->raw_since_us >= LADDER_BUTTON_MAX_US);
        if (l->held_stop)
            stop = LADDER_ESTOP;
    } else if (l->raw_count < LADDER_STOP_SAMPLES) {
        return false;
    }
    if (is_stop(stop)) {
        bool newly = !l->stop_latched;
        l->stable = stop;
        if (newly) {
            l->stop_latched = true;
            l->stop_cause = stop;
        }
        return newly;
    }

    // A press in the masked band is not counted: it may be an open e-stop.
    if (raw != l->stable && !masked && now_us - l->raw_since_us >= LADDER_BUTTON_US) {
        if (raw == LADDER_BTN1) l->btn1_presses++;
        if (raw == LADDER_BTN2) l->btn2_presses++;
        l->stable = raw;
    }
    return false;
}

bool ladder_clear_stop(ladder_t *l) {
    if (is_stop(l->raw_prev) || l->held_stop || l->masked)
        return false;
    l->stop_latched = false;
    return true;
}

const char *ladder_level_name(ladder_level_t level) {
    switch (level) {
    case LADDER_IDLE:  return "idle";
    case LADDER_BTN1:  return "btn1";
    case LADDER_BTN2:  return "btn2";
    case LADDER_ESTOP: return "e-stop open";
    case LADDER_FAULT: return "driver fault";
    }
    return "?";
}
