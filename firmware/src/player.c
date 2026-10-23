#include "player.h"

#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"

#include "control.h"
#include "led.h"
#include "programs.h"
#include "show_store.h"

// Keep this many points queued per axis (the PVT queue holds 32), sending
// at most POLL_BUDGET commands per poll so the core 1 command queue (64)
// keeps room for everything else.
#define TARGET_DEPTH   12
#define POLL_BUDGET    16

typedef enum { IDLE, PREP, RUN } state_t;

static state_t state = IDLE;
static int slot = -1;              // show slot playing, or -1
static int program = -1;           // built-in program playing, or -1
static const program_t *prog;      // what `show` is built from: NULL for a stored show
static show_t show;                // a program fills in just its axis tracks (looping)
static uint32_t axis_mask;
static uint32_t prep_seq;
static uint32_t t0_us;
static uint64_t next_point[SHOW_MAX_TRACKS];
static uint32_t t_prev[SHOW_MAX_TRACKS];
static uint32_t feed_from;  // track to serve first next poll (round robin)

static uint32_t post(control_cmd_t *c) {
    return control_post(c);  // 0 if the queue is full: callers retry next poll
}

static show_point_t point(uint32_t t, uint64_t i) {
    const show_track_t *tr = &show.track[t];
    return prog ? program_point(prog, tr->channel, NUM_MOTORS, i) : show_axis_point(&show, tr, i);
}

static void finish(void) {
    state = IDLE;
    slot = program = -1;
}

// `show` (and `prog`) are set up: check the axes, then move each to its
// first point.
static const char *begin(int show_slot, int program_k) {
    static char msg[160];
    if (!control_outputs_on)
        return "outputs are stopped (clear first)";

    static control_snapshot_t snap;  // ~1 KB: off core 0's stack
    control_snapshot(&snap);
    axis_mask = 0;
    for (uint32_t t = 0; t < show.n_tracks; t++)
        if (show.track[t].type == SHOW_TRACK_AXIS)
            axis_mask |= 1u << show.track[t].channel;
    for (int k = 0; k < GROUP_COUNT; k++)
        if (snap.group[k].active && (snap.group[k].members & axis_mask))
            return "a show axis is in a group";
    if ((snap.settled_mask & axis_mask) != axis_mask)
        return "show axes must be at rest";
    const char *err = prog ? program_check_limits(prog, NUM_MOTORS, snap.vmax, snap.amax, msg, sizeof msg)
                           : show_check_limits(&show, snap.vmax, snap.amax, msg, sizeof msg);
    if (err)
        return err;

    prep_seq = 0;
    for (uint32_t t = 0; t < show.n_tracks; t++) {
        const show_track_t *tr = &show.track[t];
        next_point[t] = 0;
        t_prev[t] = 0;
        if (tr->type != SHOW_TRACK_AXIS)
            continue;
        control_cmd_t c = {.type = CMD_MOVE, .axes = (uint16_t)(1u << tr->channel),
                           .pos = motion_steps_to_units(point(t, 0).pos)};
        uint32_t seq = post(&c);
        if (seq)
            prep_seq = seq;
    }
    slot = show_slot;
    program = program_k;
    state = PREP;
    return NULL;
}

const char *player_start(uint32_t s) {
    uint32_t len;
    if (state != IDLE)
        return "a show is already running";
    const uint8_t *blob = show_store_blob(s, &len);
    if (!blob)
        return "slot is empty";
    prog = NULL;
    const char *err = show_parse(&show, blob, len, NUM_MOTORS, LED_CHAIN);
    if (err)
        return err;
    return begin((int)s, -1);
}

const char *player_start_program(uint32_t k) {
    if (state != IDLE)
        return "a show is already running";
    if (k >= PROGRAM_COUNT)
        return "no such program";
    prog = &programs[k];
    memset(&show, 0, sizeof show);
    snprintf(show.name, sizeof show.name, "%s", prog->name);
    show.loop = true;
    show.n_tracks = NUM_MOTORS;
    for (uint32_t t = 0; t < show.n_tracks; t++)
        show.track[t] = (show_track_t){.type = SHOW_TRACK_AXIS, .channel = (uint8_t)t};
    return begin(-1, (int)k);
}

void player_stop(void) {
    if (state == IDLE)
        return;
    if (axis_mask) {
        control_cmd_t c = {.type = CMD_STOP, .axes = (uint16_t)axis_mask};
        post(&c);
    }
    finish();
}

// Top up each axis's PVT queue from its keyframes, within the per-poll
// budget, starting at a rotating track so none starves. Returns true while
// any axis still has points to send.
static bool feed(const control_snapshot_t *snap) {
    bool more = false;
    uint32_t budget = POLL_BUDGET;
    for (uint32_t n = 0; n < show.n_tracks; n++) {
        uint32_t t = (feed_from + n) % show.n_tracks;
        const show_track_t *tr = &show.track[t];
        if (tr->type != SHOW_TRACK_AXIS)
            continue;
        uint64_t total = show.loop ? UINT64_MAX : show_axis_points(&show, tr);
        uint32_t depth = snap->pvt_depth[tr->channel];
        for (; budget > 0 && depth < TARGET_DEPTH && next_point[t] < total; budget--) {
            show_point_t pt = point(t, next_point[t]);
            control_cmd_t c = {.type = CMD_PVT_POINT, .axes = (uint16_t)(1u << tr->channel),
                               .pos = motion_steps_to_units(pt.pos), .f1 = pt.vel,
                               .ms = pt.t_ms - t_prev[t]};
            if (!post(&c))
                break;
            t_prev[t] = pt.t_ms;
            next_point[t]++;
            depth++;
        }
        more |= next_point[t] < total;
    }
    feed_from = (feed_from + 1) % (show.n_tracks ? show.n_tracks : 1);
    return more;
}

void player_poll(uint32_t now_us) {
    if (state == IDLE)
        return;
    if (!control_outputs_on) {  // e-stop: the axes are already halted
        finish();
        return;
    }
    static control_snapshot_t snap;  // ~1 KB: off core 0's stack
    control_snapshot(&snap);

    if (state == PREP) {
        bool applied = (int32_t)(snap.last_seq - prep_seq) >= 0;
        if (!applied || (snap.settled_mask & axis_mask) != axis_mask)
            return;
        // Prefill a few points per axis (the queue keeps order), then start
        // them all on one tick. Until the start command is queued, stay here.
        bool prefilled = true;
        for (uint32_t t = 0; t < show.n_tracks; t++)
            if (show.track[t].type == SHOW_TRACK_AXIS && next_point[t] < 2 &&
                next_point[t] < (show.loop ? 2 : show_axis_points(&show, &show.track[t])))
                prefilled = false;
        if (!prefilled) {
            feed(&snap);
            return;
        }
        if (axis_mask) {
            control_cmd_t c = {.type = CMD_PVT_START, .axes = (uint16_t)axis_mask};
            if (!control_post(&c))
                return;  // queue full: try again next poll
        }
        t0_us = now_us;
        state = RUN;
        return;
    }

    bool more = feed(&snap);
    uint32_t t_ms = (now_us - t0_us) / 1000;
    if (!show.loop && !more && t_ms > show.duration_ms &&
        (snap.settled_mask & axis_mask) == axis_mask)
        finish();
}

int player_slot(void) {
    return slot;
}

int player_program(void) {
    return program;
}

const char *player_state(void) {
    return state == IDLE ? "idle" : state == PREP ? "moving to start" : "running";
}

bool player_led(uint32_t px, uint8_t rgb[3]) {
    if (state == IDLE)
        return false;
    uint32_t t_ms = state == RUN ? (time_us_32() - t0_us) / 1000 : 0;
    for (uint32_t t = 0; t < show.n_tracks; t++) {
        const show_track_t *tr = &show.track[t];
        if (tr->type == SHOW_TRACK_LED && tr->channel == px) {
            show_led_at(&show, tr, t_ms, rgb);
            return true;
        }
    }
    return false;
}
