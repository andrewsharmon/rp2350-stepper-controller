#include "control.h"

#include <string.h>
#include "pico/stdlib.h"
#include "pico/util/queue.h"
#include "hardware/sync.h"

#include "adc_monitor.h"
#include "hbridge.h"
#include "microstep.h"

// Automatic drive amplitude (fraction of full PWM span), tuned on the bench
// 8 mm stepper at 5 V: flat up to AMP_LOW_SPEED, linear up to the cap at
// AMP_HIGH_SPEED to make up for back-EMF. Drops to AMP_HOLD after the axis
// has been settled for HOLD_DELAY_MS, since standstill heats the coils most.
#define AMP_LOW          0.40f
#define AMP_LOW_SPEED    300.0f   // full steps/s
#define AMP_HIGH         0.60f
#define AMP_HIGH_SPEED   1600.0f
#define AMP_HOLD         0.25f
#define HOLD_DELAY_MS    500
// Amplitude slew limit, so run/hold changes don't jerk the rotor.
#define AMP_SLEW_PER_SEC 2.0f

// One ADC round covers every monitor input; don't count the same ladder
// sample twice toward the stop debounce.
#define LADDER_EVAL_US   10

#define PERIODS_PER_TICK (HBRIDGE_PWM_HZ / MOTION_TICK_HZ)
#define CMD_QUEUE_LEN    32

extern hbridge_t motors[NUM_MOTORS];  // main.c

volatile bool control_outputs_on = true;
volatile bool control_clear_request;
volatile bool control_clear_refused;
volatile bool control_energized;
volatile float control_manual_amp = -1.0f;
volatile bool control_stall_test;
volatile uint32_t control_heartbeat;
volatile uint32_t control_busy_us;
volatile uint32_t control_min_queued = HBRIDGE_RING_PERIODS;
ladder_t control_ladder;

static queue_t cmd_queue;

// Snapshot with a sequence lock: odd while core 1 is writing.
static volatile uint32_t snap_seq;
static control_snapshot_t snap_buf;

// Core 1 state.
static motion_axis_t axes[NUM_MOTORS];
static microstep_t steppers[NUM_MOTORS];
static uint32_t rejected;
static uint32_t last_seq;
static uint32_t pvt_dropped;

void control_init(void) {
    queue_init(&cmd_queue, sizeof(control_cmd_t), CMD_QUEUE_LEN);
    for (int i = 0; i < NUM_MOTORS; i++)
        motion_init(&axes[i], CONTROL_DEFAULT_VMAX, CONTROL_DEFAULT_AMAX);
    ladder_reset(&control_ladder, time_us_32());
}

uint32_t control_post(const control_cmd_t *cmd) {
    static uint32_t next_seq;
    control_cmd_t c = *cmd;
    c.seq = ++next_seq ? next_seq : ++next_seq;  // 0 means "failed"
    if (!queue_try_add(&cmd_queue, &c)) {
        next_seq--;
        return 0;
    }
    return c.seq;
}

void control_snapshot(control_snapshot_t *snap) {
    uint32_t s0, s1;
    do {
        s0 = snap_seq;
        __dmb();
        memcpy(snap, (const void *)&snap_buf, sizeof *snap);
        __dmb();
        s1 = snap_seq;
    } while (s0 != s1 || (s0 & 1));
}

static inline float auto_amplitude(float speed) {
    const float slope = (AMP_HIGH - AMP_LOW) / (AMP_HIGH_SPEED - AMP_LOW_SPEED);
    float v = speed < 0.0f ? -speed : speed;
    if (v <= AMP_LOW_SPEED)
        return AMP_LOW;
    if (v >= AMP_HIGH_SPEED)
        return AMP_HIGH;
    return AMP_LOW + slope * (v - AMP_LOW_SPEED);
}

static void apply(const control_cmd_t *c) {
    last_seq = c->seq;
    for (int i = 0; i < NUM_MOTORS; i++) {
        if (!(c->axes & (1u << i)))
            continue;
        motion_axis_t *ax = &axes[i];
        switch (c->type) {
        case CMD_MOVE:
            motion_move_to(ax, c->pos);
            break;
        case CMD_MOVE_REL:
            // Relative to where the generator is headed, not the filtered
            // output (which lags it).
            motion_move_to(ax, (ax->mode == MODE_POSITION ? ax->target : ax->gen_pos) + c->pos);
            break;
        case CMD_VELOCITY:
            motion_set_velocity(ax, c->f1);
            break;
        case CMD_JOG:
            motion_jog(ax, c->f1, c->ms);
            break;
        case CMD_STOP:
            motion_stop(ax);
            break;
        case CMD_SET_POS:
            if (!motion_set_position(ax, c->pos))
                rejected++;
            break;
        case CMD_LIMITS:
            if (c->f1 > 0.0f)
                ax->vmax = c->f1;
            if (c->f2 > 0.0f)
                ax->amax = c->f2;
            break;
        case CMD_PROFILE:
            if (!motion_set_profile(ax, (motion_profile_t)c->ms, (uint32_t)c->f1))
                rejected++;
            break;
        case CMD_PVT_POINT:
            if (!motion_pvt_push(ax, c->pos, c->f1, c->ms))
                pvt_dropped++;
            break;
        case CMD_PVT_START:
            if (!motion_pvt_start(ax))
                rejected++;
            break;
        }
    }
}

// Halt every axis where it is (outputs are already off).
static void halt_all(void) {
    for (int i = 0; i < NUM_MOTORS; i++)
        motion_halt(&axes[i]);
}

// Returns true if a stop is latched (outputs are off).
static bool __time_critical_func(poll_ladder)(uint32_t *last_eval) {
#ifndef LADDER_BYPASS
    uint32_t now = time_us_32();
    if (now - *last_eval >= LADDER_EVAL_US) {
        *last_eval = now;
        if (ladder_update(&control_ladder, adc_monitor_pin_mv(MON_LADDER), now) &&
            control_outputs_on) {
            for (int i = 0; i < NUM_MOTORS; i++)
                hbridge_safe_off(&motors[i]);
            control_outputs_on = false;
            halt_all();
        }
    }
#else
    (void)last_eval;
#endif
    return !control_outputs_on;
}

static void publish(uint32_t tick, const float *amp, uint32_t hold) {
    snap_seq++;
    __dmb();
    snap_buf.tick = tick;
    for (int i = 0; i < NUM_MOTORS; i++) {
        snap_buf.pos[i] = axes[i].pos;
        snap_buf.vel[i] = axes[i].vel;
        snap_buf.mode[i] = (uint8_t)axes[i].mode;
        snap_buf.amp[i] = amp[i];
        snap_buf.vmax[i] = axes[i].vmax;
        snap_buf.amax[i] = axes[i].amax;
        snap_buf.profile[i] = (uint8_t)axes[i].profile;
        snap_buf.jerk_ms[i] = (uint16_t)(axes[i].jerk_ticks * 1000u / MOTION_TICK_HZ);
    }
    uint32_t underruns = 0, settled = 0;
    for (int i = 0; i < NUM_MOTORS; i++) {
        snap_buf.pvt_depth[i] = (uint8_t)axes[i].pvt_count;
        snap_buf.pvt_underrun[i] = (uint16_t)axes[i].pvt_underruns;
        underruns += axes[i].pvt_underruns;
        if (axes[i].settled)
            settled |= 1u << i;
    }
    snap_buf.settled_mask = settled;
    snap_buf.last_seq = last_seq;
    snap_buf.pvt_underruns = underruns;
    snap_buf.pvt_dropped = pvt_dropped;
    snap_buf.holding_mask = hold;
    snap_buf.rejected = rejected;
    __dmb();
    snap_seq++;
}

void __time_critical_func(control_core1_main)(void) {
    int32_t max_duty = hbridge_max_duty();
    int32_t min_duty = hbridge_min_duty();

    float amp[NUM_MOTORS] = {0};
    uint32_t settled_ticks[NUM_MOTORS] = {0};
    const uint32_t hold_ticks = HOLD_DELAY_MS * (MOTION_TICK_HZ / 1000);
    const float amp_step = AMP_SLEW_PER_SEC / (float)HBRIDGE_PWM_HZ;

    // Interpolation of the current tick: per-period phase step, plus one
    // extra unit for the first `extra` periods so the tick lands exactly.
    int32_t step[NUM_MOTORS] = {0};
    uint32_t extra[NUM_MOTORS] = {0};
    int32_t extra_sign[NUM_MOTORS] = {0};
    float want_amp[NUM_MOTORS] = {0};
    uint32_t sub = 0;      // period within the tick
    uint32_t tick = 0;
    uint32_t hold = 0;

    for (int i = 0; i < NUM_MOTORS; i++)
        steppers[i].phase = (uint32_t)axes[i].pos;
    uint32_t last_eval = time_us_32();
    control_cmd_t cmd;

    for (;;) {
        while (control_stall_test)
            tight_loop_contents();  // watchdog test: stop the heartbeat
        control_heartbeat++;

        if (control_clear_request) {
            control_clear_request = false;
            if (!control_outputs_on) {
                if (ladder_clear_stop(&control_ladder)) {
                    for (int i = 0; i < NUM_MOTORS; i++) {
                        amp[i] = 0.0f;
                        steppers[i].phase = (uint32_t)axes[i].pos;
                    }
                    sub = 0;
                    hbridge_restart(motors, NUM_MOTORS);
                    control_outputs_on = true;
                } else {
                    control_clear_refused = true;
                }
            }
        }
        if (poll_ladder(&last_eval)) {
            // Keep draining commands so the queue doesn't fill; motion ones
            // are dropped (the axes are halted).
            while (queue_try_remove(&cmd_queue, &cmd))
                if (cmd.type == CMD_SET_POS || cmd.type == CMD_LIMITS || cmd.type == CMD_PROFILE)
                    apply(&cmd);
            for (int i = 0; i < NUM_MOTORS; i++)
                amp[i] = 0.0f;
            publish(tick, amp, 0);
            continue;
        }

        uint32_t n = HBRIDGE_RING_PERIODS;
        for (int i = 0; i < NUM_MOTORS; i++) {
            uint32_t f = hbridge_free_periods(&motors[i]);
            if (f < n)
                n = f;
        }
        if (n == 0)
            continue;
        uint32_t queued = HBRIDGE_RING_PERIODS - 1 - n;
        if (queued < control_min_queued)
            control_min_queued = queued;
        uint32_t t_start = time_us_32();
        float manual = control_manual_amp;
        bool energized = control_energized;

        for (uint32_t k = 0; k < n && !poll_ladder(&last_eval); k++) {
            if (sub == 0) {
                // 1 kHz trajectory tick.
                while (queue_try_remove(&cmd_queue, &cmd))
                    apply(&cmd);
                hold = 0;
                for (int i = 0; i < NUM_MOTORS; i++) {
                    int64_t before = axes[i].pos;
                    motion_tick(&axes[i]);
                    int64_t delta = axes[i].pos - before;
                    // One tick can exceed 2^31 units above ~2000 steps/s,
                    // so split in 64 bits; per period it fits 32.
                    int64_t q = delta / (int64_t)PERIODS_PER_TICK;
                    step[i] = (int32_t)q;
                    int32_t rem = (int32_t)(delta - q * (int64_t)PERIODS_PER_TICK);
                    extra[i] = (uint32_t)(rem < 0 ? -rem : rem);
                    extra_sign[i] = rem < 0 ? -1 : 1;

                    if (axes[i].settled)
                        settled_ticks[i] += settled_ticks[i] < hold_ticks;
                    else
                        settled_ticks[i] = 0;
                    bool holding = settled_ticks[i] >= hold_ticks && manual < 0.0f;
                    if (holding)
                        hold |= 1u << i;
                    want_amp[i] = !energized ? 0.0f
                                : manual >= 0.0f ? manual
                                : holding ? AMP_HOLD
                                : auto_amplitude(axes[i].vel);
                }
                tick++;
                publish(tick, amp, hold);
            }

            for (int i = 0; i < NUM_MOTORS; i++) {
                float a = amp[i], want = want_amp[i];
                if (a < want)
                    a = a + amp_step > want ? want : a + amp_step;
                else if (a > want)
                    a = a - amp_step < want ? want : a - amp_step;
                amp[i] = a;

                int32_t duty_a, duty_b;
                steppers[i].amplitude = a;
                microstep_duties(&steppers[i], max_duty, min_duty, &duty_a, &duty_b);
                hbridge_write_period(&motors[i], duty_a, duty_b);
                steppers[i].phase += (uint32_t)(step[i] + (sub < extra[i] ? extra_sign[i] : 0));
            }
            sub = sub + 1 < PERIODS_PER_TICK ? sub + 1 : 0;
        }
        control_busy_us += time_us_32() - t_start;
    }
}
