#include "motion.h"

#include <math.h>

#define DT (1.0f / (float)MOTION_TICK_HZ)
#define UNITS_PER_STEP_F ((float)MOTION_UNITS_PER_STEP)
#define STEPS_PER_UNIT_F (1.0f / (float)MOTION_UNITS_PER_STEP)

void motion_init(motion_axis_t *ax, float vmax, float amax) {
    *ax = (motion_axis_t){0};
    ax->mode = MODE_IDLE;
    ax->settled = true;
    ax->vmax = vmax;
    ax->amax = amax;
}

void motion_move_to(motion_axis_t *ax, int64_t target) {
    ax->target = target;
    ax->mode = MODE_POSITION;
    ax->settled = false;
}

void motion_set_velocity(motion_axis_t *ax, float vel) {
    ax->cmd_vel = vel;
    ax->mode = MODE_VELOCITY;
    ax->settled = false;
}

void motion_jog(motion_axis_t *ax, float vel, uint32_t timeout_ms) {
    ax->cmd_vel = vel;
    ax->jog_ticks_left = timeout_ms * (MOTION_TICK_HZ / 1000u);
    ax->mode = MODE_JOG;
    ax->settled = false;
}

void motion_stop(motion_axis_t *ax) {
    if (ax->mode == MODE_IDLE)
        return;
    // Velocity mode toward zero drops to IDLE once at rest.
    ax->cmd_vel = 0.0f;
    ax->mode = MODE_VELOCITY;
}

bool motion_set_position(motion_axis_t *ax, int64_t pos) {
    if (!ax->settled)
        return false;
    ax->pos = pos;
    ax->target = pos;
    return true;
}

static float clampf(float x, float lo, float hi) {
    return x < lo ? lo : x > hi ? hi : x;
}

// Move v toward v_des by at most dv.
static float approach(float v, float v_des, float dv) {
    if (v < v_des)
        return v + dv > v_des ? v_des : v + dv;
    return v - dv < v_des ? v_des : v - dv;
}

void motion_tick(motion_axis_t *ax) {
    float v = ax->vel;
    float dv = ax->amax * DT;
    float v_new;

    switch (ax->mode) {
    case MODE_IDLE:
        ax->vel = 0.0f;
        ax->settled = true;
        return;

    case MODE_POSITION: {
        float d = (float)(ax->target - ax->pos) * STEPS_PER_UNIT_F;
        // Fastest next speed u that still stops on the target: this tick's
        // travel (v + u)/2*dt plus the braking distance u^2/(2a) must fit in
        // the distance left. Solving that quadratic for u keeps the
        // discrete ramp from overshooting. Speeds here are toward the target.
        float a = ax->amax, dist = fabsf(d);
        float v_toward = d >= 0.0f ? v : -v;
        float h = 0.5f * a * DT;
        float disc = h * h + 2.0f * a * dist - a * v_toward * DT;
        float v_stop = disc > 0.0f ? sqrtf(disc) - h : 0.0f;
        float v_des = copysignf(fminf(v_stop, ax->vmax), d);
        v_new = approach(v, v_des, dv);

        // Arrive: when this tick would reach or cross the target at a speed
        // we could have stopped from in one tick, land exactly on it.
        float travel = 0.5f * (v + v_new) * DT;
        if (fabsf(d) <= fabsf(travel) + 1e-6f && fabsf(v_new) <= 2.0f * dv) {
            ax->pos = ax->target;
            ax->vel = 0.0f;
            ax->mode = MODE_IDLE;
            ax->settled = true;
            return;
        }
        break;
    }

    case MODE_JOG:
        if (ax->jog_ticks_left > 0)
            ax->jog_ticks_left--;
        else
            ax->cmd_vel = 0.0f;  // jog watchdog: no refresh, stop
        /* fall through */
    case MODE_VELOCITY:
        v_new = approach(v, clampf(ax->cmd_vel, -ax->vmax, ax->vmax), dv);
        if (v_new == 0.0f && ax->cmd_vel == 0.0f) {
            ax->vel = 0.0f;
            ax->mode = MODE_IDLE;
            ax->settled = true;
            return;
        }
        break;

    default:
        return;
    }

    ax->pos += (int64_t)(0.5f * (v + v_new) * DT * UNITS_PER_STEP_F);
    ax->vel = v_new;
    ax->settled = false;
}
