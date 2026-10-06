#include "motion.h"

#include <math.h>
#include <string.h>

#include "trig.h"

#define DT (1.0f / (float)MOTION_TICK_HZ)
#define UNITS_PER_STEP_F ((float)MOTION_UNITS_PER_STEP)
#define STEPS_PER_UNIT_F (1.0f / (float)MOTION_UNITS_PER_STEP)
#define PI_F 3.14159265f

#define DEFAULT_JERK_MS 30u
// Fixed-time segments integrate in float and re-sync to the exact (double)
// curve this often; doubles are software on the M33, so not every tick.
#define SEG_SYNC_TICKS 32u

const char *motion_profile_name(motion_profile_t p) {
    switch (p) {
    case PROFILE_TRAP:    return "trap";
    case PROFILE_SCURVE:  return "scurve";
    case PROFILE_SMOOTH:  return "smooth";
    case PROFILE_COSINE:  return "cosine";
    case PROFILE_QUINTIC: return "quintic";
    default:              return "?";
    }
}

// --- moving-average filter ---------------------------------------------------

static void filter_reset(motion_filter_t *f, uint32_t len) {
    memset(f, 0, sizeof *f);
    f->len = len;
}

static bool filter_flushed(const motion_filter_t *f) {
    return f->nonzero == 0 && f->carry == 0;
}

static int64_t filter_step(motion_filter_t *f, int64_t in) {
    if (f->len <= 1)
        return in;
    int64_t old = f->ring[f->head];
    f->nonzero += (in != 0) - (old != 0);
    f->ring[f->head] = in;
    f->head = f->head + 1 < f->len ? f->head + 1 : 0;
    f->sum += in - old;
    // Output the average; carry the remainder so nothing is lost. Each input
    // appears in `len` windows, so once flushed the carry returns to zero.
    int64_t acc = f->sum + f->carry;
    int64_t out = acc / (int64_t)f->len;
    f->carry = acc - out * (int64_t)f->len;
    return out;
}

static uint32_t filter_stages(const motion_axis_t *ax) {
    return ax->profile == PROFILE_SCURVE ? 1 : ax->profile == PROFILE_SMOOTH ? 2 : 0;
}

// --- setup and commands -------------------------------------------------------

void motion_init(motion_axis_t *ax, float vmax, float amax) {
    memset(ax, 0, sizeof *ax);
    ax->mode = MODE_IDLE;
    ax->settled = true;
    ax->vmax = vmax;
    ax->amax = amax;
    motion_set_profile(ax, PROFILE_SCURVE, DEFAULT_JERK_MS);
}

bool motion_set_profile(motion_axis_t *ax, motion_profile_t profile, uint32_t jerk_ms) {
    if (!ax->settled || profile >= PROFILE_COUNT)
        return false;
    uint32_t len = jerk_ms * (MOTION_TICK_HZ / 1000u);
    if (len < 1)
        len = 1;
    if (len > MOTION_MAX_FILTER)
        len = MOTION_MAX_FILTER;
    ax->profile = profile;
    ax->jerk_ticks = len;
    filter_reset(&ax->filt[0], len);
    filter_reset(&ax->filt[1], len);
    return true;
}

static bool fixed_time(const motion_axis_t *ax) {
    return ax->profile == PROFILE_COSINE || ax->profile == PROFILE_QUINTIC;
}

static void start_segment(motion_axis_t *ax);

void motion_move_to(motion_axis_t *ax, int64_t target) {
    ax->target = target;
    ax->mode = MODE_POSITION;
    ax->settled = false;
    ax->seg_active = false;
    if (fixed_time(ax))
        start_segment(ax);
}

void motion_set_velocity(motion_axis_t *ax, float vel) {
    ax->cmd_vel = vel;
    ax->mode = MODE_VELOCITY;
    ax->settled = false;
    ax->seg_active = false;
}

void motion_jog(motion_axis_t *ax, float vel, uint32_t timeout_ms) {
    motion_set_velocity(ax, vel);
    ax->jog_ticks_left = timeout_ms * (MOTION_TICK_HZ / 1000u);
    ax->mode = MODE_JOG;
}

void motion_stop(motion_axis_t *ax) {
    if (ax->mode == MODE_IDLE)
        return;
    // Velocity mode toward zero drops to IDLE once at rest. A fixed-time
    // segment hands its current speed over to the trapezoid ramp.
    ax->seg_active = false;
    ax->cmd_vel = 0.0f;
    ax->mode = MODE_VELOCITY;
}

bool motion_set_position(motion_axis_t *ax, int64_t pos) {
    if (!ax->settled)
        return false;
    ax->pos = ax->gen_pos = ax->target = pos;
    return true;
}

void motion_halt(motion_axis_t *ax) {
    ax->gen_pos = ax->target = ax->pos;
    ax->vel = ax->gen_vel = ax->gen_acc = 0.0f;
    ax->mode = MODE_IDLE;
    ax->seg_active = false;
    ax->settled = true;
    filter_reset(&ax->filt[0], ax->jerk_ticks);
    filter_reset(&ax->filt[1], ax->jerk_ticks);
}

// --- fixed-time segments ------------------------------------------------------

// Quintic in tau = t/T from (0, v0, a0) to (d, 0, 0); coefficients in steps.
// From rest this is d * (10 tau^3 - 15 tau^4 + 6 tau^5).
static void set_quintic(motion_axis_t *ax, double d, double v0, double a0, double T) {
    double vT = v0 * T, aT2 = a0 * T * T;
    double *c = ax->seg_cd;
    c[0] = 0.0;
    c[1] = vT;
    c[2] = 0.5 * aT2;
    c[3] = 10.0 * d - 6.0 * vT - 1.5 * aT2;
    c[4] = -15.0 * d + 8.0 * vT + 1.5 * aT2;
    c[5] = 6.0 * d - 3.0 * vT - 0.5 * aT2;
    for (int k = 0; k < 6; k++)
        ax->seg_c[k] = (float)c[k];
    ax->seg_T = (float)T;
}

static void segment_eval(const motion_axis_t *ax, float t, float *x, float *v, float *a) {
    float T = ax->seg_T, u = t / T;
    if (ax->seg_cosine) {
        // cos(pi u) and sin(pi u) for u in [0, 1] via the |x| <= pi/2 core.
        float d = ax->seg_c[0], w = PI_F / T;
        float c = trig_sin_core(PI_F * (0.5f - u));
        float s = trig_sin_core(PI_F * (0.5f - fabsf(u - 0.5f)));
        *x = 0.5f * d * (1.0f - c);
        *v = 0.5f * d * w * s;
        *a = 0.5f * d * w * w * c;
        return;
    }
    const float *c = ax->seg_c;
    *x = c[0] + u * (c[1] + u * (c[2] + u * (c[3] + u * (c[4] + u * c[5]))));
    *v = (c[1] + u * (2.0f * c[2] + u * (3.0f * c[3] + u * (4.0f * c[4] + u * 5.0f * c[5])))) / T;
    *a = (2.0f * c[2] + u * (6.0f * c[3] + u * (12.0f * c[4] + u * 20.0f * c[5]))) / (T * T);
}

// Exact displacement at tick n, in double (slow: software on the M33).
static double segment_x_exact(const motion_axis_t *ax, uint32_t n) {
    double u = (double)n / (double)MOTION_TICK_HZ / (double)ax->seg_T;
    const double *c = ax->seg_cd;
    if (ax->seg_cosine)
        return 0.5 * c[0] * (1.0 - trig_cos_pi_d(u));
    return c[0] + u * (c[1] + u * (c[2] + u * (c[3] + u * (c[4] + u * c[5]))));
}

// Plan a cosine (from rest) or quintic (from the current speed and
// acceleration) segment to the target, as fast as the limits allow.
static void start_segment(motion_axis_t *ax) {
    float d = (float)(ax->target - ax->gen_pos) * STEPS_PER_UNIT_F;
    float dist = fabsf(d), v0 = ax->gen_vel, a0 = ax->gen_acc;
    bool from_rest = fabsf(v0) < 1e-3f && fabsf(a0) < 1e-3f;

    ax->seg_p0 = ax->gen_pos;
    ax->seg_tick = 0;
    ax->seg_corr = 0;
    ax->seg_active = true;
    ax->seg_cosine = ax->profile == PROFILE_COSINE && from_rest;

    // Peak speed and acceleration of each shape from rest, as multiples of
    // d/T and d/T^2: cosine pi/2 and pi^2/2, quintic 15/8 and 10/sqrt(3).
    float kv = ax->seg_cosine ? 0.5f * PI_F : 1.875f;
    float ka = ax->seg_cosine ? 0.5f * PI_F * PI_F : 5.7735f;
    float T = fmaxf(kv * dist / ax->vmax, sqrtf(ka * dist / ax->amax));
    // Moving already: at least time to brake.
    T = fmaxf(T, 2.0f * fabsf(v0) / ax->amax);
    T = fmaxf(T, 2.0f * DT);

    // The distance in double: float would round a long move's target.
    double dd = (double)(ax->target - ax->gen_pos) / (double)MOTION_UNITS_PER_STEP;
    if (ax->seg_cosine) {
        ax->seg_cd[0] = dd;
        ax->seg_c[0] = (float)dd;
        ax->seg_T = T;
        return;
    }
    // From a moving start the closed-form T can violate the limits; stretch
    // until a sampled check passes.
    for (int iter = 0; iter < 24; iter++) {
        set_quintic(ax, dd, v0, a0, T);
        if (from_rest)
            return;
        bool ok = true;
        for (int k = 1; k <= 32 && ok; k++) {
            float x, v, a;
            segment_eval(ax, T * (float)k / 32.0f, &x, &v, &a);
            ok = fabsf(v) <= ax->vmax * 1.001f && fabsf(a) <= ax->amax * 1.001f;
        }
        if (ok)
            return;
        T *= 1.15f;
    }
}

// --- generators ---------------------------------------------------------------

static float clampf(float x, float lo, float hi) {
    return x < lo ? lo : x > hi ? hi : x;
}

// Move v toward v_des by at most dv.
static float approach(float v, float v_des, float dv) {
    if (v < v_des)
        return v + dv > v_des ? v_des : v + dv;
    return v - dv < v_des ? v_des : v - dv;
}

// Advance the raw generator one tick. Returns false once it is at rest with
// nothing left to do.
static bool generate(motion_axis_t *ax) {
    float v = ax->gen_vel;
    float dv = ax->amax * DT;
    float v_new;

    switch (ax->mode) {
    case MODE_IDLE:
        ax->gen_vel = ax->gen_acc = 0.0f;
        return false;

    case MODE_POSITION:
        if (ax->seg_active) {
            float t0 = (float)ax->seg_tick * DT;
            float t1 = (float)++ax->seg_tick * DT;
            if (t1 >= ax->seg_T) {
                ax->gen_pos = ax->target;
                ax->gen_vel = ax->gen_acc = 0.0f;
                ax->seg_active = false;
                ax->mode = MODE_IDLE;
                return false;
            }
            // Advance by this tick's increment (Simpson's rule on v), not by
            // evaluating x(t): float x far from the start would only resolve
            // ~0.06 steps on a million-step move.
            float x, a, vm, v1;
            segment_eval(ax, 0.5f * (t0 + t1), &x, &vm, &a);
            segment_eval(ax, t1, &x, &v1, &a);
            float dx = DT / 6.0f * (v + 4.0f * vm + v1);
            ax->gen_pos += (int64_t)(dx * UNITS_PER_STEP_F) + ax->seg_corr;
            // Float time and rounding drift slowly over long segments:
            // measure against the exact curve now and then, and spread the
            // difference over the next sync interval.
            if (ax->seg_tick % SEG_SYNC_TICKS == 0) {
                double ideal = (double)ax->seg_p0 +
                               segment_x_exact(ax, ax->seg_tick) * (double)MOTION_UNITS_PER_STEP;
                ax->seg_corr = ((int64_t)ideal - ax->gen_pos) / (int64_t)SEG_SYNC_TICKS;
            }
            ax->gen_acc = a;
            ax->gen_vel = v1;
            return true;
        } else {
            float d = (float)(ax->target - ax->gen_pos) * STEPS_PER_UNIT_F;
            // Fastest next speed u that still stops on the target: this
            // tick's travel (v + u)/2*dt plus the braking distance u^2/(2a)
            // must fit in the distance left. Solving that quadratic for u
            // keeps the discrete ramp from overshooting. Speeds here are
            // toward the target.
            float a = ax->amax, dist = fabsf(d);
            float v_toward = d >= 0.0f ? v : -v;
            float h = 0.5f * a * DT;
            float disc = h * h + 2.0f * a * dist - a * v_toward * DT;
            float v_stop = disc > 0.0f ? sqrtf(disc) - h : 0.0f;
            float v_des = copysignf(fminf(v_stop, ax->vmax), d);
            v_new = approach(v, v_des, dv);

            // Arrive: when this tick would reach or cross the target at a
            // speed we could have stopped from in one tick, land exactly.
            float travel = 0.5f * (v + v_new) * DT;
            if (fabsf(d) <= fabsf(travel) + 1e-6f && fabsf(v_new) <= 2.0f * dv) {
                ax->gen_pos = ax->target;
                ax->gen_vel = ax->gen_acc = 0.0f;
                ax->mode = MODE_IDLE;
                return false;
            }
        }
        break;

    case MODE_JOG:
        if (ax->jog_ticks_left > 0)
            ax->jog_ticks_left--;
        else
            ax->cmd_vel = 0.0f;  // jog watchdog: no refresh, stop
        /* fall through */
    case MODE_VELOCITY:
        v_new = approach(v, clampf(ax->cmd_vel, -ax->vmax, ax->vmax), dv);
        if (v_new == 0.0f && ax->cmd_vel == 0.0f) {
            ax->gen_vel = ax->gen_acc = 0.0f;
            ax->mode = MODE_IDLE;
            return false;
        }
        break;

    default:
        return false;
    }

    ax->gen_pos += (int64_t)(0.5f * (v + v_new) * DT * UNITS_PER_STEP_F);
    ax->gen_acc = (v_new - v) * (float)MOTION_TICK_HZ;
    ax->gen_vel = v_new;
    return true;
}

void motion_tick(motion_axis_t *ax) {
    int64_t before = ax->gen_pos;
    bool moving = generate(ax);
    int64_t delta = ax->gen_pos - before;

    uint32_t stages = filter_stages(ax);
    bool flushed = true;
    for (uint32_t s = 0; s < stages; s++) {
        delta = filter_step(&ax->filt[s], delta);
        flushed &= filter_flushed(&ax->filt[s]);
    }

    ax->pos += delta;
    ax->vel = (float)delta * STEPS_PER_UNIT_F * (float)MOTION_TICK_HZ;
    ax->settled = !moving && flushed && ax->mode == MODE_IDLE;
    if (ax->settled)
        ax->vel = 0.0f;
}
