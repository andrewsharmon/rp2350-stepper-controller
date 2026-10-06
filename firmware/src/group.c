#include "group.h"

#include <math.h>
#include <string.h>

#include "trig.h"

#define DT (1.0f / (float)MOTION_TICK_HZ)
#define UNITS_PER_STEP_F ((float)MOTION_UNITS_PER_STEP)
#define STEPS_PER_UNIT_F (1.0f / (float)MOTION_UNITS_PER_STEP)
#define DEFAULT_CORNER_S 0.02f

static group_seg_t *seg_at(group_t *g, uint32_t k) {
    return &g->q[(g->head + k) % GROUP_QUEUE];
}

bool group_create(group_t *g, int8_t id, motion_axis_t *axes, const uint8_t *members,
                  uint32_t n, float feed) {
    if (n == 0 || n > GROUP_MAX_AXES)
        return false;
    for (uint32_t k = 0; k < n; k++) {
        motion_axis_t *ax = &axes[members[k]];
        if (!ax->settled || ax->group >= 0)
            return false;
        for (uint32_t j = 0; j < k; j++)
            if (members[j] == members[k])
                return false;
    }
    memset(g, 0, sizeof *g);
    g->active = true;
    g->n = (uint8_t)n;
    g->feed = feed;
    g->corner_s = DEFAULT_CORNER_S;
    for (uint32_t k = 0; k < n; k++) {
        motion_axis_t *ax = &axes[members[k]];
        g->axis[k] = members[k];
        g->start[k] = g->tail[k] = ax->pos;
        ax->gen_pos = ax->gen_last = ax->pos;
        ax->gen_vel = ax->gen_acc = 0.0f;
        ax->mode = MODE_GROUP;
        ax->group = id;
        ax->ext_moving = false;
    }
    return true;
}

bool group_release(group_t *g, motion_axis_t *axes) {
    if (!g->active || !group_idle(g))
        return false;
    for (uint32_t k = 0; k < g->n; k++) {
        motion_axis_t *ax = &axes[g->axis[k]];
        if (!ax->settled)
            return false;  // filters still flushing
    }
    for (uint32_t k = 0; k < g->n; k++) {
        motion_axis_t *ax = &axes[g->axis[k]];
        ax->group = -1;
        ax->mode = MODE_IDLE;
    }
    g->active = false;
    return true;
}

bool group_idle(const group_t *g) {
    return !g->running && g->count == 0 && !g->arc_active;
}

// Backward pass: every entry speed must still allow stopping by the end of
// the queue, and no faster than its junction allows. The executing
// segment's entry is history, so stop at index 1.
static void plan(group_t *g) {
    float v_next = 0.0f;  // speed at the end of the queue
    for (uint32_t k = g->count; k-- > 1;) {
        group_seg_t *sg = seg_at(g, k);
        float v = sqrtf(v_next * v_next + 2.0f * sg->amax * sg->len);
        sg->v_entry = fminf(sg->v_junction, v);
        v_next = sg->v_entry;
    }
    if (g->count == 1 || (g->count > 0 && !g->running))
        seg_at(g, 0)->v_entry = 0.0f;
}

bool group_line(group_t *g, const motion_axis_t *axes, const int64_t *target, float feed) {
    if (!g->active || g->count >= GROUP_QUEUE)
        return false;
    group_seg_t *sg = &g->q[(g->head + g->count) % GROUP_QUEUE];

    float d[GROUP_MAX_AXES], len2 = 0.0f;
    for (uint32_t k = 0; k < g->n; k++) {
        d[k] = (float)(target[k] - g->tail[k]) * STEPS_PER_UNIT_F;
        len2 += d[k] * d[k];
    }
    if (len2 < 1e-12f)
        return true;  // zero-length: nothing to do

    float len = sqrtf(len2);
    float vmax = feed > 0.0f ? feed : g->feed, amax = INFINITY;
    for (uint32_t k = 0; k < g->n; k++) {
        const motion_axis_t *ax = &axes[g->axis[k]];
        sg->end[k] = target[k];
        sg->u[k] = d[k] / len;
        float au = fabsf(sg->u[k]);
        if (au > 1e-6f) {
            vmax = fminf(vmax, ax->vmax / au);
            amax = fminf(amax, ax->amax / au);
        }
    }
    sg->len = len;
    sg->vmax = vmax;
    sg->amax = amax;

    // Junction with the previous queued segment: limit each axis's speed
    // jump to what it could change in the corner time.
    sg->v_junction = 0.0f;
    if (g->count > 0) {
        const group_seg_t *prev = seg_at(g, g->count - 1);
        float vj = fminf(prev->vmax, vmax);
        for (uint32_t k = 0; k < g->n; k++) {
            float du = fabsf(sg->u[k] - prev->u[k]);
            if (du > 1e-6f)
                vj = fminf(vj, axes[g->axis[k]].amax * g->corner_s / du);
        }
        sg->v_junction = vj;
    }
    memcpy(g->tail, target, sizeof(int64_t) * g->n);
    g->count++;
    plan(g);
    return true;
}

bool group_arc(group_t *g, const int64_t *center, double angle, float feed, float tol) {
    if (!g->active || g->n < 2 || g->arc_active)
        return false;
    float r0x = (float)(g->tail[0] - center[0]) * STEPS_PER_UNIT_F;
    float r0y = (float)(g->tail[1] - center[1]) * STEPS_PER_UNIT_F;
    float r = sqrtf(r0x * r0x + r0y * r0y);
    if (r < 1e-3f || angle == 0.0)
        return true;
    // Chord of angle t sits r(1 - cos(t/2)) ~ r t^2/8 from the circle.
    float t_max = 2.0f * sqrtf(2.0f * tol / r);
    uint32_t n = (uint32_t)ceilf((float)fabs(angle) / t_max);
    if (n < 1)
        n = 1;
    g->arc_c[0] = center[0];
    g->arc_c[1] = center[1];
    g->arc_r0[0] = r0x;
    g->arc_r0[1] = r0y;
    g->arc_step = (float)(angle / (double)n);
    g->arc_angle = angle;
    g->arc_p0[0] = g->tail[0];
    g->arc_p0[1] = g->tail[1];
    g->arc_k = 1;
    g->arc_n = n;
    g->arc_feed = feed;
    g->arc_active = true;
    return true;
}

// Queue the next chord(s) while there is room.
static void feed_arc(group_t *g, const motion_axis_t *axes) {
    while (g->arc_active && g->count < GROUP_QUEUE) {
        int64_t target[GROUP_MAX_AXES];
        memcpy(target, g->tail, sizeof(int64_t) * g->n);
        if (g->arc_k == g->arc_n) {
            // Last chord: end point in double so arcs (and full circles)
            // end exactly where they should.
            double c = trig_cos_d(g->arc_angle), s = trig_sin_d(g->arc_angle);
            double rx = (double)(g->arc_p0[0] - g->arc_c[0]), ry = (double)(g->arc_p0[1] - g->arc_c[1]);
            target[0] = g->arc_c[0] + (int64_t)llround(c * rx - s * ry);
            target[1] = g->arc_c[1] + (int64_t)llround(s * rx + c * ry);
        } else {
            float t = g->arc_step * (float)g->arc_k;
            float c = trig_cosf(t), s = trig_sinf(t);
            target[0] = g->arc_c[0] + (int64_t)((c * g->arc_r0[0] - s * g->arc_r0[1]) * UNITS_PER_STEP_F);
            target[1] = g->arc_c[1] + (int64_t)((s * g->arc_r0[0] + c * g->arc_r0[1]) * UNITS_PER_STEP_F);
        }
        group_line(g, axes, target, g->arc_feed);
        if (++g->arc_k > g->arc_n)
            g->arc_active = false;
    }
}

void group_halt(group_t *g, const motion_axis_t *axes) {
    if (!g->active)
        return;
    g->count = 0;
    g->running = g->hold = g->flush_on_stop = g->arc_active = false;
    g->v = g->s = 0.0f;
    for (uint32_t k = 0; k < g->n; k++)
        g->start[k] = g->tail[k] = axes[g->axis[k]].pos;
}

void group_hold(group_t *g, bool hold) {
    g->hold = hold;
    if (!hold)
        g->flush_on_stop = false;
}

void group_stop(group_t *g) {
    g->hold = true;
    g->flush_on_stop = true;
    g->arc_active = false;
}

// Move v toward v_des by at most dv.
static float approach(float v, float v_des, float dv) {
    if (v < v_des)
        return v + dv > v_des ? v_des : v + dv;
    return v - dv < v_des ? v_des : v - dv;
}

static void set_axes(group_t *g, motion_axis_t *axes, const group_seg_t *sg, float s, bool at_end) {
    for (uint32_t k = 0; k < g->n; k++) {
        motion_axis_t *ax = &axes[g->axis[k]];
        ax->gen_pos = at_end ? sg->end[k] : g->start[k] + (int64_t)(sg->u[k] * s * UNITS_PER_STEP_F);
        ax->gen_vel = sg->u[k] * g->v;
        ax->ext_moving = g->running;
    }
}

void group_tick(group_t *g, motion_axis_t *axes) {
    if (!g->active)
        return;
    feed_arc(g, axes);
    if (!g->running) {
        if (g->count == 0 || g->hold) {
            for (uint32_t k = 0; k < g->n; k++) {
                axes[g->axis[k]].gen_vel = 0.0f;
                axes[g->axis[k]].ext_moving = false;
            }
            return;
        }
        g->running = true;
        g->s = 0.0f;
        g->v = 0.0f;
    }

    group_seg_t *sg = seg_at(g, 0);
    float v = g->v;
    float a = sg->amax;
    float v_exit = g->count > 1 ? seg_at(g, 1)->v_entry : 0.0f;
    float rem = sg->len - g->s;

    // Fastest next speed u that still slows to v_exit by the segment end:
    // (v + u)/2*dt + (u^2 - v_exit^2)/(2a) <= rem.
    float h = 0.5f * a * DT;
    float disc = h * h + 2.0f * a * rem + v_exit * v_exit - a * v * DT;
    // Past the point of a clean stop the root goes negative; path speed
    // never does (that would creep backwards toward the end).
    float v_stop = disc > 0.0f ? fmaxf(sqrtf(disc) - h, 0.0f) : 0.0f;
    float v_des = g->hold ? 0.0f : fminf(v_stop, sg->vmax);
    float u = approach(v, v_des, a * DT);
    float travel = 0.5f * (v + u) * DT;
    g->v = u;

    if (g->hold && u == 0.0f && travel <= 0.0f) {
        // Held at rest. A stop drops the rest of the queue here.
        if (g->flush_on_stop) {
            for (uint32_t k = 0; k < g->n; k++)
                g->tail[k] = axes[g->axis[k]].gen_pos;
            g->count = 0;
            g->running = false;
            g->hold = g->flush_on_stop = false;
        }
        set_axes(g, axes, sg, g->s, false);
        for (uint32_t k = 0; k < g->n; k++)
            axes[g->axis[k]].ext_moving = false;
        return;
    }

    g->s += travel;
    // Last segment: arrive exactly when this tick reaches the end at a
    // speed we could have stopped from (mirrors the single-axis law). The
    // 1e-3 step slack covers float resolution of s along long segments.
    bool last = g->count == 1;
    bool arrive = g->s >= sg->len - 1e-6f || (last && rem <= travel + 1e-3f && u <= 2.0f * a * DT);
    while (arrive) {
        float carry = g->s - sg->len;
        memcpy(g->start, sg->end, sizeof(int64_t) * g->n);
        g->head = (g->head + 1) % GROUP_QUEUE;
        g->count--;
        g->segments_done++;
        if (g->count == 0) {
            g->running = false;
            g->v = 0.0f;
            g->s = 0.0f;
            // A stop whose queue ran out while braking is complete.
            if (g->flush_on_stop)
                g->hold = g->flush_on_stop = false;
            set_axes(g, axes, sg, 0.0f, true);
            return;
        }
        sg = seg_at(g, 0);
        g->s = carry > 0.0f ? carry : 0.0f;
        arrive = g->s >= sg->len - 1e-6f;
    }
    set_axes(g, axes, sg, g->s, false);
}
