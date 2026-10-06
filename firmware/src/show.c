#include "show.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define AXIS_KEY_SIZE 12
#define LED_KEY_SIZE  8

static uint32_t rd32(const uint8_t *p) {
    return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}

static float rdf(const uint8_t *p) {
    uint32_t u = rd32(p);
    float f;
    memcpy(&f, &u, 4);
    return f;
}

uint32_t show_crc32(const uint8_t *blob, uint32_t len) {
    uint32_t crc = 0xffffffffu;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= i >= 12 && i < 16 ? 0 : blob[i];  // the crc field counts as zero
        for (int b = 0; b < 8; b++)
            crc = crc & 1 ? crc >> 1 ^ 0xedb88320u : crc >> 1;
    }
    return ~crc;
}

static uint32_t key_t_ms(const show_track_t *tr, uint32_t k) {
    return rd32(tr->keys + k * (tr->type == SHOW_TRACK_AXIS ? AXIS_KEY_SIZE : LED_KEY_SIZE));
}

const char *show_parse(show_t *s, const uint8_t *blob, uint32_t len, uint32_t n_axes,
                       uint32_t n_pixels) {
    memset(s, 0, sizeof *s);
    if (len < SHOW_HEADER_SIZE || len > SHOW_MAX_SIZE)
        return "bad size";
    if (rd32(blob) != SHOW_MAGIC)
        return "not a show";
    if ((blob[4] | blob[5] << 8) != SHOW_VERSION || (blob[6] | blob[7] << 8) != SHOW_HEADER_SIZE)
        return "unsupported show version";
    if (rd32(blob + 8) != len)
        return "length mismatch";
    if (rd32(blob + 12) != show_crc32(blob, len))
        return "CRC mismatch";
    s->blob = blob;
    s->len = len;
    memcpy(s->name, blob + 16, SHOW_NAME_LEN);
    uint32_t duration = rd32(blob + 32);
    s->loop = blob[36] & SHOW_FLAG_LOOP;
    s->n_tracks = blob[37];
    if (s->n_tracks > SHOW_MAX_TRACKS)
        return "too many tracks";

    uint32_t off = SHOW_HEADER_SIZE, last_t = 0;
    for (uint32_t t = 0; t < s->n_tracks; t++) {
        if (off + 4 > len)
            return "truncated track";
        show_track_t *tr = &s->track[t];
        tr->type = blob[off];
        tr->channel = blob[off + 1];
        tr->n_keys = (uint16_t)(blob[off + 2] | blob[off + 3] << 8);
        tr->keys = blob + off + 4;
        uint32_t ksize = tr->type == SHOW_TRACK_AXIS ? AXIS_KEY_SIZE : tr->type == SHOW_TRACK_LED ? LED_KEY_SIZE : 0;
        if (!ksize)
            return "unknown track type";
        if (tr->type == SHOW_TRACK_AXIS ? tr->channel >= n_axes : tr->channel >= n_pixels)
            return "track channel out of range";
        if (tr->n_keys == 0)
            return "empty track";
        off += 4 + ksize * tr->n_keys;
        if (off > len)
            return "truncated keys";
        for (uint32_t k = 0; k < tr->n_keys; k++) {
            uint32_t tk = key_t_ms(tr, k);
            if (k > 0 && tk <= key_t_ms(tr, k - 1))
                return "key times must increase";
            if (tr->type == SHOW_TRACK_AXIS && !isfinite(rdf(tr->keys + k * AXIS_KEY_SIZE + 4)))
                return "axis key position is not a number";
            if (tk > last_t)
                last_t = tk;
        }
        for (uint32_t u = 0; u < t; u++)
            if (s->track[u].type == tr->type && s->track[u].channel == tr->channel)
                return "two tracks for the same channel";
    }
    if (off != len)
        return "trailing bytes";
    if (s->loop && duration <= last_t)
        return "a looping show needs a duration after its last key";
    s->duration_ms = duration ? duration : last_t;
    return NULL;
}

// --- axis tracks ---------------------------------------------------------------

static float key_pos(const show_track_t *tr, uint32_t k) {
    return rdf(tr->keys + k * AXIS_KEY_SIZE + 4);
}

// Time and position of key k in cycle c, where k may run one past either
// end (wrapping into the neighbouring cycle for a looping show).
static void neighbour(const show_t *s, const show_track_t *tr, int64_t k, int64_t c,
                      double *t, float *p) {
    int64_t n = tr->n_keys;
    while (k < 0) { k += n; c--; }
    while (k >= n) { k -= n; c++; }
    *t = (double)key_t_ms(tr, (uint32_t)k) + (double)c * s->duration_ms;
    *p = key_pos(tr, (uint32_t)k);
}

show_point_t show_axis_key(const show_t *s, const show_track_t *tr, uint32_t k, uint32_t cycle) {
    show_point_t pt;
    pt.t_ms = key_t_ms(tr, k) + cycle * s->duration_ms;
    pt.pos = key_pos(tr, k);
    pt.vel = rdf(tr->keys + k * AXIS_KEY_SIZE + 8);
    if (isnan(pt.vel)) {
        bool end = !s->loop && (k == 0 || k + 1 == tr->n_keys);
        if (end || tr->n_keys < 2) {
            pt.vel = 0.0f;
        } else {
            double t0, t1;
            float p0, p1;
            neighbour(s, tr, (int64_t)k - 1, cycle, &t0, &p0);
            neighbour(s, tr, (int64_t)k + 1, cycle, &t1, &p1);
            pt.vel = (float)((p1 - p0) / ((t1 - t0) / 1000.0));
        }
    }
    return pt;
}

// The stream starts at show time 0 from the first key's position; a key
// at t = 0 is that start, not a point.
static uint32_t skip_first(const show_track_t *tr) {
    return key_t_ms(tr, 0) == 0 ? 1 : 0;
}

uint32_t show_axis_points(const show_t *s, const show_track_t *tr) {
    return s->loop ? tr->n_keys : tr->n_keys - skip_first(tr);
}

show_point_t show_axis_point(const show_t *s, const show_track_t *tr, uint64_t i) {
    uint64_t j = i + skip_first(tr);
    uint32_t k = (uint32_t)(j % tr->n_keys), c = (uint32_t)(j / tr->n_keys);
    return show_axis_key(s, tr, k, s->loop ? c : 0);
}

// --- LED tracks ----------------------------------------------------------------

static void led_key(const show_track_t *tr, uint32_t k, uint8_t rgb[3]) {
    const uint8_t *p = tr->keys + k * LED_KEY_SIZE + 4;
    rgb[0] = p[0];
    rgb[1] = p[1];
    rgb[2] = p[2];
}

void show_led_at(const show_t *s, const show_track_t *tr, uint32_t t_ms, uint8_t rgb[3]) {
    uint32_t n = tr->n_keys;
    if (s->loop && s->duration_ms)
        t_ms %= s->duration_ms;
    uint32_t t_first = key_t_ms(tr, 0), t_last = key_t_ms(tr, n - 1);
    uint8_t a[3], b[3];
    uint32_t ta, tb;
    if (t_ms >= t_first && t_ms < t_last) {
        uint32_t k = 0;
        while (key_t_ms(tr, k + 1) <= t_ms)
            k++;
        led_key(tr, k, a);
        led_key(tr, k + 1, b);
        ta = key_t_ms(tr, k);
        tb = key_t_ms(tr, k + 1);
    } else if (!s->loop || n == 1) {
        led_key(tr, t_ms < t_first ? 0 : n - 1, rgb);
        return;
    } else {
        // Across the loop seam: last key -> first key of the next cycle.
        led_key(tr, n - 1, a);
        led_key(tr, 0, b);
        ta = t_last;
        tb = t_first + s->duration_ms;
        if (t_ms < t_first)
            t_ms += s->duration_ms;
    }
    float u = (float)(t_ms - ta) / (float)(tb - ta);
    for (int c = 0; c < 3; c++)
        rgb[c] = (uint8_t)((float)a[c] + ((float)b[c] - (float)a[c]) * u + 0.5f);
}

// --- limits ----------------------------------------------------------------------

const char *show_check_limits(const show_t *s, const float *vmax, const float *amax,
                              char *msg, uint32_t msg_len) {
    for (uint32_t t = 0; t < s->n_tracks; t++) {
        const show_track_t *tr = &s->track[t];
        if (tr->type != SHOW_TRACK_AXIS)
            continue;
        uint32_t ch = tr->channel;
        // One pass, plus the seam into the next cycle for a looping show.
        show_point_t prev = {0, key_pos(tr, 0), 0.0f};
        uint32_t n = show_axis_points(s, tr) + (s->loop ? 1 : 0);
        for (uint32_t i = 0; i < n; i++) {
            show_point_t pt = show_axis_point(s, tr, i);  // starts from rest at key 0
            double T = (pt.t_ms - prev.t_ms) / 1000.0;
            double d = pt.pos - prev.pos, v0T = prev.vel * T, v1T = pt.vel * T;
            double c0 = v0T, c1 = 3 * d - 2 * v0T - v1T, c2 = -2 * d + v0T + v1T;
            // v(u) = (c0 + 2 c1 u + 3 c2 u^2)/T: ends or the vertex.
            double vpk = fmax(fabs(c0), fabs(c0 + 2 * c1 + 3 * c2));
            if (c2 != 0.0) {
                double u = -c1 / (3 * c2);
                if (u > 0 && u < 1)
                    vpk = fmax(vpk, fabs(c0 + 2 * c1 * u + 3 * c2 * u * u));
            }
            vpk /= T;
            // a(u) = (2 c1 + 6 c2 u)/T^2: linear, so the ends.
            double apk = fmax(fabs(2 * c1), fabs(2 * c1 + 6 * c2)) / (T * T);
            if (vpk > vmax[ch] * 1.001 || apk > amax[ch] * 1.01) {
                snprintf(msg, msg_len, "axis %lu at %lu-%lu ms needs %.0f steps/s, %.0f steps/s^2 "
                         "(limits %.0f, %.0f)", (unsigned long)ch + 1, (unsigned long)prev.t_ms,
                         (unsigned long)pt.t_ms, vpk, apk, (double)vmax[ch], (double)amax[ch]);
                return msg;
            }
            prev = pt;
        }
    }
    return NULL;
}
