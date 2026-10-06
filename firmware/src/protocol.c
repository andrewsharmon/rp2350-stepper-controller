#include "protocol.h"

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"

#include "adc_monitor.h"
#include "console.h"
#include "frame.h"
#include "hbridge.h"
#include "led.h"
#include "player.h"
#include "show_store.h"

#define FIRMWARE_NAME "rp2350-stepper"

// --- input demux ----------------------------------------------------------------

static uint8_t rx[FRAME_MAX_WIRE];
static uint32_t rx_len;
static bool in_frame;

static void handle_frame(const uint8_t *buf, uint32_t len);

void protocol_input(int c) {
    if (c == 0) {
        // A zero opens a frame; the next zero closes it.
        if (in_frame && rx_len > 0) {
            handle_frame(rx, rx_len);
            in_frame = false;
        } else {
            in_frame = true;
        }
        rx_len = 0;
        return;
    }
    if (!in_frame) {
        console_input(c);
        return;
    }
    if (rx_len < sizeof rx)
        rx[rx_len++] = (uint8_t)c;
    else
        in_frame = false;  // oversize: drop it and go back to text
}

// --- little-endian helpers -------------------------------------------------------

typedef struct {
    const uint8_t *p;
    int left;
    bool ok;
} reader_t;

static const uint8_t *take(reader_t *r, int n) {
    if (r->left < n) {
        r->ok = false;
        static const uint8_t zeros[8];
        return zeros;
    }
    const uint8_t *p = r->p;
    r->p += n;
    r->left -= n;
    return p;
}

static uint8_t rd_u8(reader_t *r) { return *take(r, 1); }
static uint16_t rd_u16(reader_t *r) { const uint8_t *p = take(r, 2); return (uint16_t)(p[0] | p[1] << 8); }
static uint32_t rd_u32(reader_t *r) {
    const uint8_t *p = take(r, 4);
    return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}
static int64_t rd_i64(reader_t *r) {
    uint64_t lo = rd_u32(r), hi = rd_u32(r);
    return (int64_t)(lo | hi << 32);
}
static float rd_f32(reader_t *r) {
    uint32_t u = rd_u32(r);
    float f;
    memcpy(&f, &u, 4);
    return f;
}
static double rd_f64(reader_t *r) {
    uint64_t lo = rd_u32(r), hi = rd_u32(r);
    uint64_t u = lo | hi << 32;
    double d;
    memcpy(&d, &u, 8);
    return d;
}

typedef struct {
    uint8_t buf[FRAME_MAX_PAYLOAD];
    int len;
} writer_t;

static void put(writer_t *w, const void *v, int n) {
    if (w->len + n <= (int)sizeof w->buf) {
        memcpy(w->buf + w->len, v, (size_t)n);  // the target is little-endian
        w->len += n;
    }
}
static void wr_u8(writer_t *w, uint8_t v) { put(w, &v, 1); }
static void wr_u16(writer_t *w, uint16_t v) { put(w, &v, 2); }
static void wr_u32(writer_t *w, uint32_t v) { put(w, &v, 4); }
static void wr_i64(writer_t *w, int64_t v) { put(w, &v, 8); }
static void wr_f32(writer_t *w, float v) { put(w, &v, 4); }

// When set, replies are captured here instead of sent over USB
// (protocol_request). Telemetry and events always go to USB.
static uint8_t *capture;
static size_t capture_max, capture_len;

static void send(uint8_t type, uint16_t seq, const writer_t *w) {
    if (capture) {
        if ((size_t)w->len + 1 <= capture_max) {
            capture[0] = type;
            memcpy(capture + 1, w->buf, (size_t)w->len);
            capture_len = (size_t)w->len + 1;
        }
        return;
    }
    uint8_t wire[FRAME_MAX_WIRE];
    size_t n = frame_build(type, seq, w->buf, (size_t)w->len, wire);
    // Raw: the text path turns \n into \r\n, which would corrupt a frame.
    fflush(stdout);
    stdio_put_string((const char *)wire, (int)n, false, false);
}

static void ack(uint16_t seq, uint8_t req, uint8_t result, uint32_t cmd_seq) {
    writer_t w = {0};
    wr_u8(&w, req);
    wr_u8(&w, result);
    wr_u32(&w, cmd_seq);
    send(PROTO_ACK, seq, &w);
}

// --- replies ------------------------------------------------------------------------

static void send_pong(uint16_t seq) {
    writer_t w = {0};
    wr_u8(&w, PROTO_VERSION);
    wr_u8(&w, NUM_MOTORS);
    wr_u8(&w, GROUP_COUNT);
    wr_u16(&w, MOTION_TICK_HZ);
    wr_u16(&w, HBRIDGE_PWM_HZ);
    wr_u8(&w, 30);
    put(&w, FIRMWARE_NAME, (int)strlen(FIRMWARE_NAME));
    send(PROTO_PONG, seq, &w);
}

static void send_status(uint16_t seq) {
    static control_snapshot_t s;  // ~1 KB: off core 0's stack
    control_snapshot(&s);
    writer_t w = {0};
    bool on = control_outputs_on;
    wr_u32(&w, s.tick);
    wr_u8(&w, (uint8_t)((on ? 1 : 0) |
                        (!on && control_ladder.stop_cause == LADDER_ESTOP ? 2 : 0) |
                        (!on && control_ladder.stop_cause == LADDER_FAULT ? 4 : 0)));
    wr_u32(&w, s.last_seq);
    wr_u32(&w, s.rejected);
    wr_u32(&w, s.pvt_underruns);
    wr_u16(&w, (uint16_t)adc_monitor_vmot_mv());
    wr_u16(&w, (uint16_t)adc_monitor_pin_mv(MON_LADDER));
    for (int i = 0; i < NUM_MOTORS; i++) {
        wr_i64(&w, s.pos[i]);
        wr_f32(&w, s.vel[i]);
        wr_u8(&w, s.mode[i]);
        wr_u8(&w, (uint8_t)(s.amp[i] * 100.0f + 0.5f));
        wr_u8(&w, (uint8_t)(((s.holding_mask >> i) & 1) | ((s.settled_mask >> i) & 1) << 1));
        wr_u8(&w, s.pvt_depth[i]);
        int8_t grp = -1;
        for (int k = 0; k < GROUP_COUNT; k++)
            if (s.group[k].active && (s.group[k].members & (1u << i)))
                grp = (int8_t)k;
        wr_u8(&w, (uint8_t)grp);
    }
    for (int k = 0; k < GROUP_COUNT; k++) {
        wr_u8(&w, (uint8_t)(s.group[k].active | s.group[k].running << 1 | s.group[k].hold << 2 |
                            s.group[k].arc << 3));
        wr_u16(&w, s.group[k].members);
        wr_u8(&w, s.group[k].queued);
        wr_f32(&w, s.group[k].v);
        wr_u32(&w, s.group[k].segments_done);
    }
    send(PROTO_STATUS, seq, &w);
}

static void send_telem(uint8_t type, const writer_t *w) {
    uint8_t *saved = capture;  // never captured: always to USB
    capture = NULL;
    send(type, 0, w);
    capture = saved;
}

void protocol_send_telemetry(const control_snapshot_t *s, uint32_t line, uint32_t axes,
                             uint32_t fields, uint32_t sys) {
    writer_t w = {0};
    wr_u32(&w, line);
    wr_u32(&w, s->tick);
    if (sys & 1)
        wr_u32(&w, s->last_seq);
    if (sys & 2)
        wr_u16(&w, (uint16_t)adc_monitor_vmot_mv());
    if (sys & 4)
        wr_u16(&w, (uint16_t)adc_monitor_pin_mv(MON_LADDER));
    if (sys & 8)
        wr_u8(&w, control_outputs_on ? 0 : control_ladder.stop_cause == LADDER_FAULT ? 2 : 1);
    for (int i = 0; i < NUM_MOTORS; i++) {
        if (!(axes & (1u << i)))
            continue;
        if (fields & 1)
            wr_i64(&w, s->pos[i]);
        if (fields & 2)
            wr_f32(&w, s->vel[i]);
        if (fields & 4)
            wr_u8(&w, s->mode[i]);
        if (fields & 8)
            wr_u8(&w, (uint8_t)(s->amp[i] * 100.0f + 0.5f));
        if (fields & 16)
            wr_u8(&w, s->pvt_depth[i]);
    }
    send_telem(PROTO_TELEM, &w);
}

void protocol_send_event(uint32_t tick, uint8_t kind, uint8_t axis, int64_t pos) {
    writer_t w = {0};
    wr_u32(&w, tick);
    wr_u8(&w, kind);
    wr_u8(&w, axis);
    wr_i64(&w, pos);
    send_telem(PROTO_EVENT, &w);
}

// --- shows -------------------------------------------------------------------------

static uint8_t upload[SHOW_MAX_SIZE];   // padded with 0xff to whole flash pages
static uint32_t upload_len, upload_slot = SHOW_SLOTS;

static void send_show_info(uint16_t seq) {
    writer_t w = {0};
    wr_u8(&w, player_slot() < 0 ? 0xff : (uint8_t)player_slot());
    for (uint32_t k = 0; k < SHOW_SLOTS; k++) {
        uint32_t len;
        const uint8_t *blob = show_store_blob(k, &len);
        show_t s;
        bool valid = blob && !show_parse(&s, blob, len, NUM_MOTORS, LED_CHAIN);
        wr_u8(&w, valid);
        char name[SHOW_NAME_LEN] = {0};
        if (valid)
            memcpy(name, s.name, SHOW_NAME_LEN);
        put(&w, name, SHOW_NAME_LEN);
        wr_u32(&w, valid ? s.duration_ms : 0);
        wr_u8(&w, valid && s.loop);
        wr_u8(&w, valid ? (uint8_t)s.n_tracks : 0);
    }
    send(PROTO_SHOW_INFO, seq, &w);
}

// Flash writes pause core 1: only with every axis at rest.
static bool flash_write_ok(void) {
    static control_snapshot_t s;  // ~1 KB: off core 0's stack
    control_snapshot(&s);
    return player_slot() < 0 && (!control_outputs_on || s.settled_mask == CONTROL_ALL_AXES);
}

static bool store(uint32_t slot, const uint8_t *data, uint32_t len) {
    if (!flash_write_ok())
        return false;
    control_flash_busy = true;
    bool ok = show_store_write(slot, data, len);
    control_flash_busy = false;
    return ok;
}

// --- cams --------------------------------------------------------------------------

static float cam_x[CAM_TABLES][CAM_MAX_POINTS], cam_y[CAM_TABLES][CAM_MAX_POINTS];

static void send_cam_info(uint16_t seq) {
    static control_snapshot_t s;  // ~1 KB: off core 0's stack
    control_snapshot(&s);
    writer_t w = {0};
    wr_u8(&w, s.cams_loaded);
    for (int i = 0; i < NUM_MOTORS; i++) {
        wr_u8(&w, (uint8_t)s.cam_table[i]);
        wr_u8(&w, s.cam_leader[i]);
    }
    for (int k = 0; k < VIRTUAL_LEADERS; k++) {
        wr_i64(&w, s.vlead[k].pos);
        wr_f32(&w, s.vlead[k].vel);
        wr_f32(&w, s.vlead[k].vmax);
        wr_f32(&w, s.vlead[k].amax);
        wr_u8(&w, s.vlead[k].mode);
    }
    send(PROTO_CAM_INFO, seq, &w);
}

// --- requests -----------------------------------------------------------------------

static void dispatch(uint8_t type, uint16_t seq, const uint8_t *payload, int n);

static void handle_frame(const uint8_t *buf, uint32_t len) {
    uint8_t type, payload[FRAME_MAX_PAYLOAD];
    uint16_t seq;
    int n = frame_parse(buf, len, &type, &seq, payload);
    if (n < 0)
        return;  // corrupt: the host times out and retries
    dispatch(type, seq, payload, n);
}

size_t protocol_request(uint8_t type, const uint8_t *payload, size_t len,
                        uint8_t *reply, size_t reply_max) {
    if (len > FRAME_MAX_PAYLOAD)
        return 0;
    capture = reply;
    capture_max = reply_max;
    capture_len = 0;
    dispatch(type, 0, payload, (int)len);
    capture = NULL;
    return capture_len;
}

static void dispatch(uint8_t type, uint16_t seq, const uint8_t *payload, int n) {
    reader_t r = {payload, n, true};
    control_cmd_t c = {0};
    bool post_cmd = true;

    switch (type) {
    case PROTO_PING:
        send_pong(seq);
        return;
    case PROTO_STATUS_REQ:
        send_status(seq);
        return;
    case PROTO_MOVE:
    case PROTO_MOVE_REL:
    case PROTO_SET_POS:
        c.type = type == PROTO_MOVE ? CMD_MOVE : type == PROTO_MOVE_REL ? CMD_MOVE_REL : CMD_SET_POS;
        c.axes = rd_u16(&r);
        c.pos = rd_i64(&r);
        break;
    case PROTO_VELOCITY:
        c.type = CMD_VELOCITY;
        c.axes = rd_u16(&r);
        c.f1 = rd_f32(&r);
        break;
    case PROTO_JOG:
        c.type = CMD_JOG;
        c.axes = rd_u16(&r);
        c.f1 = rd_f32(&r);
        c.ms = rd_u16(&r);
        break;
    case PROTO_STOP:
    case PROTO_PVT_START:
        c.type = type == PROTO_STOP ? CMD_STOP : CMD_PVT_START;
        c.axes = rd_u16(&r);
        break;
    case PROTO_LIMITS:
        c.type = CMD_LIMITS;
        c.axes = rd_u16(&r);
        c.f1 = rd_f32(&r);
        c.f2 = rd_f32(&r);
        break;
    case PROTO_PROFILE:
        c.type = CMD_PROFILE;
        c.axes = rd_u16(&r);
        c.ms = rd_u8(&r);
        c.f1 = (float)rd_u16(&r);
        r.ok &= c.ms < PROFILE_COUNT;
        break;
    case PROTO_PVT_POINT:
        c.type = CMD_PVT_POINT;
        c.axes = rd_u16(&r);
        c.pos = rd_i64(&r);
        c.f1 = rd_f32(&r);
        c.ms = rd_u16(&r);
        r.ok &= c.ms >= 1;
        break;
    case PROTO_GROUP_CREATE:
        c.type = CMD_GROUP_CREATE;
        c.ms = rd_u8(&r);
        c.axes = rd_u16(&r);
        c.f1 = rd_f32(&r);
        c.f2 = rd_f32(&r);
        break;
    case PROTO_GROUP_RELEASE:
    case PROTO_GROUP_STOP:
        c.type = type == PROTO_GROUP_RELEASE ? CMD_GROUP_RELEASE : CMD_GROUP_STOP;
        c.ms = rd_u8(&r);
        break;
    case PROTO_GROUP_LINE:
        c.type = CMD_GROUP_LINE;
        c.ms = rd_u8(&r);
        c.f1 = rd_f32(&r);
        c.n = rd_u8(&r);
        r.ok &= c.n >= 1 && c.n <= GROUP_MAX_AXES;
        for (int k = 0; r.ok && k < c.n; k++)
            c.vec[k] = rd_i64(&r);
        break;
    case PROTO_GROUP_ARC:
        c.type = CMD_GROUP_ARC;
        c.ms = rd_u8(&r);
        c.f1 = rd_f32(&r);
        c.f2 = rd_f32(&r);
        c.vec[0] = rd_i64(&r);
        c.vec[1] = rd_i64(&r);
        c.d1 = rd_f64(&r);
        break;
    case PROTO_GROUP_HOLD:
        c.type = CMD_GROUP_HOLD;
        c.ms = rd_u8(&r);
        c.f1 = rd_u8(&r) ? 1.0f : 0.0f;
        break;
    case PROTO_AMPLITUDE: {
        float a = rd_f32(&r);
        r.ok &= a <= 1.0f;
        if (r.ok)
            control_manual_amp = a < 0.0f ? -1.0f : a;
        post_cmd = false;
        break;
    }
    case PROTO_CLEAR_STOP:
        control_clear_request = true;
        post_cmd = false;
        break;
    case PROTO_CONFIG_SAVE:
        r.ok &= app_save_config() == NULL;
        post_cmd = false;
        break;
    case PROTO_BOOT_SHOW: {
        uint8_t slot = rd_u8(&r);
        r.ok &= slot < SHOW_SLOTS || slot == 0xff;
        if (r.ok)
            app_set_boot_show(slot == 0xff ? -1 : slot);
        post_cmd = false;
        break;
    }
    case PROTO_DRIVE:
        c.type = CMD_DRIVE;
        c.axes = rd_u16(&r);
        c.drive.amp_low = rd_f32(&r);
        c.drive.amp_high = rd_f32(&r);
        c.drive.amp_hold = rd_f32(&r);
        c.drive.low_speed = rd_f32(&r);
        c.drive.high_speed = rd_f32(&r);
        c.drive.flags = rd_u8(&r);
        r.ok &= c.drive.amp_low >= 0.0f && c.drive.amp_low <= 1.0f && c.drive.amp_high >= 0.0f &&
                c.drive.amp_high <= 1.0f && c.drive.amp_hold >= 0.0f && c.drive.amp_hold <= 1.0f;
        break;
    case PROTO_SHOW_BEGIN:
        upload_slot = rd_u8(&r);
        upload_len = rd_u32(&r);
        r.ok &= upload_slot < SHOW_SLOTS && upload_len <= SHOW_MAX_SIZE;
        if (r.ok)
            memset(upload, 0xff, sizeof upload);
        post_cmd = false;
        break;
    case PROTO_SHOW_DATA: {
        uint32_t off = rd_u32(&r);
        r.ok &= upload_slot < SHOW_SLOTS && off + (uint32_t)r.left <= upload_len;
        if (r.ok) {
            memcpy(upload + off, r.p, (size_t)r.left);
            r.left = 0;
        }
        post_cmd = false;
        break;
    }
    case PROTO_SHOW_END: {
        show_t s;
        r.ok &= upload_slot < SHOW_SLOTS &&
                show_parse(&s, upload, upload_len, NUM_MOTORS, LED_CHAIN) == NULL &&
                store(upload_slot, upload, upload_len);
        upload_slot = SHOW_SLOTS;
        post_cmd = false;
        break;
    }
    case PROTO_SHOW_RUN: {
        uint8_t slot = rd_u8(&r);
        r.ok &= r.left == 0 && player_start(slot) == NULL;
        app_set_stress(false);
        post_cmd = false;
        break;
    }
    case PROTO_SHOW_STOP:
        player_stop();
        post_cmd = false;
        break;
    case PROTO_SHOW_LIST:
        send_show_info(seq);
        return;
    case PROTO_SHOW_ERASE: {
        uint8_t slot = rd_u8(&r);
        r.ok &= slot < SHOW_SLOTS && r.left == 0 && store(slot, upload, 0);
        post_cmd = false;
        break;
    }
    case PROTO_CAM_POINTS: {
        uint8_t t = rd_u8(&r);
        uint16_t off = rd_u16(&r);
        uint8_t cnt = rd_u8(&r);
        r.ok &= t < CAM_TABLES && off + cnt <= CAM_MAX_POINTS && r.left == cnt * 8;
        for (int k = 0; r.ok && k < cnt; k++) {
            cam_x[t][off + k] = rd_f32(&r);
            cam_y[t][off + k] = rd_f32(&r);
        }
        post_cmd = false;
        break;
    }
    case PROTO_CAM_LOAD: {
        uint8_t t = rd_u8(&r), cyclic = rd_u8(&r);
        uint16_t cnt = rd_u16(&r);
        r.ok &= t < CAM_TABLES && cnt <= CAM_MAX_POINTS && r.left == 0 &&
                control_cam_load(t, cam_x[t], cam_y[t], cnt, cyclic != 0) == NULL;
        post_cmd = false;
        break;
    }
    case PROTO_CAM_ENGAGE:
        c.type = CMD_CAM_ENGAGE;
        c.axes = rd_u16(&r);
        c.ms = rd_u8(&r);
        c.n = rd_u8(&r);
        c.pos = rd_i64(&r);
        c.f1 = (float)rd_u16(&r);
        r.ok &= r.left == 0 && control_cam_engage_check(c.axes, c.ms, c.n) == NULL;
        break;
    case PROTO_VLEADER:
        c.type = CMD_VLEADER;
        c.ms = rd_u8(&r);
        c.n = rd_u8(&r);
        c.f1 = rd_f32(&r);
        c.f2 = rd_f32(&r);
        c.pos = rd_i64(&r);
        r.ok &= c.ms < VIRTUAL_LEADERS && c.n <= VL_ZERO;
        break;
    case PROTO_CAM_STATUS:
        send_cam_info(seq);
        return;
    case PROTO_TELEMETRY: {
        uint16_t hz = rd_u16(&r);
        uint16_t axes = rd_u16(&r);
        uint8_t fields = rd_u8(&r), sys = rd_u8(&r), events = rd_u8(&r);
        r.ok &= hz <= 1000;
        if (r.ok)
            console_set_telemetry(hz, axes ? axes : CONTROL_ALL_AXES, fields, sys, events != 0, true);
        post_cmd = false;
        break;
    }
    default:
        ack(seq, type, PROTO_UNKNOWN, 0);
        return;
    }

    if (!r.ok || r.left != 0) {
        ack(seq, type, PROTO_BAD_REQUEST, 0);
        return;
    }
    if (!post_cmd) {
        ack(seq, type, PROTO_OK, 0);
        return;
    }
    if (c.type != CMD_LIMITS && c.type != CMD_PROFILE && c.type != CMD_SET_POS && c.type != CMD_DRIVE)
        app_set_stress(false);  // host motion takes over from the stress test
    if (!control_is_group_cmd(c.type) && c.type != CMD_VLEADER &&
        (c.axes == 0 || c.axes & ~CONTROL_ALL_AXES)) {
        ack(seq, type, PROTO_BAD_REQUEST, 0);
        return;
    }
    uint32_t cmd_seq = control_post(&c);
    ack(seq, type, cmd_seq ? PROTO_OK : PROTO_QUEUE_FULL, cmd_seq);
}
