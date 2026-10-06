#include "console.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/watchdog.h"

#include "adc_monitor.h"
#include "control.h"
#include "microstep.h"
#include "protocol.h"

#define LINE_MAX      160
#define MAX_TELEM_HZ  1000  // one line per motion tick
#define JOG_TIMEOUT_MS_DEFAULT 300
#define GROUP_DEFAULT_FEED     1500.0f

static char line[LINE_MAX];
static uint32_t line_len;
static uint32_t telem_period_us;  // 0 = off
static uint32_t telem_last_us;
static uint32_t telem_line;       // line counter, lets a host spot drops
static bool telem_header_due;
static bool events_on;
static bool telem_binary;  // send telemetry and events as protocol frames

// Telemetry contents: per-axis fields for the selected axes, then system
// fields. Each line: T <line> <tick> <system fields> <axis fields...>.
enum { TF_POS = 1, TF_VEL = 2, TF_MODE = 4, TF_AMP = 8, TF_Q = 16 };
enum { TS_SEQ = 1, TS_VMOT = 2, TS_LADDER = 4, TS_STOP = 8 };
static const char *const axis_field_names[] = {"pos", "vel", "mode", "amp", "q"};
static const char *const sys_field_names[] = {"seq", "vmot", "ladder", "stop"};
static uint32_t telem_axes = CONTROL_ALL_AXES;
static uint32_t telem_fields = TF_POS | TF_VEL;
static uint32_t telem_sys;
static bool echo = true;  // off for host tools: no echo, no prompt

static const char help_text[] =
    "axes: 1-10, a list like 1,3,5, or * for all. Positions in full steps.\n"
    "  m <ax> <pos>          move to absolute position\n"
    "  mr <ax> <delta>       move relative\n"
    "  v <ax> <vel>          velocity mode (full steps/s)\n"
    "  j <ax> <vel> [ms]     jog: stops by itself unless repeated within ms (default 300)\n"
    "  s [ax]                stop (decelerate); all axes if none given\n"
    "  z <ax> [pos]          set current position (default 0; axis must be at rest)\n"
    "  lim <ax> <vmax> [amax]  speed / acceleration limits\n"
    "  p <ax> <pos> <vel> <ms>  queue a PVT point: reach pos at vel, ms after the last\n"
    "  pgo <ax>              start following queued PVT points (axes at rest; same tick)\n"
    "  g <id> <ax> [feed] [corner_ms]  group 1-4 over axes (at rest); g <id> off releases\n"
    "  gl <id> <pos...> [f<feed>]       group line: one absolute position per member\n"
    "  ga <id> <cx> <cy> <deg> [f<feed>] [t<tol>]  arc on the first two members\n"
    "  gh <id> / gr <id> / gs <id>      hold / resume / stop (stop drops the queue)\n"
    "  prof <ax> <name> [ms] motion profile: trap, scurve, smooth, cosine, quintic;\n"
    "                        ms = jerk time for scurve/smooth (default 30, max 100)\n"
    "  amp auto|<percent>    drive amplitude: automatic curve, or fixed\n"
    "  drive <ax> <low%> <high%> <hold%> [low_spd high_spd]  automatic amplitude curve\n"
    "  dir <ax> fwd|rev      reverse rotation      coils <ax> ab|ba  swap coils\n"
    "  cfg                   show the configuration    save  write it to flash\n"
    "  defaults              restore defaults (until saved)\n"
    "  t <hz>                telemetry lines per second (0 = off, max 1000)\n"
    "  ta <ax>               telemetry axes (default *)\n"
    "  tf <f,...>            per-axis fields: pos vel mode amp q (default pos,vel)\n"
    "  ts <f,...>|none       system fields: seq vmot ladder stop (default none)\n"
    "  te on|off             event lines: E <tick> done|underrun <ax>, stop <cause>, clear\n"
    "  echo on|off           echo typed characters and show the prompt (off for scripts)\n"
    "  stress on|off         every axis sweeps its own speed wave\n"
    "  ?                     status\n"
    "  c                     clear a latched e-stop / fault\n"
    "  W                     watchdog test (stall core 1)\n"
    "  X                     reboot\n"
    "  L                     dump sine table\n";

// Double precision: float resolves only ~0.06 steps at a million steps.
static double steps(int64_t units) {
    return (double)units / (double)MOTION_UNITS_PER_STEP;
}

static const char *mode_name(uint8_t m) {
    switch (m) {
    case MODE_IDLE:     return "idle";
    case MODE_POSITION: return "pos";
    case MODE_VELOCITY: return "vel";
    case MODE_JOG:      return "jog";
    case MODE_PVT:      return "pvt";
    case MODE_GROUP:    return "grp";
    }
    return "?";
}

// "*", "3" or "1,3,5" -> bit mask; 0 if invalid.
static uint32_t parse_axes(const char *s) {
    if (!s)
        return 0;
    if (strcmp(s, "*") == 0)
        return CONTROL_ALL_AXES;
    uint32_t mask = 0;
    while (*s) {
        char *end;
        long a = strtol(s, &end, 10);
        if (end == s || a < 1 || a > NUM_MOTORS)
            return 0;
        mask |= 1u << (a - 1);
        s = end;
        if (*s == ',')
            s++;
        else if (*s)
            return 0;
    }
    return mask;
}

// "pos,vel" -> bit mask over `names`.
static bool parse_names(const char *s, const char *const *names, int n, uint32_t *mask) {
    char buf[LINE_MAX];
    strncpy(buf, s, sizeof buf - 1);
    buf[sizeof buf - 1] = '\0';
    *mask = 0;
    for (char *tok = strtok(buf, ","); tok; tok = strtok(NULL, ",")) {
        int k = 0;
        while (k < n && strcmp(tok, names[k]) != 0)
            k++;
        if (k == n)
            return false;
        *mask |= 1u << k;
    }
    return *mask != 0;
}

static bool parse_float(const char *s, float *out) {
    if (!s)
        return false;
    char *end;
    *out = strtof(s, &end);
    return end != s && *end == '\0';
}

static void post(control_cmd_t *c) {
    if (!control_post(c))
        printf("command queue full\n");
}

static void print_axes(void) {
    control_snapshot_t s;
    control_snapshot(&s);
    printf("  ax  mode  position        speed    amp  vmax   amax  profile\n");
    for (int i = 0; i < NUM_MOTORS; i++) {
        printf("  %2d  %-4s %12.3f %9.1f %5.2f%s %5.0f %6.0f  %s", i + 1, mode_name(s.mode[i]),
               steps(s.pos[i]), (double)s.vel[i], (double)s.amp[i],
               (s.holding_mask >> i) & 1 ? "h" : " ", (double)s.vmax[i], (double)s.amax[i],
               motion_profile_name((motion_profile_t)s.profile[i]));
        if (s.profile[i] == PROFILE_SCURVE || s.profile[i] == PROFILE_SMOOTH)
            printf(" %u ms", s.jerk_ms[i]);
        if (s.pvt_depth[i])
            printf("  pvt queue %u", s.pvt_depth[i]);
        putchar('\n');
    }
    for (int k = 0; k < GROUP_COUNT; k++) {
        if (!s.group[k].active)
            continue;
        printf("  group %d: axes", k + 1);
        for (int i = 0; i < NUM_MOTORS; i++)
            if (s.group[k].members & (1u << i))
                printf(" %d", i + 1);
        printf(", %s, path speed %.1f, %u queued%s, %lu segments done\n",
               s.group[k].hold ? "held" : s.group[k].running ? "running" : "idle",
               (double)s.group[k].v, s.group[k].queued, s.group[k].arc ? " + arc" : "",
               (unsigned long)s.group[k].segments_done);
    }
    if (s.pvt_underruns || s.pvt_dropped)
        printf("  pvt: %lu underrun(s), %lu point(s) dropped (queue full)\n",
               (unsigned long)s.pvt_underruns, (unsigned long)s.pvt_dropped);
    if (s.rejected)
        printf("  %lu command(s) rejected (z, prof, pgo and g need axes at rest; grouped axes\n"
               "    only take group commands)\n",
               (unsigned long)s.rejected);
}

static void print_config(void) {
    control_snapshot_t s;
    control_snapshot(&s);
    const config_t *saved = app_config();
    printf("  ax   vmax    amax  profile      amp low/high/hold   speeds      dir  coils\n");
    for (int i = 0; i < NUM_MOTORS; i++) {
        const config_drive_t *d = &s.drive[i];
        const config_axis_t *a = &saved->axis[i];
        bool changed = a->vmax != s.vmax[i] || a->amax != s.amax[i] || a->profile != s.profile[i] ||
                       a->jerk_ms != s.jerk_ms[i] || memcmp(&a->drive, d, sizeof *d) != 0;
        printf("  %2d %6.0f %7.0f  %-7s %3u  %3.0f%% /%3.0f%% /%3.0f%%  %5.0f-%-5.0f  %s  %s%s\n", i + 1,
               (double)s.vmax[i], (double)s.amax[i], motion_profile_name((motion_profile_t)s.profile[i]),
               s.jerk_ms[i], (double)(d->amp_low * 100), (double)(d->amp_high * 100),
               (double)(d->amp_hold * 100), (double)d->low_speed, (double)d->high_speed,
               d->flags & CONFIG_FLAG_REVERSE ? "rev" : "fwd", d->flags & CONFIG_FLAG_SWAP_COILS ? "ba" : "ab",
               changed ? "  (not saved)" : "");
    }
}

static void run_line(char *buf) {
    char *argv[16];
    int argc = 0;
    for (char *tok = strtok(buf, " \t"); tok && argc < 16; tok = strtok(NULL, " \t"))
        argv[argc++] = tok;
    if (argc == 0)
        return;
    const char *cmd = argv[0];
    control_cmd_t c = {0};
    float f;

    if (strcmp(cmd, "help") == 0 || strcmp(cmd, "h") == 0) {
        fputs(help_text, stdout);
    } else if (strcmp(cmd, "?") == 0) {
        app_print_status();
        print_axes();
    } else if ((strcmp(cmd, "m") == 0 || strcmp(cmd, "mr") == 0) && argc == 3) {
        c.axes = (uint16_t)parse_axes(argv[1]);
        if (!c.axes || !parse_float(argv[2], &f))
            goto usage;
        c.type = cmd[1] == 'r' ? CMD_MOVE_REL : CMD_MOVE;
        c.pos = motion_steps_to_units(f);
        app_set_stress(false);
        post(&c);
    } else if ((strcmp(cmd, "v") == 0 || strcmp(cmd, "j") == 0) && (argc == 3 || (argc == 4 && cmd[0] == 'j'))) {
        c.axes = (uint16_t)parse_axes(argv[1]);
        if (!c.axes || !parse_float(argv[2], &c.f1))
            goto usage;
        c.type = cmd[0] == 'j' ? CMD_JOG : CMD_VELOCITY;
        c.ms = argc == 4 ? (uint32_t)atoi(argv[3]) : JOG_TIMEOUT_MS_DEFAULT;
        app_set_stress(false);
        post(&c);
    } else if (strcmp(cmd, "s") == 0 && argc <= 2) {
        c.axes = (uint16_t)(argc == 2 ? parse_axes(argv[1]) : CONTROL_ALL_AXES);
        if (!c.axes)
            goto usage;
        c.type = CMD_STOP;
        app_set_stress(false);
        post(&c);
    } else if (strcmp(cmd, "z") == 0 && (argc == 2 || argc == 3)) {
        c.axes = (uint16_t)parse_axes(argv[1]);
        f = 0.0f;
        if (!c.axes || (argc == 3 && !parse_float(argv[2], &f)))
            goto usage;
        c.type = CMD_SET_POS;
        c.pos = motion_steps_to_units(f);
        post(&c);
    } else if (strcmp(cmd, "lim") == 0 && (argc == 3 || argc == 4)) {
        c.axes = (uint16_t)parse_axes(argv[1]);
        if (!c.axes || !parse_float(argv[2], &c.f1) || (argc == 4 && !parse_float(argv[3], &c.f2)))
            goto usage;
        c.type = CMD_LIMITS;
        post(&c);
    } else if (strcmp(cmd, "p") == 0 && argc == 5) {
        c.axes = (uint16_t)parse_axes(argv[1]);
        float ms;
        if (!c.axes || !parse_float(argv[2], &f) || !parse_float(argv[3], &c.f1) ||
            !parse_float(argv[4], &ms) || ms < 1.0f)
            goto usage;
        c.type = CMD_PVT_POINT;
        c.pos = motion_steps_to_units(f);
        c.ms = (uint32_t)ms;
        app_set_stress(false);
        post(&c);
    } else if (strcmp(cmd, "pgo") == 0 && argc == 2) {
        c.axes = (uint16_t)parse_axes(argv[1]);
        if (!c.axes)
            goto usage;
        c.type = CMD_PVT_START;
        post(&c);
    } else if (cmd[0] == 'g' && argc >= 2 &&
               (cmd[1] == '\0' || (cmd[2] == '\0' && strchr("lahrs", cmd[1])))) {
        long id = strtol(argv[1], NULL, 10);
        if (id < 1 || id > GROUP_COUNT)
            goto usage;
        c.ms = (uint32_t)(id - 1);
        // Trailing f<feed> / t<tol> options (gl, ga).
        while (argc > 2 && (argv[argc - 1][0] == 'f' || argv[argc - 1][0] == 't') &&
               (cmd[1] == 'l' || cmd[1] == 'a')) {
            float *dst = argv[argc - 1][0] == 'f' ? &c.f1 : &c.f2;
            if (!parse_float(argv[argc - 1] + 1, dst))
                goto usage;
            argc--;
        }
        switch (cmd[1]) {
        case '\0':
            if (argc == 3 && strcmp(argv[2], "off") == 0) {
                c.type = CMD_GROUP_RELEASE;
                break;
            }
            if (argc < 3 || argc > 5)
                goto usage;
            c.type = CMD_GROUP_CREATE;
            c.axes = (uint16_t)parse_axes(argv[2]);
            c.f1 = GROUP_DEFAULT_FEED;
            if (!c.axes || (argc >= 4 && !parse_float(argv[3], &c.f1)) ||
                (argc == 5 && !parse_float(argv[4], &c.f2)))
                goto usage;
            break;
        case 'l':
            c.type = CMD_GROUP_LINE;
            c.n = (uint8_t)(argc - 2);
            if (c.n < 1 || c.n > GROUP_MAX_AXES)
                goto usage;
            for (int k = 0; k < c.n; k++) {
                if (!parse_float(argv[2 + k], &f))
                    goto usage;
                c.vec[k] = motion_steps_to_units(f);
            }
            break;
        case 'a': {
            float cx, cy;
            if (argc != 5 || !parse_float(argv[2], &cx) || !parse_float(argv[3], &cy))
                goto usage;
            char *end;
            double deg = strtod(argv[4], &end);
            if (*end)
                goto usage;
            c.type = CMD_GROUP_ARC;
            c.vec[0] = motion_steps_to_units(cx);
            c.vec[1] = motion_steps_to_units(cy);
            c.d1 = deg * 3.14159265358979323846 / 180.0;
            break;
        }
        case 'h':
        case 'r':
            c.type = CMD_GROUP_HOLD;
            c.f1 = cmd[1] == 'h' ? 1.0f : 0.0f;
            break;
        case 's':
            c.type = CMD_GROUP_STOP;
            break;
        }
        app_set_stress(false);
        post(&c);
    } else if (strcmp(cmd, "prof") == 0 && (argc == 3 || argc == 4)) {
        c.axes = (uint16_t)parse_axes(argv[1]);
        int p = 0;
        while (p < PROFILE_COUNT && strcmp(argv[2], motion_profile_name((motion_profile_t)p)) != 0)
            p++;
        f = 30.0f;
        if (!c.axes || p == PROFILE_COUNT || (argc == 4 && !parse_float(argv[3], &f)))
            goto usage;
        c.type = CMD_PROFILE;
        c.ms = (uint32_t)p;
        c.f1 = f;
        post(&c);
    } else if (strcmp(cmd, "cfg") == 0) {
        print_config();
    } else if (strcmp(cmd, "save") == 0) {
        const char *err = app_save_config();
        if (err)
            printf("not saved: %s\n", err);
        else
            printf("saved (generation %lu)\n", (unsigned long)app_config()->generation);
    } else if (strcmp(cmd, "defaults") == 0) {
        config_t d;
        config_defaults(&d);
        for (int i = 0; i < NUM_MOTORS; i++) {
            const config_axis_t *a = &d.axis[i];
            control_cmd_t k = {.axes = (uint16_t)(1u << i)};
            k.type = CMD_LIMITS; k.f1 = a->vmax; k.f2 = a->amax; post(&k);
            k.type = CMD_PROFILE; k.ms = a->profile; k.f1 = a->jerk_ms; post(&k);
            k.type = CMD_DRIVE; k.drive = a->drive; post(&k);
        }
        printf("defaults restored (save to keep them)\n");
    } else if ((strcmp(cmd, "drive") == 0 && (argc == 5 || argc == 7)) ||
               ((strcmp(cmd, "dir") == 0 || strcmp(cmd, "coils") == 0) && argc == 3)) {
        uint32_t mask = parse_axes(argv[1]);
        if (!mask)
            goto usage;
        float v[5];
        bool drive_cmd = cmd[0] == 'd' && cmd[1] == 'r';
        for (int k = 0; drive_cmd && k < argc - 2; k++)
            if (!parse_float(argv[2 + k], &v[k]) || (k < 3 && (v[k] < 0.0f || v[k] > 100.0f)))
                goto usage;
        if (!drive_cmd && strcmp(argv[2], "fwd") && strcmp(argv[2], "rev") &&
            strcmp(argv[2], "ab") && strcmp(argv[2], "ba"))
            goto usage;
        control_snapshot_t s;
        control_snapshot(&s);
        for (int i = 0; i < NUM_MOTORS; i++) {
            if (!(mask & (1u << i)))
                continue;
            control_cmd_t k = {.type = CMD_DRIVE, .axes = (uint16_t)(1u << i), .drive = s.drive[i]};
            config_drive_t *d = &k.drive;
            if (drive_cmd) {
                d->amp_low = v[0] / 100.0f;
                d->amp_high = v[1] / 100.0f;
                d->amp_hold = v[2] / 100.0f;
                if (argc == 7) {
                    d->low_speed = v[3];
                    d->high_speed = v[4];
                }
            } else if (strcmp(cmd, "dir") == 0) {
                d->flags = (uint8_t)((d->flags & ~CONFIG_FLAG_REVERSE) |
                                     (strcmp(argv[2], "rev") == 0 ? CONFIG_FLAG_REVERSE : 0));
            } else {
                d->flags = (uint8_t)((d->flags & ~CONFIG_FLAG_SWAP_COILS) |
                                     (strcmp(argv[2], "ba") == 0 ? CONFIG_FLAG_SWAP_COILS : 0));
            }
            post(&k);
        }
    } else if (strcmp(cmd, "amp") == 0 && argc == 2) {
        if (strcmp(argv[1], "auto") == 0)
            control_manual_amp = -1.0f;
        else if (parse_float(argv[1], &f) && f >= 0.0f && f <= 100.0f)
            control_manual_amp = f / 100.0f;
        else
            goto usage;
    } else if (strcmp(cmd, "t") == 0 && argc == 2) {
        int hz = atoi(argv[1]);
        if (hz < 0 || hz > MAX_TELEM_HZ)
            goto usage;
        console_set_telemetry((uint32_t)hz, telem_axes, telem_fields, telem_sys, events_on, false);
    } else if (strcmp(cmd, "ta") == 0 && argc == 2) {
        uint32_t m = parse_axes(argv[1]);
        if (!m)
            goto usage;
        telem_axes = m;
        telem_header_due = true;
    } else if ((strcmp(cmd, "tf") == 0 || strcmp(cmd, "ts") == 0) && argc == 2) {
        bool axis = cmd[1] == 'f';
        uint32_t m;
        if (!axis && strcmp(argv[1], "none") == 0)
            m = 0;
        else if (!parse_names(argv[1], axis ? axis_field_names : sys_field_names,
                              axis ? 5 : 4, &m))
            goto usage;
        if (axis)
            telem_fields = m;
        else
            telem_sys = m;
        telem_header_due = true;
    } else if (strcmp(cmd, "te") == 0 && argc == 2) {
        events_on = strcmp(argv[1], "off") != 0;
        telem_binary = false;
    } else if (strcmp(cmd, "echo") == 0 && argc == 2) {
        echo = strcmp(argv[1], "off") != 0;
    } else if (strcmp(cmd, "stress") == 0 && argc == 2) {
        app_set_stress(strcmp(argv[1], "on") == 0);
    } else if (strcmp(cmd, "c") == 0) {
        control_clear_request = true;
    } else if (strcmp(cmd, "W") == 0) {
        printf("watchdog test: stalling core 1\n");
        control_stall_test = true;
    } else if (strcmp(cmd, "X") == 0) {
        printf("rebooting\n");
        sleep_ms(50);
        watchdog_reboot(0, 0, 0);
        for (;;)
            tight_loop_contents();
    } else if (strcmp(cmd, "L") == 0) {
        for (uint32_t i = 0; i < (1u << MICROSTEP_LUT_BITS); i++)
            printf("%d%c", microstep_sine_lut[i], (i & 15) == 15 ? '\n' : ' ');
    } else {
        goto usage;
    }
    return;

usage:
    printf("? %s (type help)\n", cmd);
}

void console_input(int c) {
    if (c == '\r' || c == '\n') {
        if (echo)
            putchar('\n');
        line[line_len] = '\0';
        line_len = 0;
        run_line(line);
        if (echo)
            printf("> ");
    } else if (c == '\b' || c == 0x7f) {
        if (line_len) {
            line_len--;
            if (echo)
                printf("\b \b");
        }
    } else if (isprint(c) && line_len < LINE_MAX - 1) {
        line[line_len++] = (char)c;
        if (echo)
            putchar(c);
    }
}

void console_set_telemetry(uint32_t hz, uint32_t axes, uint32_t fields, uint32_t sys,
                           bool events, bool binary) {
    telem_period_us = hz ? 1000000u / hz : 0;
    telem_last_us = time_us_32() - telem_period_us;  // first line now
    telem_header_due = hz != 0 && !binary;
    telem_axes = axes & CONTROL_ALL_AXES;
    telem_fields = fields;
    telem_sys = sys;
    events_on = events;
    telem_binary = binary;
    telem_line = 0;
}

// --- telemetry -------------------------------------------------------------
// Integers only: float printf is too slow for 10 axes at 1 kHz on core 0.

typedef struct {
    char buf[512];
    uint32_t len;
} line_t;

static void put_char(line_t *l, char c) {
    if (l->len < sizeof l->buf - 1)
        l->buf[l->len++] = c;
}

static void put_str(line_t *l, const char *s) {
    while (*s)
        put_char(l, *s++);
}

static void put_u64(line_t *l, uint64_t v) {
    char tmp[21];
    int n = 0;
    do {
        tmp[n++] = (char)('0' + v % 10);
        v /= 10;
    } while (v);
    while (n)
        put_char(l, tmp[--n]);
}

static void put_i64(line_t *l, int64_t v) {
    if (v < 0) {
        put_char(l, '-');
        put_u64(l, (uint64_t)-v);
    } else {
        put_u64(l, (uint64_t)v);
    }
}

// Value in 1/10^decimals, printed with a decimal point.
static void put_fixed(line_t *l, int64_t v, int decimals, int64_t scale) {
    if (v < 0) {
        put_char(l, '-');
        v = -v;
    }
    put_u64(l, (uint64_t)(v / scale));
    put_char(l, '.');
    int64_t frac = v % scale;
    for (int64_t d = scale / 10; decimals--; d /= 10) {
        put_char(l, (char)('0' + frac / d));
        frac %= d;
    }
}

// Position in full steps with 3 decimals, rounded.
static void put_pos(line_t *l, int64_t units) {
    bool neg = units < 0;
    uint64_t u = neg ? (uint64_t)-units : (uint64_t)units;
    uint64_t whole = u >> 30;
    uint64_t milli = ((u & ((1ull << 30) - 1)) * 1000 + (1ull << 29)) >> 30;
    int64_t v = (int64_t)(whole * 1000 + milli);
    put_fixed(l, neg ? -v : v, 3, 1000);
}

static void flush_line(line_t *l) {
    put_char(l, '\n');
    fwrite(l->buf, 1, l->len, stdout);
    fflush(stdout);
}

static char mode_letter(uint8_t m) {
    static const char letters[] = "ipvjt";  // idle pos vel jog pvt
    return m < sizeof letters - 1 ? letters[m] : '?';
}

static void print_header(void) {
    line_t l = {0};
    put_str(&l, "# T line tick");
    for (int k = 0; k < 4; k++)
        if (telem_sys & (1u << k)) {
            put_char(&l, ' ');
            put_str(&l, sys_field_names[k]);
        }
    for (int i = 0; i < NUM_MOTORS; i++) {
        if (!(telem_axes & (1u << i)))
            continue;
        for (int k = 0; k < 5; k++)
            if (telem_fields & (1u << k)) {
                put_char(&l, ' ');
                put_u64(&l, (uint64_t)(i + 1));
                put_char(&l, '.');
                put_str(&l, axis_field_names[k]);
            }
    }
    flush_line(&l);
}

static void print_line(const control_snapshot_t *s) {
    line_t l = {0};
    put_str(&l, "T ");
    put_u64(&l, ++telem_line);
    put_char(&l, ' ');
    put_u64(&l, s->tick);
    if (telem_sys & TS_SEQ) {
        put_char(&l, ' ');
        put_u64(&l, s->last_seq);
    }
    if (telem_sys & TS_VMOT) {
        put_char(&l, ' ');
        put_u64(&l, adc_monitor_vmot_mv());
    }
    if (telem_sys & TS_LADDER) {
        put_char(&l, ' ');
        put_u64(&l, adc_monitor_pin_mv(MON_LADDER));
    }
    if (telem_sys & TS_STOP) {
        // 0 running, else the latched cause: 1 e-stop, 2 driver fault.
        put_char(&l, ' ');
        put_char(&l, control_outputs_on ? '0' :
                 control_ladder.stop_cause == LADDER_FAULT ? '2' : '1');
    }
    for (int i = 0; i < NUM_MOTORS; i++) {
        if (!(telem_axes & (1u << i)))
            continue;
        if (telem_fields & TF_POS) {
            put_char(&l, ' ');
            put_pos(&l, s->pos[i]);
        }
        if (telem_fields & TF_VEL) {
            float v = s->vel[i] * 10.0f;
            put_char(&l, ' ');
            put_fixed(&l, (int64_t)(v >= 0.0f ? v + 0.5f : v - 0.5f), 1, 10);
        }
        if (telem_fields & TF_MODE) {
            put_char(&l, ' ');
            put_char(&l, (s->holding_mask >> i) & 1 ? 'h' : mode_letter(s->mode[i]));
        }
        if (telem_fields & TF_AMP) {
            put_char(&l, ' ');
            put_u64(&l, (uint64_t)(s->amp[i] * 100.0f + 0.5f));
        }
        if (telem_fields & TF_Q) {
            put_char(&l, ' ');
            put_u64(&l, s->pvt_depth[i]);
        }
    }
    flush_line(&l);
}

static void print_events(const control_snapshot_t *s) {
    static bool have_prev;
    static uint32_t prev_settled;
    static uint16_t prev_underrun[NUM_MOTORS];
    static bool prev_on = true;
    bool on = control_outputs_on;

    if (events_on && have_prev) {
        line_t l = {0};
        for (int i = 0; i < NUM_MOTORS; i++) {
            bool done = (s->settled_mask & ~prev_settled) & (1u << i);
            bool underrun = s->pvt_underrun[i] != prev_underrun[i];
            if (!done && !underrun)
                continue;
            if (telem_binary) {
                protocol_send_event(s->tick, done ? PROTO_EV_DONE : PROTO_EV_UNDERRUN,
                                    (uint8_t)(i + 1), s->pos[i]);
                continue;
            }
            l.len = 0;
            put_str(&l, "E ");
            put_u64(&l, s->tick);
            put_str(&l, done ? " done " : " underrun ");
            put_u64(&l, (uint64_t)(i + 1));
            if (done) {
                put_char(&l, ' ');
                put_pos(&l, s->pos[i]);
            }
            flush_line(&l);
        }
        if (on != prev_on && telem_binary) {
            protocol_send_event(s->tick, on ? PROTO_EV_CLEAR :
                                control_ladder.stop_cause == LADDER_FAULT ? PROTO_EV_FAULT : PROTO_EV_ESTOP,
                                0, 0);
        } else if (on != prev_on) {
            l.len = 0;
            put_str(&l, "E ");
            put_u64(&l, s->tick);
            put_str(&l, on ? " clear" : control_ladder.stop_cause == LADDER_FAULT ? " stop fault"
                                                                              : " stop estop");
            flush_line(&l);
        }
    }
    have_prev = true;
    prev_settled = s->settled_mask;
    memcpy(prev_underrun, s->pvt_underrun, sizeof prev_underrun);
    prev_on = on;
}

void console_telemetry(uint32_t now_us) {
    bool line_due = telem_period_us && now_us - telem_last_us >= telem_period_us;
    if (!line_due && !events_on)
        return;
    control_snapshot_t s;
    control_snapshot(&s);
    print_events(&s);
    if (!line_due)
        return;
    // Keep the cadence even if a line is late.
    telem_last_us = now_us - (now_us - telem_last_us) % telem_period_us;
    if (now_us - telem_last_us >= telem_period_us)
        telem_last_us = now_us;
    if (telem_binary) {
        protocol_send_telemetry(&s, ++telem_line, telem_axes, telem_fields, telem_sys);
        return;
    }
    if (telem_header_due) {
        telem_header_due = false;
        print_header();
    }
    print_line(&s);
}
