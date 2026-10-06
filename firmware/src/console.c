#include "console.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/watchdog.h"

#include "control.h"
#include "microstep.h"

#define LINE_MAX      80
#define MAX_TELEM_HZ  200   // ~10 axes of text per line over USB CDC
#define JOG_TIMEOUT_MS_DEFAULT 300

static char line[LINE_MAX];
static uint32_t line_len;
static uint32_t telem_period_us;  // 0 = off
static uint32_t telem_last_us;
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
    "  prof <ax> <name> [ms] motion profile: trap, scurve, smooth, cosine, quintic;\n"
    "                        ms = jerk time for scurve/smooth (default 30, max 100)\n"
    "  amp auto|<percent>    drive amplitude: automatic curve, or fixed\n"
    "  t <hz>                telemetry lines per second (0 = off, max 200)\n"
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
    if (s.pvt_underruns || s.pvt_dropped)
        printf("  pvt: %lu underrun(s), %lu point(s) dropped (queue full)\n",
               (unsigned long)s.pvt_underruns, (unsigned long)s.pvt_dropped);
    if (s.rejected)
        printf("  %lu command(s) rejected (z, prof and pgo need the axis at rest)\n",
               (unsigned long)s.rejected);
}

static void run_line(char *buf) {
    char *argv[6];
    int argc = 0;
    for (char *tok = strtok(buf, " \t"); tok && argc < 6; tok = strtok(NULL, " \t"))
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
        telem_period_us = hz ? 1000000u / (uint32_t)hz : 0;
        if (hz)
            printf("telemetry: T <tick> then <position> <speed> per axis\n");
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

void console_telemetry(uint32_t now_us) {
    if (!telem_period_us || now_us - telem_last_us < telem_period_us)
        return;
    telem_last_us = now_us;
    control_snapshot_t s;
    control_snapshot(&s);
    printf("T %lu", (unsigned long)s.tick);
    for (int i = 0; i < NUM_MOTORS; i++)
        printf(" %.3f %.1f", steps(s.pos[i]), (double)s.vel[i]);
    putchar('\n');
}
