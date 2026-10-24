// Bench firmware for the RP2350 stepper controller.
//
// Core 1 (control.c) runs the 1 kHz trajectory tick for every axis and
// streams sine-PWM to the PIO state machines through DMA rings. Core 0 (this
// file) sets everything up, then runs the USB console (console.c), the
// status LEDs, the stress test, the ToF sensor on the Qwiic port (tof.c)
// and the core 1 watchdog.
//
// Standalone mode (standalone.h): the buttons step through a list of the
// built-in motion programs (programs.h) followed by the stored shows. Btn1
// clears a latched stop or starts / stops the selected entry, Btn2 selects
// (or switches to) the next one. An optional boot entry starts at power-up.
//
// LEDs: pixel 0 is the general status, pixels 1-10 show each axis (green
// forward, blue reverse, brighter = faster; dim white holding; grey stopped
// but not yet holding; red while a stop is latched). For a moment after a
// button picks an entry, they show its number as a bar instead.

#include <math.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"

#include "adc_monitor.h"
#include "board_pins.h"
#include "cam_store.h"
#include "config.h"
#include "console.h"
#include "protocol.h"
#include "control.h"
#include "hbridge.h"
#include "led.h"
#include "i2c_target.h"
#include "tof.h"
#include "player.h"
#include "programs.h"
#include "show_store.h"
#include "standalone.h"
#include "trig.h"

#define POWER_UP_SETTLE_MS 100
// Each ring holds 3.2 ms; trip well before a stalled producer lets it wrap.
#define WATCHDOG_US 1500

// Stress test: axis i swings sinusoidally about where it stood when the test
// started, with its own period and phase, so the speed pattern rolls along
// the LED strip. Swing = STRESS_PEAK * period / 2pi each way (axis 1 ~670,
// axis 10 ~1370 full steps), sized for linear actuators. Every STRESS_CYCLE_S
// all axes pause for STRESS_PAUSE_S to exercise stop, hold and restart; the
// swing's clock stops during the pause and a position correction takes up
// the stopping distance, so the axes stay centred on their start.
#define STRESS_PEAK        700.0f   // full steps/s
#define STRESS_PERIOD_S    6.0f     // axis 1; later axes slightly slower
#define STRESS_PERIOD_STEP 0.7f
#define STRESS_CYCLE_S     20.0f
#define STRESS_PAUSE_S     2.5f
#define STRESS_RAMP_S      3.0f     // swing grows from 0 over this long at start
#define STRESS_GAIN        2.0f     // position correction, 1/s
#define STRESS_UPDATE_US   10000
#define LED_UPDATE_US      20000
#define LED_FULL_SPEED     1600.0f  // full brightness at this speed
#define SELECT_SHOW_US     1500000u // how long the selected entry's number shows

// Button entries: the built-in programs, then the stored show slots.
#define SHOW_ENTRY(slot)   (PROGRAM_COUNT + (slot))
#define ENTRIES            (PROGRAM_COUNT + SHOW_SLOTS)

hbridge_t motors[NUM_MOTORS];

static bool stress;
static uint64_t stress_start;
static int64_t stress_center[NUM_MOTORS];  // units
static bool tripped;  // core 1 watchdog fired
static config_t config;   // as loaded / last saved
static int selected_entry = -1;  // what Btn1 starts
static int pending_entry = -1;   // start once the axes are at rest and outputs on
static uint32_t select_shown_at; // when a button last picked an entry
static bool select_showing;
static bool config_from_flash;
// Core 0 idle: time spent waiting for serial input (USB interrupts that run
// during the wait count as idle, so core 0 load reads slightly low).
static uint32_t core0_idle_us;

bool app_stress_on(void) {
    return stress;
}

void app_set_stress(bool on) {
    if (on == stress)
        return;
    if (on && !control_outputs_on) {
        printf("stress test: clear the stop first (c)\n");
        return;
    }
    if (on) {
        player_stop();
        pending_entry = -1;
        control_snapshot_t snap;
        control_snapshot(&snap);
        for (int i = 0; i < NUM_MOTORS; i++)
            stress_center[i] = snap.pos[i];
    }
    stress = on;
    stress_start = time_us_64();
    if (!on) {
        control_cmd_t c = {.type = CMD_STOP, .axes = CONTROL_ALL_AXES};
        control_post(&c);
    }
    printf("stress test %s\n", on ? "on" : "off");
}

const config_t *app_config(void) {
    return &config;
}

void app_set_boot_show(int slot) {
    config.boot_show = slot < 0 ? 0xff : (uint8_t)slot;
    if (slot >= 0)
        selected_entry = SHOW_ENTRY(slot);
}

void app_set_boot_program(int k) {
    config.boot_show = (uint8_t)(CONFIG_BOOT_PROGRAM + k);
    selected_entry = k;
}

static int boot_entry(void) {
    if (config.boot_show < SHOW_SLOTS)
        return SHOW_ENTRY(config.boot_show);
    int k = config.boot_show - CONFIG_BOOT_PROGRAM;
    return k >= 0 && k < PROGRAM_COUNT ? k : -1;
}

static const char *entry_name(int e) {
    static char name[40];
    if (e < PROGRAM_COUNT)
        snprintf(name, sizeof name, "program %d (%s)", e + 1, programs[e].name);
    else
        snprintf(name, sizeof name, "show %d", e - PROGRAM_COUNT + 1);
    return name;
}

static int running_entry(void) {
    if (player_program() >= 0)
        return player_program();
    return player_slot() >= 0 ? SHOW_ENTRY(player_slot()) : -1;
}

static uint32_t valid_slots(void) {
    uint32_t mask = 0;
    for (uint32_t k = 0; k < SHOW_SLOTS; k++) {
        uint32_t len;
        show_t s;
        const uint8_t *blob = show_store_blob(k, &len);
        if (blob && !show_parse(&s, blob, len, NUM_MOTORS, LED_CHAIN))
            mask |= 1u << k;
    }
    return mask;
}

static void start_entry(int e) {
    const char *err = e < PROGRAM_COUNT ? player_start_program((uint32_t)e)
                                        : player_start((uint32_t)(e - PROGRAM_COUNT));
    if (err)
        printf("%s not started: %s\n", entry_name(e), err);
    else
        printf("%s started\n", entry_name(e));
}

// Stop whatever is playing; start entry e once the axes are at rest.
static void queue_entry(int e) {
    app_set_stress(false);
    player_stop();
    pending_entry = selected_entry = e;
}

static void show_selection(void) {
    select_shown_at = time_us_32();
    select_showing = true;
}

const char *app_start_program(int k) {
    if (k < 0 || k >= PROGRAM_COUNT)
        return "no such program";
    if (!control_outputs_on)
        return "outputs are stopped (c to clear)";
    queue_entry(k);
    return NULL;
}

void app_stop_playing(void) {
    player_stop();
    pending_entry = -1;
}

static void on_button(int button) {
    sa_state_t st = {
        .stop_latched = !control_outputs_on,
        .line_idle = control_ladder.raw_prev != LADDER_ESTOP && control_ladder.raw_prev != LADDER_FAULT,
        .running = pending_entry >= 0 ? pending_entry : running_entry(),
        .selected = selected_entry,
        .valid = ((1u << PROGRAM_COUNT) - 1) | valid_slots() << PROGRAM_COUNT,
    };
    sa_result_t r = sa_button(&st, button, ENTRIES);
    switch (r.action) {
    case SA_CLEAR:
        control_clear_request = true;
        printf("btn%d: clear stop\n", button);
        return;
    case SA_START:
    case SA_SWITCH:
        queue_entry(r.slot);
        break;
    case SA_STOP:
        printf("btn%d: stop %s\n", button, entry_name(st.running));
        app_stop_playing();
        return;
    case SA_SELECT:
        selected_entry = r.slot;
        break;
    case SA_NONE:
        printf("btn%d: nothing to do\n", button);
        return;
    }
    show_selection();
    printf("btn%d: %s %s\n", button, r.action == SA_SELECT ? "select" : r.action == SA_START ? "start" : "switch to",
           entry_name(r.slot));
}

// A queued entry (boot, button or console) starts once the axes are at rest.
static void poll_pending(void) {
    if (pending_entry < 0 || running_entry() >= 0 || !control_outputs_on)
        return;
    static control_snapshot_t s;  // ~1 KB: off core 0's stack
    control_snapshot(&s);
    if (s.settled_mask != CONTROL_ALL_AXES)
        return;
    int e = pending_entry;
    pending_entry = -1;
    start_entry(e);
}

const char *app_save_config(void) {
    static control_snapshot_t s;  // ~1 KB: off core 0's stack
    control_snapshot(&s);
    // Writing flash pauses core 1 for tens of ms while the DMA keeps
    // replaying the ring. At rest every queued period drives the same
    // electrical phase (only the amplitude may still be easing to hold),
    // so the coils just keep holding; while moving, the rotor would jerk.
    if (control_outputs_on && s.settled_mask != CONTROL_ALL_AXES)
        return "all axes must be at rest";
    config_t c = config;
    for (int i = 0; i < NUM_MOTORS; i++) {
        c.axis[i].vmax = s.vmax[i];
        c.axis[i].amax = s.amax[i];
        c.axis[i].profile = s.profile[i];
        c.axis[i].jerk_ms = s.jerk_ms[i];
        c.axis[i].drive = s.drive[i];
    }
    control_flash_busy = true;  // pauses the core 1 watchdog
    bool ok = config_save(&c);
    // Cam tables mirror what is loaded: loaded ones are stored, the rest
    // cleared (unchanged sectors aren't rewritten). The staging copies match
    // the loaded tables: core 0 only changes them through control_cam_load.
    for (uint32_t k = 0; ok && k < CAM_TABLES; k++) {
        const cam_table_t *t = &control_cam_staging[k];
        bool loaded = s.cams_loaded & (1u << k);
        ok = cam_store_write(k, t->x, t->y, loaded ? t->n : 0, t->cyclic);
    }
    control_flash_busy = false;
    if (!ok)
        return "flash write failed";
    config = c;
    config_from_flash = true;
    return NULL;
}

void app_print_status(void) {
    static uint32_t last_us;
    uint32_t now = time_us_32();
    uint32_t busy = control_busy_us, low = control_min_queued, idle0 = core0_idle_us;
    control_busy_us = 0;
    control_min_queued = HBRIDGE_RING_PERIODS;
    core0_idle_us = 0;

    if (config_from_flash)
        printf("config: saved, generation %lu\n", (unsigned long)config.generation);
    else
        printf("config: defaults (nothing saved yet)\n");
    if (running_entry() >= 0)
        printf("%s: %s\n", entry_name(running_entry()), player_state());
    else if (pending_entry >= 0)
        printf("%s: waiting for the axes to be at rest%s\n", entry_name(pending_entry),
               control_outputs_on ? "" : " and the stop to be cleared");
    else if (selected_entry >= 0)
        printf("selected: %s\n", entry_name(selected_entry));
    if (boot_entry() >= 0)
        printf("boot: %s\n", entry_name(boot_entry()));
    printf("%s%s%s\n", tripped ? "WATCHDOG TRIPPED (X to reboot)" :
                       control_outputs_on ? "running" : "STOPPED (c to clear)",
           control_manual_amp >= 0.0f ? ", manual amplitude" : "", stress ? ", stress test" : "");

    uint32_t ladder_mv = adc_monitor_pin_mv(MON_LADDER);
    printf("  ladder %lu mV (%s%s)  vmot %lu mV  board id %lu mV",
           (unsigned long)ladder_mv, ladder_level_name(ladder_classify(ladder_mv)),
#ifdef LADDER_BYPASS
           ", BYPASSED",
#else
           control_ladder.stop_latched ? ", stop latched" : "",
#endif
           (unsigned long)adc_monitor_vmot_mv(), (unsigned long)adc_monitor_pin_mv(MON_BOARD_ID));
    if (adc_monitor_present(MON_CC1))
        printf("  cc %lu/%lu mV", (unsigned long)adc_monitor_pin_mv(MON_CC1),
               (unsigned long)adc_monitor_pin_mv(MON_CC2));
    if (tof_available()) {
        tof_info_t t;
        tof_get_info(&t);
        printf("  tof %s", tof_state_name(t.state));
    }

    double span = last_us ? (double)(now - last_us) : 0.0;
    printf("\n  %d motors (%d with pins), core 0 busy %.1f%%, core 1 busy %.1f%%, ring low-water ",
           NUM_MOTORS, NUM_MOTORS < BOARD_MOTORS_WITH_PINS ? NUM_MOTORS : BOARD_MOTORS_WITH_PINS,
           span > 0.0 ? 100.0 * (1.0 - idle0 / span) : 0.0, span > 0.0 ? 100.0 * busy / span : 0.0);
    if (low < HBRIDGE_RING_PERIODS)
        printf("%lu/%u periods\n", (unsigned long)low, HBRIDGE_RING_PERIODS - 1);
    else
        printf("- (no refills)\n");
    last_us = now;
}

// Pixel 0: green running, blue all holding, red blinking e-stop, magenta
// blinking driver fault, solid red watchdog trip, amber ladder bypassed.
static void update_leds(void) {
    static control_snapshot_t s;  // ~1 KB: off core 0's stack
    control_snapshot(&s);
    bool blink = (time_us_32() / 250000) & 1;
    bool on = control_outputs_on;

    if (tripped)
        led_set(LED_STATUS, 255, 0, 0);
    else if (!on)
        led_set(LED_STATUS, blink ? 255 : 0, 0,
                control_ladder.stop_cause == LADDER_FAULT && blink ? 255 : 0);
    else if (s.holding_mask == CONTROL_ALL_AXES)
        led_set(LED_STATUS, 0, 0, 255);
    else
#ifdef LADDER_BYPASS
        led_set(LED_STATUS, 255, 120, 0);
#else
        led_set(LED_STATUS, 0, 255, 0);
#endif

    for (int i = 0; i < LED_AXIS_COUNT; i++) {
        uint32_t px = LED_AXIS_FIRST + (uint32_t)i;
        if (i >= NUM_MOTORS || tripped) {
            led_set(px, 0, 0, 0);
        } else if (!on) {
            led_set(px, blink ? 120 : 0, 0, 0);
        } else if (s.holding_mask & (1u << i)) {
            led_set(px, 40, 40, 40);
        } else {
            float v = s.vel[i];
            float mag = fabsf(v) / LED_FULL_SPEED;
            uint8_t level = (uint8_t)(30.0f + 225.0f * (mag > 1.0f ? 1.0f : mag));
            if (v > 0.5f)
                led_set(px, 0, level, 0);
            else if (v < -0.5f)
                led_set(px, 0, 0, level);
            else
                led_set(px, 20, 20, 20);  // at rest, not yet holding
        }
    }
    // A running show's LED tracks take over their pixels.
    for (uint32_t px = 0; px < LED_CHAIN; px++) {
        uint8_t rgb[3];
        if (player_led(px, rgb))
            led_set(px, rgb[0], rgb[1], rgb[2]);
        else if (px >= LED_COUNT)
            led_set(px, 0, 0, 0);
    }
    // Selection: a bar as long as the entry's number (counting from 1 again
    // past ten), amber for programs 1-10, cyan for 11 on, magenta for shows.
    if (select_showing && time_us_32() - select_shown_at < SELECT_SHOW_US && selected_entry >= 0) {
        bool is_show = selected_entry >= PROGRAM_COUNT;
        int k = is_show ? selected_entry - PROGRAM_COUNT : selected_entry;
        uint8_t r = 255, g = 120, b = 0;
        if (is_show)
            r = 200, g = 0, b = 200;
        else if (k >= LED_AXIS_COUNT)
            r = 0, g = 200, b = 255;
        for (int i = 0; i < LED_AXIS_COUNT; i++) {
            if (i <= k % LED_AXIS_COUNT)
                led_set(LED_AXIS_FIRST + (uint32_t)i, r, g, b);
            else
                led_set(LED_AXIS_FIRST + (uint32_t)i, 0, 0, 0);
        }
    } else {
        select_showing = false;
    }
    led_show();
}

// t: seconds since the test started (double: float loses the phase within
// hours of running).
static void stress_update(double t) {
    const double active_s = STRESS_CYCLE_S - STRESS_PAUSE_S;
    double cycles = floor(t / STRESS_CYCLE_S);
    double in_cycle = t - cycles * STRESS_CYCLE_S;
    bool pause = in_cycle > active_s;
    double ts = cycles * active_s + fmin(in_cycle, active_s);  // swing clock

    // Envelope: smoothstep 0 -> 1, so the swing starts at rest on the centre.
    float e = 1.0f, de = 0.0f;
    if (ts < STRESS_RAMP_S) {
        float u = (float)ts / STRESS_RAMP_S;
        e = u * u * (3.0f - 2.0f * u);
        de = 6.0f * u * (1.0f - u) / STRESS_RAMP_S;
    }

    control_snapshot_t snap;
    control_snapshot(&snap);
    for (int i = 0; i < NUM_MOTORS; i++) {
        float v = 0.0f;
        if (!pause) {
            float period = STRESS_PERIOD_S + STRESS_PERIOD_STEP * (float)i;
            float w = 2.0f * (float)M_PI / period;
            float phase = w * (float)fmod(ts, (double)period) +
                          2.0f * (float)M_PI * (float)i / (float)NUM_MOTORS;
            float a = STRESS_PEAK / w;  // swing each way, full steps
            float s = trig_sinf(phase), co = trig_cosf(phase);
            float target = e * a * s;
            float pos = motion_units_to_steps(snap.pos[i] - stress_center[i]);
            v = a * (de * s + e * w * co) + STRESS_GAIN * (target - pos);
        }
        control_cmd_t c = {
            .type = CMD_VELOCITY,
            .axes = (uint16_t)(1u << i),
            .f1 = v,
        };
        control_post(&c);
    }
}

int main(void) {
    stdio_init_all();
#ifdef BOOT_TRACE
    sleep_ms(3000);  // give the host time to open the port
#define TRACE(...) printf(__VA_ARGS__)
#else
#define TRACE(...) ((void)0)
#endif
    TRACE("stepper bring-up, %d motor(s)\n", NUM_MOTORS);

    for (int i = 0; i < NUM_MOTORS; i++) {
        TRACE("init motor %d\n", i + 1);
        hbridge_init(&motors[i], (uint)i, i < BOARD_MOTORS_WITH_PINS);
    }
    led_init();  // after the motors: PIO2's GPIO base is set there
    adc_monitor_init();
    config_from_flash = config_load(&config);
    control_init(&config);
    // Saved cam tables; core 1 loads them on its first tick.
    for (uint32_t k = 0; k < CAM_TABLES; k++) {
        static float x[CAM_MAX_POINTS], y[CAM_MAX_POINTS];
        uint32_t n;
        bool cyclic;
        if (cam_store_read(k, x, y, &n, &cyclic))
            control_cam_load(k, x, y, n, cyclic);
    }
    sleep_us(100);  // a few ADC rounds before core 1 starts judging the ladder
    hbridge_start(motors, NUM_MOTORS);
    TRACE("PIO started\n");

    multicore_launch_core1(control_core1_main);
    TRACE("core 1 running\n");

    // Let V5 settle before driving the coils (no soft-start on V5).
    sleep_ms(POWER_UP_SETTLE_MS);
    control_energized = true;
    i2c_target_init();  // Qwiic, on boards that have it

    // Standalone: queue the boot entry (it starts once outputs are on, so a
    // stop latched at power-up waits for Btn1 to clear it).
    if (boot_entry() >= 0)
        pending_entry = selected_entry = boot_entry();

    uint32_t last_beat = control_heartbeat;
    absolute_time_t beat_seen = get_absolute_time();
    bool was_on = true;
    uint32_t btn1_seen = 0, btn2_seen = 0;
    uint32_t last_stress = 0, last_led = 0;

    for (;;) {
        uint32_t now = time_us_32();
        if (now - last_led >= LED_UPDATE_US) {
            last_led = now;
            update_leds();
        }
        if (stress && now - last_stress >= STRESS_UPDATE_US) {
            last_stress = now;
            stress_update((double)(time_us_64() - stress_start) * 1e-6);
        }
        console_telemetry(now);
        player_poll(now);
        i2c_target_poll();
        tof_poll(now);
        if (adc_monitor_check())
            printf("ADC FIFO overflow: monitor restarted\n");

        bool on = control_outputs_on;
        if (was_on && !on) {
            stress = false;
            printf("STOP: %s, motor outputs off (c to clear)\n",
                   ladder_level_name(control_ladder.stop_cause));
        }
        if (!was_on && on)
            printf("stop cleared\n");
        was_on = on;
        if (control_clear_refused) {
            control_clear_refused = false;
            printf("cannot clear: ladder still reads %s\n",
                   ladder_level_name(control_ladder.raw_prev));
        }
        if (control_ladder.btn1_presses != btn1_seen) {
            btn1_seen = control_ladder.btn1_presses;
            on_button(1);
        }
        if (control_ladder.btn2_presses != btn2_seen) {
            btn2_seen = control_ladder.btn2_presses;
            on_button(2);
        }
        poll_pending();

        uint32_t beat = control_heartbeat;
        if (beat != last_beat || control_flash_busy) {
            last_beat = beat;
            beat_seen = get_absolute_time();
        } else if (!tripped && absolute_time_diff_us(beat_seen, get_absolute_time()) > WATCHDOG_US) {
            for (int i = 0; i < NUM_MOTORS; i++)
                hbridge_safe_off(&motors[i]);
            tripped = true;
            stress = false;
            printf("core 1 stalled: motor outputs off (X to reboot)\n");
        }

        uint32_t wait_start = time_us_32();
        int c = getchar_timeout_us(250);  // short: telemetry runs up to 1 kHz
        core0_idle_us += time_us_32() - wait_start;
        if (c != PICO_ERROR_TIMEOUT)
            protocol_input(c);
    }
}
