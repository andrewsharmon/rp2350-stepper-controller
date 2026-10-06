// Bench firmware for the RP2350 stepper controller.
//
// Core 1 (control.c) runs the 1 kHz trajectory tick for every axis and
// streams sine-PWM to the PIO state machines through DMA rings. Core 0 (this
// file) sets everything up, then runs the USB console (console.c), the
// status LEDs, the stress test and the core 1 watchdog.
//
// Standalone mode (standalone.h): an optional boot show starts at power-up;
// Btn1 clears a latched stop or starts / stops the selected show, Btn2
// selects (or switches to) the next stored show.
//
// LEDs: pixel 0 is the general status, pixels 1-10 show each axis (green
// forward, blue reverse, brighter = faster; dim white holding; grey stopped
// but not yet holding; red while a stop is latched).

#include <math.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"

#include "adc_monitor.h"
#include "board_pins.h"
#include "config.h"
#include "console.h"
#include "protocol.h"
#include "control.h"
#include "hbridge.h"
#include "led.h"
#include "i2c_target.h"
#include "player.h"
#include "show_store.h"
#include "standalone.h"
#include "trig.h"

#define POWER_UP_SETTLE_MS 100
// Each ring holds 3.2 ms; trip well before a stalled producer lets it wrap.
#define WATCHDOG_US 1500

// Stress test: axis i sweeps a sine of speed with its own period and phase,
// so the speed pattern rolls along the LED strip. Every STRESS_CYCLE_S all
// axes pause for STRESS_PAUSE_S to exercise stop, hold and restart.
#define STRESS_PEAK        1400.0f  // full steps/s, inside the default vmax
#define STRESS_PERIOD_S    6.0f     // axis 1; later axes slightly slower
#define STRESS_PERIOD_STEP 0.7f
#define STRESS_CYCLE_S     20.0f
#define STRESS_PAUSE_S     2.5f
#define STRESS_UPDATE_US   10000
#define LED_UPDATE_US      20000
#define LED_FULL_SPEED     1600.0f  // full brightness at this speed

hbridge_t motors[NUM_MOTORS];

static bool stress;
static uint32_t stress_start;
static bool tripped;  // core 1 watchdog fired
static config_t config;   // as loaded / last saved
static int selected_slot = -1;   // the show Btn1 starts
static int pending_slot = -1;    // start once the axes are at rest and outputs on
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
    stress = on;
    stress_start = time_us_32();
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
        selected_slot = slot;
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

static void start_show(int slot) {
    app_set_stress(false);
    const char *err = player_start((uint32_t)slot);
    if (err)
        printf("show %d not started: %s\n", slot + 1, err);
    else
        selected_slot = slot;
}

static void on_button(int button) {
    sa_state_t st = {
        .stop_latched = !control_outputs_on,
        .line_idle = control_ladder.raw_prev != LADDER_ESTOP && control_ladder.raw_prev != LADDER_FAULT,
        .running = player_slot(),
        .selected = selected_slot,
        .valid = valid_slots(),
    };
    sa_result_t r = sa_button(&st, button, SHOW_SLOTS);
    switch (r.action) {
    case SA_CLEAR:
        control_clear_request = true;
        break;
    case SA_START:
        start_show(r.slot);
        break;
    case SA_STOP:
        player_stop();
        pending_slot = -1;
        break;
    case SA_SWITCH:
        player_stop();
        pending_slot = selected_slot = r.slot;
        break;
    case SA_SELECT:
        selected_slot = r.slot;
        break;
    case SA_NONE:
        break;
    }
    printf("btn%d: %s\n", button, r.action == SA_CLEAR ? "clear stop" : r.action == SA_START ? "start show" :
           r.action == SA_STOP ? "stop show" : r.action == SA_SWITCH ? "switch show" :
           r.action == SA_SELECT ? "select show" : "nothing to do");
}

// A boot or switched-to show starts once the axes are at rest.
static void poll_pending(void) {
    if (pending_slot < 0 || player_slot() >= 0 || !control_outputs_on)
        return;
    control_snapshot_t s;
    control_snapshot(&s);
    if (s.settled_mask != CONTROL_ALL_AXES)
        return;
    int slot = pending_slot;
    pending_slot = -1;
    start_show(slot);
}

const char *app_save_config(void) {
    control_snapshot_t s;
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
    if (player_slot() >= 0)
        printf("show %d: %s\n", player_slot() + 1, player_state());
    else if (pending_slot >= 0)
        printf("show %d: waiting for the axes to be at rest%s\n", pending_slot + 1,
               control_outputs_on ? "" : " and the stop to be cleared");
    if (config.boot_show < SHOW_SLOTS)
        printf("boot show: %d\n", config.boot_show + 1);
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
    control_snapshot_t s;
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
    led_show();
}

static void stress_update(float t) {
    float in_cycle = fmodf(t, STRESS_CYCLE_S);
    bool pause = in_cycle > STRESS_CYCLE_S - STRESS_PAUSE_S;
    for (int i = 0; i < NUM_MOTORS; i++) {
        float period = STRESS_PERIOD_S + STRESS_PERIOD_STEP * (float)i;
        float phase = 2.0f * (float)M_PI * (t / period + (float)i / (float)NUM_MOTORS);
        control_cmd_t c = {
            .type = CMD_VELOCITY,
            .axes = (uint16_t)(1u << i),
            .f1 = pause ? 0.0f : STRESS_PEAK * trig_sinf(phase),
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
    sleep_us(100);  // a few ADC rounds before core 1 starts judging the ladder
    hbridge_start(motors, NUM_MOTORS);
    TRACE("PIO started\n");

    multicore_launch_core1(control_core1_main);
    TRACE("core 1 running\n");

    // Let V5 settle before driving the coils (no soft-start on V5).
    sleep_ms(POWER_UP_SETTLE_MS);
    control_energized = true;
    i2c_target_init();  // Qwiic, on boards that have it

    // Standalone: queue the boot show (it starts once outputs are on, so a
    // stop latched at power-up waits for Btn1 to clear it).
    if (config.boot_show < SHOW_SLOTS)
        pending_slot = selected_slot = config.boot_show;

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
            stress_update((float)(now - stress_start) * 1e-6f);
        }
        console_telemetry(now);
        player_poll(now);
        i2c_target_poll();
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
