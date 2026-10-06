// Bench firmware: sine microstepping on NUM_MOTORS motors, each ramping at a
// constant acceleration to its own target speed, with speed-dependent drive
// amplitude, the ADC monitor, the e-stop/button ladder and the status LEDs.
// Each motor's PWM is streamed to its PIO state machine by DMA from a ring
// buffer; core 1 keeps the rings topped up, core 0 takes single-key commands
// over USB CDC, runs the stress test and watches core 1.
//
// Speed commands apply to every axis:
//   123<Enter>  set target speed (full steps/s; each axis keeps its direction)
//   + / -   target speed +/- 10%             r   reverse (ramps through 0)
//   s       stop (ramp to 0)                 g   go (back to last speeds)
//   > / <   double / halve acceleration
//   ] / [   manual amplitude +/- 5%          0   manual amplitude 0 (brake)
//   a       automatic amplitude (speed curve + standstill hold)
//   T       stress test on/off: every axis sweeps its own speed wave
//   ?       print status                     L   dump sine table
//   c       clear a latched e-stop / fault (line must be back to idle)
//   W       watchdog test: stall core 1 (outputs must drop to coast)
//   X       reboot
//
// Ladder buttons: Btn1 toggles stop/go, Btn2 reverses.
//
// LEDs: pixel 0 is the general status, pixels 1-10 show each axis (green
// forward, blue reverse, brighter = faster; dim white holding; red stopped).

#include <math.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/watchdog.h"

#include "adc_monitor.h"
#include "board_pins.h"
#include "hbridge.h"
#include "ladder.h"
#include "led.h"
#include "microstep.h"

#ifndef NUM_MOTORS
#define NUM_MOTORS 10
#endif

#define POWER_UP_SETTLE_MS 100
// Each ring holds 3.2 ms; trip well before a stalled producer lets it wrap.
#define WATCHDOG_US 1500

// Automatic drive amplitude (fraction of full PWM span), tuned on the bench
// 8 mm stepper at 5 V: flat up to AMP_LOW_SPEED, linear up to the cap at
// AMP_HIGH_SPEED to make up for back-EMF. Drops to AMP_HOLD after the motor
// has been stopped for HOLD_DELAY_MS, since standstill heats the coils most.
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

// Stress test: axis i sweeps a sine of speed with its own period and phase,
// so the speed pattern rolls along the LED strip. Every STRESS_CYCLE_S all
// axes pause for STRESS_PAUSE_S to exercise stop, hold and restart.
#define STRESS_PEAK        1500.0f  // full steps/s; holds at 60% on the bench motor
#define STRESS_PERIOD_S    6.0f     // axis 1; later axes slightly slower
#define STRESS_PERIOD_STEP 0.7f
#define STRESS_CYCLE_S     20.0f
#define STRESS_PAUSE_S     2.5f
#define STRESS_UPDATE_US   10000
#define LED_UPDATE_US      20000

static hbridge_t motors[NUM_MOTORS];
static microstep_t steppers[NUM_MOTORS];

static volatile float axis_target[NUM_MOTORS];  // full steps/s, signed
static volatile float accel = 2000.0f;          // full steps/s^2
static volatile bool stall_core1;
static volatile float manual_amp = -1.0f;       // < 0: automatic
static volatile bool energized = false;
static volatile uint32_t core1_heartbeat;

// Written by core 1 only, after each refill.
static volatile float axis_speed[NUM_MOTORS];
static volatile float axis_amp[NUM_MOTORS];
static volatile uint32_t holding_mask;

// Ladder state lives on core 1 (it acts on stops); core 0 only reads it.
static ladder_t ladder;
static volatile bool outputs_on = true;         // false while a stop is latched
static volatile bool clear_request;
static volatile bool clear_refused;
// Core 1 load: min queued periods seen before a refill (ring headroom) and
// busy time, both reset when core 0 reads them.
static volatile uint32_t min_queued = HBRIDGE_RING_PERIODS;
static volatile uint32_t busy_us;
// Core 0 idle: time spent waiting for serial input (USB interrupts that run
// during the wait count as idle, so core 0 load reads slightly low).
static uint32_t core0_idle_us;

static inline float auto_amplitude(float speed) {
    const float slope = (AMP_HIGH - AMP_LOW) / (AMP_HIGH_SPEED - AMP_LOW_SPEED);
    float v = speed < 0.0f ? -speed : speed;
    if (v <= AMP_LOW_SPEED)
        return AMP_LOW;
    if (v >= AMP_HIGH_SPEED)
        return AMP_HIGH;
    return AMP_LOW + slope * (v - AMP_LOW_SPEED);
}

static void set_all_targets(float v) {
    for (int i = 0; i < NUM_MOTORS; i++)
        axis_target[i] = v;
}

// Returns true if a stop is latched (outputs are off).
static bool __time_critical_func(poll_ladder)(uint32_t *last_eval) {
#ifndef LADDER_BYPASS
    uint32_t now = time_us_32();
    if (now - *last_eval >= LADDER_EVAL_US) {
        *last_eval = now;
        if (ladder_update(&ladder, adc_monitor_pin_mv(MON_LADDER), now) && outputs_on) {
            for (int i = 0; i < NUM_MOTORS; i++)
                hbridge_safe_off(&motors[i]);
            outputs_on = false;
            set_all_targets(0.0f);
        }
    }
#else
    (void)last_eval;
#endif
    return !outputs_on;
}

static void __time_critical_func(core1_main)(void) {
    int32_t max_duty = hbridge_max_duty();
    int32_t min_duty = hbridge_min_duty();

    float speed[NUM_MOTORS] = {0};
    float amp[NUM_MOTORS] = {0};
    uint32_t stopped_periods[NUM_MOTORS] = {0};
    const uint32_t hold_periods = HOLD_DELAY_MS * (HBRIDGE_PWM_HZ / 1000);
    const float amp_step = AMP_SLEW_PER_SEC / (float)HBRIDGE_PWM_HZ;
    // Phase advance per period per full step/s (one cycle = 4 full steps).
    const float phase_per_sps = 4294967296.0f / 4.0f / (float)HBRIDGE_PWM_HZ;
    uint32_t last_eval = time_us_32();

    for (;;) {
        while (stall_core1)
            tight_loop_contents();  // watchdog test: stop the heartbeat
        core1_heartbeat++;

        if (clear_request) {
            clear_request = false;
            if (!outputs_on) {
                if (ladder_clear_stop(&ladder)) {
                    for (int i = 0; i < NUM_MOTORS; i++)
                        speed[i] = amp[i] = 0.0f;
                    hbridge_restart(motors, NUM_MOTORS);
                    outputs_on = true;
                } else {
                    clear_refused = true;
                }
            }
        }
        if (poll_ladder(&last_eval)) {
            for (int i = 0; i < NUM_MOTORS; i++)
                speed[i] = amp[i] = axis_speed[i] = axis_amp[i] = 0.0f;
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
        if (queued < min_queued)
            min_queued = queued;
        uint32_t t_start = time_us_32();

        float manual = manual_amp;
        float dv = accel / (float)HBRIDGE_PWM_HZ;  // speed change per period
        float target[NUM_MOTORS];
        for (int i = 0; i < NUM_MOTORS; i++)
            target[i] = axis_target[i];

        // Every ring gets the same number of periods, so the axes stay
        // aligned in time.
        for (uint32_t k = 0; k < n && !poll_ladder(&last_eval); k++) {
            for (int i = 0; i < NUM_MOTORS; i++) {
                float v = speed[i], t = target[i];
                if (v < t)
                    v = v + dv > t ? t : v + dv;
                else if (v > t)
                    v = v - dv < t ? t : v - dv;
                speed[i] = v;

                if (v == 0.0f && t == 0.0f)
                    stopped_periods[i] += stopped_periods[i] < hold_periods;
                else
                    stopped_periods[i] = 0;
                float want = !energized ? 0.0f
                           : manual >= 0.0f ? manual
                           : stopped_periods[i] >= hold_periods ? AMP_HOLD
                           : auto_amplitude(v);
                float a = amp[i];
                if (a < want)
                    a = a + amp_step > want ? want : a + amp_step;
                else if (a > want)
                    a = a - amp_step < want ? want : a - amp_step;
                amp[i] = a;

                int32_t duty_a, duty_b;
                steppers[i].amplitude = a;
                microstep_duties(&steppers[i], max_duty, min_duty, &duty_a, &duty_b);
                hbridge_write_period(&motors[i], duty_a, duty_b);
                float p = v * phase_per_sps;
                steppers[i].phase += (uint32_t)(int32_t)(p >= 0.0f ? p + 0.5f : p - 0.5f);
            }
        }
        busy_us += time_us_32() - t_start;

        uint32_t hold = 0;
        for (int i = 0; i < NUM_MOTORS; i++) {
            axis_speed[i] = speed[i];
            axis_amp[i] = amp[i];
            if (stopped_periods[i] >= hold_periods && manual < 0.0f)
                hold |= 1u << i;
        }
        holding_mask = hold;
    }
}

static void print_status(void) {
    printf("axis 1: target %.1f full steps/s (now %.1f), accel %.0f steps/s^2, amplitude %.2f (%s)%s\n",
           (double)axis_target[0], (double)axis_speed[0], (double)accel, (double)axis_amp[0],
           manual_amp >= 0.0f ? "manual" : (holding_mask & 1) ? "auto, holding" : "auto",
           outputs_on ? "" : " [STOPPED]");
}

static void print_monitor(void) {
    static uint32_t last_us;
    uint32_t now = time_us_32();
    uint32_t busy = busy_us, low = min_queued, idle0 = core0_idle_us;
    busy_us = 0;
    core0_idle_us = 0;
    min_queued = HBRIDGE_RING_PERIODS;

    printf("  speeds:");
    for (int i = 0; i < NUM_MOTORS; i++)
        printf(" %.0f", (double)axis_speed[i]);
    printf("\n");

    uint32_t ladder_mv = adc_monitor_pin_mv(MON_LADDER);
    printf("  ladder %lu mV (%s%s)  vmot %lu mV  board id %lu mV",
           (unsigned long)ladder_mv, ladder_level_name(ladder_classify(ladder_mv)),
#ifdef LADDER_BYPASS
           ", BYPASSED",
#else
           ladder.stop_latched ? ", stop latched" : "",
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
// Pixels 1-10: per axis, see the header comment.
static void update_leds(bool tripped) {
    bool blink = (time_us_32() / 250000) & 1;
    bool all_holding = holding_mask == (1u << NUM_MOTORS) - 1;
    if (tripped)
        led_set(LED_STATUS, 255, 0, 0);
    else if (!outputs_on)
        led_set(LED_STATUS, blink ? 255 : 0, 0, ladder.stop_cause == LADDER_FAULT && blink ? 255 : 0);
    else if (all_holding)
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
        } else if (!outputs_on) {
            led_set(px, blink ? 120 : 0, 0, 0);
        } else if (holding_mask & (1u << i)) {
            led_set(px, 40, 40, 40);
        } else {
            float v = axis_speed[i];
            float mag = fabsf(v) / AMP_HIGH_SPEED;
            uint8_t level = (uint8_t)(30.0f + 225.0f * (mag > 1.0f ? 1.0f : mag));
            if (v > 0.5f)
                led_set(px, 0, level, 0);
            else if (v < -0.5f)
                led_set(px, 0, 0, level);
            else
                led_set(px, 20, 20, 20);  // stopped, not yet holding
        }
    }
    led_show();
}

static void stress_update(float t) {
    float in_cycle = fmodf(t, STRESS_CYCLE_S);
    for (int i = 0; i < NUM_MOTORS; i++) {
        if (in_cycle > STRESS_CYCLE_S - STRESS_PAUSE_S) {
            axis_target[i] = 0.0f;
            continue;
        }
        float period = STRESS_PERIOD_S + STRESS_PERIOD_STEP * (float)i;
        float phase = 2.0f * (float)M_PI * (t / period + (float)i / (float)NUM_MOTORS);
        axis_target[i] = STRESS_PEAK * sinf(phase);
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

    set_all_targets(100.0f);
    for (int i = 0; i < NUM_MOTORS; i++) {
        TRACE("init motor %d\n", i + 1);
        hbridge_init(&motors[i], (uint)i, i < BOARD_MOTORS_WITH_PINS);
    }
    led_init();  // after the motors: PIO2's GPIO base is set there
    adc_monitor_init();
    ladder_reset(&ladder, time_us_32());
    sleep_us(100);  // a few ADC rounds before core 1 starts judging the ladder
    hbridge_start(motors, NUM_MOTORS);
    TRACE("PIO started\n");

    multicore_launch_core1(core1_main);
    TRACE("core 1 running\n");

    // Let V5 settle before driving the coils (no soft-start on V5).
    sleep_ms(POWER_UP_SETTLE_MS);
    energized = true;

    uint32_t last_beat = core1_heartbeat;
    absolute_time_t beat_seen = get_absolute_time();
    bool tripped = false;
    float last_target[NUM_MOTORS];
    for (int i = 0; i < NUM_MOTORS; i++)
        last_target[i] = axis_target[i];
    uint32_t entry = 0;  // digits typed so far
    bool entering = false;
    bool was_on = true;
    uint32_t btn1_seen = 0, btn2_seen = 0;
    bool stress = false;
    uint32_t stress_start = 0, last_stress = 0, last_led = 0;

    for (;;) {
        uint32_t now = time_us_32();
        if (now - last_led >= LED_UPDATE_US) {
            last_led = now;
            update_leds(tripped);
        }
        if (stress && now - last_stress >= STRESS_UPDATE_US) {
            last_stress = now;
            stress_update((float)(now - stress_start) * 1e-6f);
        }
        if (adc_monitor_check())
            printf("ADC FIFO overflow: monitor restarted\n");

        bool on = outputs_on;
        if (was_on && !on) {
            stress = false;
            printf("STOP: %s, motor outputs off (press c to clear)\n", ladder_level_name(ladder.stop_cause));
        }
        if (!was_on && on)
            printf("stop cleared\n");
        was_on = on;
        if (clear_refused) {
            clear_refused = false;
            printf("cannot clear: ladder still reads %s\n", ladder_level_name(ladder.raw_prev));
        }
        if (ladder.btn1_presses != btn1_seen) {
            btn1_seen = ladder.btn1_presses;
            stress = false;
            bool moving = false;
            for (int i = 0; i < NUM_MOTORS; i++)
                moving |= axis_target[i] != 0.0f;
            for (int i = 0; i < NUM_MOTORS; i++) {
                if (moving) {
                    last_target[i] = axis_target[i];
                    axis_target[i] = 0.0f;
                } else {
                    axis_target[i] = last_target[i];
                }
            }
            printf("btn1: ");
            print_status();
        }
        if (ladder.btn2_presses != btn2_seen) {
            btn2_seen = ladder.btn2_presses;
            for (int i = 0; i < NUM_MOTORS; i++) {
                axis_target[i] = -axis_target[i];
                last_target[i] = -last_target[i];
            }
            printf("btn2: ");
            print_status();
        }

        uint32_t beat = core1_heartbeat;
        if (beat != last_beat) {
            last_beat = beat;
            beat_seen = get_absolute_time();
        } else if (!tripped && absolute_time_diff_us(beat_seen, get_absolute_time()) > WATCHDOG_US) {
            for (int i = 0; i < NUM_MOTORS; i++)
                hbridge_safe_off(&motors[i]);
            tripped = true;
            stress = false;
            printf("core 1 stalled: motor outputs off (press X to reboot)\n");
        }

        uint32_t wait_start = time_us_32();
        int c = getchar_timeout_us(1000);
        core0_idle_us += time_us_32() - wait_start;
        if (c == PICO_ERROR_TIMEOUT)
            continue;

        if (c >= '0' && c <= '9' && (entering || c != '0')) {
            entry = entry * 10 + (uint32_t)(c - '0');
            if (entry > 100000)
                entry = 100000;
            entering = true;
            putchar(c);
            continue;
        }
        if (entering) {
            entering = false;
            if (c == '\r' || c == '\n') {
                putchar('\n');
                stress = false;
                for (int i = 0; i < NUM_MOTORS; i++)
                    axis_target[i] = axis_target[i] < 0.0f ? -(float)entry : (float)entry;
                entry = 0;
                print_status();
                continue;
            }
            entry = 0;  // any other key cancels the entry
            printf(" (cancelled)\n");
        }

        // Manual speed commands take over from the stress test.
        if (c == '+' || c == '-' || c == 'r' || c == 's' || c == 'g')
            stress = false;

        switch (c) {
        case '+':
        case '-':
            for (int i = 0; i < NUM_MOTORS; i++)
                axis_target[i] = c == '+' ? axis_target[i] * 1.1f : axis_target[i] / 1.1f;
            break;
        case 'r':
            for (int i = 0; i < NUM_MOTORS; i++)
                axis_target[i] = -axis_target[i];
            break;
        case 's':
            for (int i = 0; i < NUM_MOTORS; i++) {
                if (axis_target[i] != 0.0f)
                    last_target[i] = axis_target[i];
                axis_target[i] = 0.0f;
            }
            break;
        case 'g':
            for (int i = 0; i < NUM_MOTORS; i++)
                axis_target[i] = last_target[i];
            break;
        case '>': accel *= 2.0f; break;
        case '<': accel *= 0.5f; break;
        case ']':
        case '[': {
            float m = manual_amp >= 0.0f ? manual_amp : axis_amp[0];
            m += c == ']' ? 0.05f : -0.05f;
            manual_amp = m > 1.0f ? 1.0f : m < 0.0f ? 0.0f : m;
            break;
        }
        case '0': manual_amp = 0.0f; break;
        case 'a': manual_amp = -1.0f; break;
        case 'T':
            stress = !stress && outputs_on;
            if (stress) {
                stress_start = last_stress = time_us_32();
                printf("stress test on: %d axes, peak %.0f full steps/s\n", NUM_MOTORS,
                       (double)STRESS_PEAK);
            } else {
                set_all_targets(0.0f);
                printf("stress test off\n");
            }
            continue;
        case 'c':
            clear_request = true;
            sleep_ms(5);
            continue;
        case '?':
            print_status();
            print_monitor();
            continue;
        case 'L':  // dump the sine table
            for (uint32_t i = 0; i < (1u << MICROSTEP_LUT_BITS); i++)
                printf("%d%c", microstep_sine_lut[i], (i & 15) == 15 ? '\n' : ' ');
            continue;
        case 'W':
            printf("watchdog test: stalling core 1\n");
            stall_core1 = true;
            continue;
        case 'X':
            printf("rebooting\n");
            sleep_ms(50);
            watchdog_reboot(0, 0, 0);
            for (;;)
                tight_loop_contents();
        default: break;
        }
        print_status();
    }
}
