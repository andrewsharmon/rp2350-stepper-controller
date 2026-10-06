// Bench firmware: sine microstepping on NUM_MOTORS motors (all following one
// commanded speed), with a constant-acceleration ramp, speed-dependent drive
// amplitude, the ADC monitor, the e-stop/button ladder and the status LED. Each motor's PWM is streamed to its PIO state machine by
// DMA from a ring buffer; core 1 keeps the rings topped up, core 0 takes single-key
// commands over USB CDC and watches core 1.
//
//   123<Enter>  set target speed (full steps/s; keeps direction)
//   + / -   target speed +/- 10%             r   reverse (ramps through 0)
//   s       stop (ramp to 0)                 g   go (back to last speed)
//   > / <   double / halve acceleration
//   ] / [   manual amplitude +/- 5%          0   manual amplitude 0 (brake)
//   a       automatic amplitude (speed curve + standstill hold)
//   ?       print status                     L   dump sine table
//   c       clear a latched e-stop / fault (line must be back to idle)
//   W       watchdog test: stall core 1 (outputs must drop to coast)
//   X       reboot
//
// Ladder buttons: Btn1 toggles stop/go, Btn2 reverses.

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

static hbridge_t motors[NUM_MOTORS];
static microstep_t steppers[NUM_MOTORS];

static volatile float target_speed = 100.0f;   // full steps/s, signed
static volatile float accel = 2000.0f;         // full steps/s^2
static volatile float current_speed;           // written by core 1 only
static volatile bool stall_core1;
static volatile float manual_amp = -1.0f;      // < 0: automatic
static volatile float current_amp;             // written by core 1 only
static volatile bool holding;                  // written by core 1 only
static volatile bool energized = false;
static volatile uint32_t core1_heartbeat;

// Ladder state lives on core 1 (it acts on stops); core 0 only reads it.
static ladder_t ladder;
static volatile bool outputs_on = true;        // false while a stop is latched
static volatile bool clear_request;
static volatile bool clear_refused;
// Core 1 load: min queued periods seen before a refill (ring headroom) and
// busy time, both reset when core 0 reads them.
static volatile uint32_t min_queued = HBRIDGE_RING_PERIODS;
static volatile uint32_t busy_us;

static float auto_amplitude(float speed) {
    float v = speed < 0.0f ? -speed : speed;
    if (v <= AMP_LOW_SPEED)
        return AMP_LOW;
    if (v >= AMP_HIGH_SPEED)
        return AMP_HIGH;
    return AMP_LOW + (AMP_HIGH - AMP_LOW) * (v - AMP_LOW_SPEED) / (AMP_HIGH_SPEED - AMP_LOW_SPEED);
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
            target_speed = 0.0f;
        }
    }
#else
    (void)last_eval;
#endif
    return !outputs_on;
}

static void core1_main(void) {
    int32_t max_duty = hbridge_max_duty();
    int32_t min_duty = hbridge_min_duty();

    float speed = 0.0f;
    float amp = 0.0f;
    uint32_t stopped_periods = 0;
    const uint32_t hold_periods = HOLD_DELAY_MS * (HBRIDGE_PWM_HZ / 1000);
    const float amp_step = AMP_SLEW_PER_SEC / (float)HBRIDGE_PWM_HZ;
    uint32_t last_eval = time_us_32();

    for (;;) {
        while (stall_core1)
            tight_loop_contents();  // watchdog test: stop the heartbeat
        core1_heartbeat++;

        if (clear_request) {
            clear_request = false;
            if (!outputs_on) {
                if (ladder_clear_stop(&ladder)) {
                    speed = 0.0f;
                    amp = 0.0f;
                    hbridge_restart(motors, NUM_MOTORS);
                    outputs_on = true;
                } else {
                    clear_refused = true;
                }
            }
        }
        if (poll_ladder(&last_eval)) {
            speed = 0.0f;
            current_speed = 0.0f;
            current_amp = 0.0f;
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
        float target = target_speed;
        float dv = accel / (float)HBRIDGE_PWM_HZ;  // speed change per period

        // Every motor gets the same number of periods, so they stay in step.
        for (uint32_t k = 0; k < n && !poll_ladder(&last_eval); k++) {
            if (speed < target)
                speed = speed + dv > target ? target : speed + dv;
            else if (speed > target)
                speed = speed - dv < target ? target : speed - dv;
            uint32_t inc = microstep_phase_inc(speed, HBRIDGE_PWM_HZ);

            if (speed == 0.0f && target == 0.0f)
                stopped_periods = stopped_periods < hold_periods ? stopped_periods + 1 : hold_periods;
            else
                stopped_periods = 0;
            float want = !energized ? 0.0f
                       : manual >= 0.0f ? manual
                       : stopped_periods >= hold_periods ? AMP_HOLD
                       : auto_amplitude(speed);
            if (amp < want)
                amp = amp + amp_step > want ? want : amp + amp_step;
            else if (amp > want)
                amp = amp - amp_step < want ? want : amp - amp_step;
            for (int i = 0; i < NUM_MOTORS; i++) {
                int32_t a, b;
                steppers[i].amplitude = amp;
                microstep_duties(&steppers[i], max_duty, min_duty, &a, &b);
                hbridge_write_period(&motors[i], a, b);
                steppers[i].phase += inc;
            }
        }
        busy_us += time_us_32() - t_start;
        current_speed = speed;
        current_amp = amp;
        holding = stopped_periods >= hold_periods && manual < 0.0f;
    }
}

static void print_status(void) {
    printf("target %.1f full steps/s (now %.1f), accel %.0f steps/s^2, amplitude %.2f (%s)%s\n",
           (double)target_speed, (double)current_speed, (double)accel, (double)current_amp,
           manual_amp >= 0.0f ? "manual" : holding ? "auto, holding" : "auto",
           outputs_on ? "" : " [STOPPED]");
}

static void print_monitor(void) {
    static uint32_t last_us;
    uint32_t now = time_us_32();
    uint32_t busy = busy_us, low = min_queued;
    busy_us = 0;
    min_queued = HBRIDGE_RING_PERIODS;
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
    printf("\n  %d motors (%d with pins), core 1 busy %.1f%%, ring low-water %lu/%u periods\n",
           NUM_MOTORS, NUM_MOTORS < BOARD_MOTORS_WITH_PINS ? NUM_MOTORS : BOARD_MOTORS_WITH_PINS,
           last_us ? 100.0 * busy / (double)(now - last_us) : 0.0,
           (unsigned long)low, HBRIDGE_RING_PERIODS - 1);
    last_us = now;
}

// Status LED: green running, blue holding, red blinking e-stop, magenta
// blinking driver fault, solid red watchdog trip, amber ladder bypassed.
static void update_led(bool tripped) {
    bool blink = (time_us_32() / 250000) & 1;
    if (tripped)
        led_set(255, 0, 0);
    else if (!outputs_on)
        led_set(blink ? 255 : 0, 0, ladder.stop_cause == LADDER_FAULT && blink ? 255 : 0);
    else if (holding)
        led_set(0, 0, 255);
    else
#ifdef LADDER_BYPASS
        led_set(255, 120, 0);
#else
        led_set(0, 255, 0);
#endif
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
    float last_target = target_speed;
    uint32_t entry = 0;  // digits typed so far
    bool entering = false;
    bool was_on = true;
    uint32_t btn1_seen = 0, btn2_seen = 0;

    for (;;) {
        update_led(tripped);
        if (adc_monitor_check())
            printf("ADC FIFO overflow: monitor restarted\n");

        bool on = outputs_on;
        if (was_on && !on)
            printf("STOP: %s, motor outputs off (press c to clear)\n", ladder_level_name(ladder.stop_cause));
        if (!was_on && on)
            printf("stop cleared\n");
        was_on = on;
        if (clear_refused) {
            clear_refused = false;
            printf("cannot clear: ladder still reads %s\n", ladder_level_name(ladder.raw_prev));
        }
        if (ladder.btn1_presses != btn1_seen) {
            btn1_seen = ladder.btn1_presses;
            if (target_speed != 0.0f) {
                last_target = target_speed;
                target_speed = 0.0f;
            } else {
                target_speed = last_target;
            }
            printf("btn1: ");
            print_status();
        }
        if (ladder.btn2_presses != btn2_seen) {
            btn2_seen = ladder.btn2_presses;
            target_speed = -target_speed;
            last_target = -last_target;
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
            printf("core 1 stalled: motor outputs off (press X to reboot)\n");
        }

        int c = getchar_timeout_us(1000);
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
                target_speed = target_speed < 0.0f ? -(float)entry : (float)entry;
                entry = 0;
                print_status();
                continue;
            }
            entry = 0;  // any other key cancels the entry
            printf(" (cancelled)\n");
        }

        switch (c) {
        case '+': target_speed *= 1.1f; break;
        case '-': target_speed /= 1.1f; break;
        case 'r': target_speed = -target_speed; break;
        case 's':
            if (target_speed != 0.0f)
                last_target = target_speed;
            target_speed = 0.0f;
            break;
        case 'g': target_speed = last_target; break;
        case '>': accel *= 2.0f; break;
        case '<': accel *= 0.5f; break;
        case ']':
        case '[': {
            float m = manual_amp >= 0.0f ? manual_amp : current_amp;
            m += c == ']' ? 0.05f : -0.05f;
            manual_amp = m > 1.0f ? 1.0f : m < 0.0f ? 0.0f : m;
            break;
        }
        case '0': manual_amp = 0.0f; break;
        case 'a': manual_amp = -1.0f; break;
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
