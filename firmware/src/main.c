// Milestone 2 bring-up: sine microstepping on NUM_MOTORS motors, with a
// constant-acceleration ramp to the commanded speed. Each motor's PWM is streamed to its PIO state machine by DMA from
// a ring buffer; core 1 keeps the rings topped up, core 0 takes single-key
// commands over USB CDC and watches core 1.
//
//   123<Enter>  set target speed (full steps/s; keeps direction)
//   + / -   target speed +/- 10%             r   reverse (ramps through 0)
//   s       stop (ramp to 0)                 g   go (back to last speed)
//   > / <   double / halve acceleration
//   ] / [   amplitude +/- 5%                 0   amplitude 0 (brake)
//   ?       print status                     L   dump sine table
//   W       watchdog test: stall core 1 (outputs must drop to coast)
//   X       reboot

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/watchdog.h"

#include "hbridge.h"
#include "microstep.h"

#ifndef NUM_MOTORS
#define NUM_MOTORS 1
#endif

#define POWER_UP_SETTLE_MS 100
// Each ring holds 3.2 ms; trip well before a stalled producer lets it wrap.
#define WATCHDOG_US 1500

static hbridge_t motors[NUM_MOTORS];
static microstep_t steppers[NUM_MOTORS];

static volatile float target_speed = 100.0f;   // full steps/s, signed
static volatile float accel = 2000.0f;         // full steps/s^2
static volatile float current_speed;           // written by core 1 only
static volatile bool stall_core1;
static volatile float amplitude = 0.3f;
static volatile bool energized = false;
static volatile uint32_t core1_heartbeat;

static void core1_main(void) {
    int32_t max_duty = hbridge_max_duty();
    int32_t min_duty = hbridge_min_duty();

    float speed = 0.0f;

    for (;;) {
        while (stall_core1)
            tight_loop_contents();  // watchdog test: stop the heartbeat
        core1_heartbeat++;

        uint32_t n = HBRIDGE_RING_PERIODS;
        for (int i = 0; i < NUM_MOTORS; i++) {
            uint32_t f = hbridge_free_periods(&motors[i]);
            if (f < n)
                n = f;
        }

        float amp = energized ? amplitude : 0.0f;
        float target = target_speed;
        float dv = accel / (float)HBRIDGE_PWM_HZ;  // speed change per period

        // Every motor gets the same number of periods, so they stay in step.
        for (uint32_t k = 0; k < n; k++) {
            if (speed < target)
                speed = speed + dv > target ? target : speed + dv;
            else if (speed > target)
                speed = speed - dv < target ? target : speed - dv;
            uint32_t inc = microstep_phase_inc(speed, HBRIDGE_PWM_HZ);
            for (int i = 0; i < NUM_MOTORS; i++) {
                int32_t a, b;
                steppers[i].amplitude = amp;
                microstep_duties(&steppers[i], max_duty, min_duty, &a, &b);
                hbridge_write_period(&motors[i], a, b);
                steppers[i].phase += inc;
            }
        }
        current_speed = speed;
    }
}

static void print_status(void) {
    printf("target %.1f full steps/s (now %.1f), accel %.0f steps/s^2, amplitude %.2f\n",
           (double)target_speed, (double)current_speed, (double)accel, (double)amplitude);
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
        hbridge_init(&motors[i], (uint)i);
    }
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

    for (;;) {
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
        case ']': amplitude = amplitude + 0.05f > 1.0f ? 1.0f : amplitude + 0.05f; break;
        case '[': amplitude = amplitude - 0.05f < 0.0f ? 0.0f : amplitude - 0.05f; break;
        case '0': amplitude = 0.0f; break;
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
