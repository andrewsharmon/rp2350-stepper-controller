#include "hbridge.h"

#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "pico/time.h"
#include <stdio.h>
#include "hbridge_encode.h"
#include "hbridge_pwm.pio.h"

// SM clock = sys / CLKDIV. At 150 MHz / 2 a 20 kHz period is 3750 clocks,
// which keeps every segment length within the 12-bit field.
#define HBRIDGE_CLKDIV 2.0f

// RP2350 TRANS_COUNT MODE 0xf: never stops. COUNT must still be non-zero or
// the channel completes at once without transferring.
#define DMA_ENDLESS ((0xfu << 28) | 1u)

// The read ring wraps on its size, so each ring must be aligned to it.
static uint32_t rings[HBRIDGE_MAX_MOTORS][HBRIDGE_RING_WORDS]
    __attribute__((aligned(1u << HBRIDGE_RING_BITS)));

static int pio_offset[3] = {-1, -1, -1};
static int32_t period_clocks;

static uint32_t read_index(const hbridge_t *hb) {
    uintptr_t addr = (uintptr_t)dma_channel_hw_addr(hb->dma_ch)->read_addr;
    return (uint32_t)((addr - (uintptr_t)hb->ring) / 4) & (HBRIDGE_RING_WORDS - 1);
}

void hbridge_init(hbridge_t *hb, uint index) {
    uint block = index / 4;
    PIO pio = block == 0 ? pio0 : block == 1 ? pio1 : pio2;

    period_clocks = (int32_t)(clock_get_hz(clk_sys) / HBRIDGE_CLKDIV / HBRIDGE_PWM_HZ);

    if (pio_offset[block] < 0) {
        if (block == 2)
            pio_set_gpio_base(pio, 16);
        pio_offset[block] = pio_add_program(pio, &hbridge_pwm_program);
    }

    hb->pio = pio;
    hb->sm = index % 4;
    hb->pin_base = 4 * index;
    hb->ring = rings[index];
    pio_sm_claim(pio, hb->sm);
    hbridge_pwm_program_init(pio, hb->sm, (uint)pio_offset[block], hb->pin_base, HBRIDGE_CLKDIV);

    uint32_t w0, w1;
    hbridge_encode_period(hbridge_max_duty(), 0, 0, &w0, &w1);
    for (uint i = 0; i < HBRIDGE_RING_WORDS; i += 2) {
        hb->ring[i] = w0;
        hb->ring[i + 1] = w1;
    }

    hb->dma_ch = (uint)dma_claim_unused_channel(true);
    dma_channel_config c = dma_channel_get_default_config(hb->dma_ch);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_ring(&c, false, HBRIDGE_RING_BITS);
    channel_config_set_dreq(&c, pio_get_dreq(pio, hb->sm, true));
    dma_channel_configure(hb->dma_ch, &c, &pio->txf[hb->sm], hb->ring, DMA_ENDLESS, true);

    // The DMA fills the (idle) SM's FIFO straight away; write from where it
    // stops, keeping the period boundary.
    absolute_time_t deadline = make_timeout_time_ms(10);
    while (!pio_sm_is_tx_fifo_full(pio, hb->sm)) {
        if (time_reached(deadline)) {
            dma_channel_hw_t *ch = dma_channel_hw_addr(hb->dma_ch);
            printf("hbridge: FIFO not filled: ch %u ctrl %08lx count %08lx read %08lx ring %p fifo %u\n",
                   hb->dma_ch, (unsigned long)ch->al1_ctrl, (unsigned long)ch->transfer_count,
                   (unsigned long)ch->read_addr, (void *)hb->ring, pio_sm_get_tx_fifo_level(pio, hb->sm));
            break;
        }
    }
    hb->wr = read_index(hb) & ~1u;
}

void hbridge_start(hbridge_t *hbs, uint count) {
    uint32_t masks[3] = {0, 0, 0};
    for (uint i = 0; i < count; i++)
        masks[pio_get_index(hbs[i].pio)] |= 1u << hbs[i].sm;
    for (uint b = 0; b < 3; b++)
        if (masks[b])
            pio_enable_sm_mask_in_sync(pio_get_instance(b), masks[b]);
}

int32_t hbridge_max_duty(void) {
    return period_clocks - 4 * HBRIDGE_SEGMENT_OVERHEAD;
}

int32_t hbridge_min_duty(void) {
    int64_t sm_hz = (int64_t)(clock_get_hz(clk_sys) / HBRIDGE_CLKDIV);
    return (int32_t)((sm_hz * HBRIDGE_MIN_PULSE_NS + 999999999) / 1000000000);
}

uint32_t hbridge_free_periods(const hbridge_t *hb) {
    // Words from the DMA read position up to wr are queued. Keep one period
    // of slack so a full ring (wr just behind the reader) never looks empty.
    uint32_t pending = (hb->wr - read_index(hb)) & (HBRIDGE_RING_WORDS - 1);
    if (pending > HBRIDGE_RING_WORDS - 2)
        return 0;  // reader overtook wr (underrun); the watchdog handles it
    return (HBRIDGE_RING_WORDS - 2 - pending) / 2;
}

void hbridge_write_period(hbridge_t *hb, int32_t duty_a, int32_t duty_b) {
    uint32_t w0, w1;
    hbridge_encode_period(hbridge_max_duty(), duty_a, duty_b, &w0, &w1);
    hb->ring[hb->wr] = w0;
    hb->ring[hb->wr + 1] = w1;
    hb->wr = (hb->wr + 2) & (HBRIDGE_RING_WORDS - 1);
}

void hbridge_safe_off(hbridge_t *hb) {
    for (uint i = 0; i < 4; i++) {
        gpio_put(hb->pin_base + i, 0);
        gpio_set_dir(hb->pin_base + i, GPIO_OUT);
        gpio_set_function(hb->pin_base + i, GPIO_FUNC_SIO);
    }
    pio_sm_set_enabled(hb->pio, hb->sm, false);
    dma_channel_abort(hb->dma_ch);
}
