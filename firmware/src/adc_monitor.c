#include "adc_monitor.h"

#include "hardware/adc.h"
#include "hardware/dma.h"
#include "board_pins.h"

static const uint8_t input_gpio[MON_COUNT] = {
    [MON_LADDER] = BOARD_LADDER_GPIO,
    [MON_VMOT] = BOARD_VMOT_GPIO,
    [MON_BOARD_ID] = BOARD_ID_GPIO,
    [MON_CC1] = BOARD_CC1_GPIO,
    [MON_CC2] = BOARD_CC2_GPIO,
};

// Round robin visits channels in ascending order, so slot k of `samples`
// holds the k-th lowest enabled channel.
static volatile uint16_t samples[8];
static int8_t slot_of[MON_COUNT];
static uint32_t rr_mask;
static uint first_channel, n_channels;

static uint data_ch, ctrl_ch;
static volatile uint16_t *samples_addr = samples;

static void start(void) {
    adc_run(false);
    dma_channel_abort(data_ch);
    dma_channel_abort(ctrl_ch);
    adc_fifo_drain();
    adc_hw->fcs |= ADC_FCS_OVER_BITS | ADC_FCS_UNDER_BITS;  // write-1-to-clear
    adc_select_input(first_channel);
    adc_set_round_robin(rr_mask);
    dma_channel_set_write_addr(data_ch, (void *)samples, false);
    dma_channel_set_trans_count(data_ch, n_channels, true);
    adc_run(true);
}

void adc_monitor_init(void) {
    adc_init();
    for (int i = 0; i < MON_COUNT; i++) {
        if (input_gpio[i] == ADC_NONE)
            continue;
        adc_gpio_init(input_gpio[i]);
        rr_mask |= 1u << (input_gpio[i] - ADC_BASE_PIN);
    }
    first_channel = (uint)__builtin_ctz(rr_mask);
    n_channels = (uint)__builtin_popcount(rr_mask);
    for (int i = 0; i < MON_COUNT; i++) {
        slot_of[i] = -1;
        if (input_gpio[i] == ADC_NONE)
            continue;
        uint ch = input_gpio[i] - ADC_BASE_PIN;
        slot_of[i] = (int8_t)__builtin_popcount(rr_mask & ((1u << ch) - 1));
    }

    // 48 MHz / 96 cycles = 500 ksps total: a full round of 5 inputs every 10 us.
    adc_set_clkdiv(0);
    adc_fifo_setup(true, true, 1, true, false);

    // data_ch copies one round into `samples`, then chains to ctrl_ch, which
    // rewrites data_ch's write address (and retriggers it) for the next round.
    data_ch = (uint)dma_claim_unused_channel(true);
    ctrl_ch = (uint)dma_claim_unused_channel(true);

    dma_channel_config c = dma_channel_get_default_config(data_ch);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_16);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_dreq(&c, DREQ_ADC);
    channel_config_set_chain_to(&c, ctrl_ch);
    dma_channel_configure(data_ch, &c, (void *)samples, &adc_hw->fifo, n_channels, false);

    c = dma_channel_get_default_config(ctrl_ch);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, false);
    dma_channel_configure(ctrl_ch, &c, &dma_hw->ch[data_ch].al2_write_addr_trig,
                          &samples_addr, 1, false);

    start();
}

bool adc_monitor_present(mon_input_t in) {
    return slot_of[in] >= 0;
}

uint32_t adc_monitor_pin_mv(mon_input_t in) {
    if (slot_of[in] < 0)
        return 0;
    // 12-bit result; the FIFO's error flag (bit 15) is masked off.
    return ((uint32_t)(samples[slot_of[in]] & 0xfff) * 3300u) / 4095u;
}

uint32_t adc_monitor_vmot_mv(void) {
    return (uint32_t)((float)adc_monitor_pin_mv(MON_VMOT) * BOARD_VMOT_RATIO);
}

bool adc_monitor_check(void) {
    if (!(adc_hw->fcs & ADC_FCS_OVER_BITS))
        return false;
    start();
    return true;
}
