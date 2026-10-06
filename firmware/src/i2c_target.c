#include "i2c_target.h"

#include "board_pins.h"

#ifndef BOARD_I2C_SDA_GPIO

bool i2c_target_init(void) {
    return false;
}

void i2c_target_poll(void) {
}

#else

#include <string.h>
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "pico/i2c_slave.h"
#include "frame.h"
#include "protocol.h"

#define I2C_BAUD 400000  // the target follows the master's clock; this sets timings

static uint8_t rx[1 + FRAME_MAX_PAYLOAD];   // request being written by the master
static volatile uint32_t rx_len;
static uint8_t req[1 + FRAME_MAX_PAYLOAD];  // complete request waiting for the loop
static volatile uint32_t req_len;
static volatile bool req_pending;
static uint8_t tx[2 + FRAME_MAX_PAYLOAD];   // [n][type][payload]
static volatile uint32_t tx_len, tx_pos;
static volatile bool tx_ready;

static void handler(i2c_inst_t *i2c, i2c_slave_event_t event) {
    switch (event) {
    case I2C_SLAVE_RECEIVE:
        while (i2c_get_read_available(i2c)) {
            uint8_t b = i2c_read_byte_raw(i2c);
            if (rx_len < sizeof rx)
                rx[rx_len++] = b;
        }
        break;
    case I2C_SLAVE_REQUEST:
        // Not ready (or nothing asked yet): a single 0. Past the end: 0xff.
        if (!tx_ready)
            i2c_write_byte_raw(i2c, 0);
        else
            i2c_write_byte_raw(i2c, tx_pos < tx_len ? tx[tx_pos++] : 0xff);
        break;
    case I2C_SLAVE_FINISH:
        if (rx_len > 0) {
            if (!req_pending) {  // one request at a time; extra writes are dropped
                memcpy(req, rx, rx_len);
                req_len = rx_len;
                tx_ready = false;
                req_pending = true;
            }
            rx_len = 0;
        }
        tx_pos = 0;  // every read starts from the length byte
        break;
    }
}

bool i2c_target_init(void) {
    gpio_set_function(BOARD_I2C_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(BOARD_I2C_SCL_GPIO, GPIO_FUNC_I2C);
    i2c_init(i2c1, I2C_BAUD);
    i2c_slave_init(i2c1, BOARD_I2C_TARGET_ADDR, handler);
    return true;
}

void i2c_target_poll(void) {
    if (!req_pending)
        return;
    size_t n = protocol_request(req[0], req + 1, req_len - 1, tx + 1, sizeof tx - 1);
    tx[0] = (uint8_t)n;   // n == 0: no reply (unknown requests are ACKed, so rare)
    tx_len = 1 + n;
    tx_pos = 0;
    tx_ready = true;
    req_pending = false;
}

#endif
