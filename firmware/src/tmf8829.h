#pragma once

// ams OSRAM TMF8829 multi-zone direct time-of-flight sensor (MikroE
// LightRanger 14 Click) on an I2C host port.
//
// The chip boots into a ROM bootloader; the measurement application
// (third_party/tmf8829) is written to its RAM and started from there. Once
// it runs, a preset command picks the zone layout (8x8 up to 48x32), the
// configuration page (0x22-0xdf) can be read and rewritten, and each
// measurement leaves one or two result frames in the chip's FIFO.
// Register sequences follow ams's reference driver
// (github.com/ams-OSRAM/tmf8829_driver_arduino, MIT).
//
// Everything here blocks for at most a few milliseconds per call; the
// firmware download and the FIFO reads come in pieces so the caller can
// spread them over its main loop. Core 0 only.

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "hardware/i2c.h"

#define TMF8829_ADDR       0x41

// Configuration page: registers 0x22-0xdf, read and written as a block.
#define TMF8829_CFG_FIRST  0x22
#define TMF8829_CFG_SIZE   190
// Offsets into the page (register - TMF8829_CFG_FIRST) the firmware uses.
#define TMF8829_CFG_PERIOD_MS       (0x22 - TMF8829_CFG_FIRST)   // u16
#define TMF8829_CFG_KILO_ITERATIONS (0x24 - TMF8829_CFG_FIRST)   // u16
#define TMF8829_CFG_FP_MODE         (0x26 - TMF8829_CFG_FIRST)
#define TMF8829_CFG_RESULT_FORMAT   (0x2a - TMF8829_CFG_FIRST)
#define TMF8829_CFG_DUMP_HISTOGRAMS (0x2b - TMF8829_CFG_FIRST)

// Presets (CMD_STAT commands): each sets up the zone layout and range and
// loads the configuration page.
enum {
    TMF8829_PRESET_8X8 = 0x40,
    TMF8829_PRESET_8X8_LONG_RANGE,
    TMF8829_PRESET_8X8_HIGH_ACCURACY,
    TMF8829_PRESET_16X16,
    TMF8829_PRESET_16X16_HIGH_ACCURACY,
    TMF8829_PRESET_32X32,
    TMF8829_PRESET_32X32_HIGH_ACCURACY,
    TMF8829_PRESET_48X32,
    TMF8829_PRESET_48X32_HIGH_ACCURACY,
};
#define TMF8829_PRESET_FIRST TMF8829_PRESET_8X8
#define TMF8829_PRESET_LAST  TMF8829_PRESET_48X32_HIGH_ACCURACY

// Focal plane modes (CFG_FP_MODE, and the low nibble of a frame's id).
enum { TMF8829_FP_8X8A, TMF8829_FP_8X8B, TMF8829_FP_16X16, TMF8829_FP_32X32, TMF8829_FP_32X32S,
       TMF8829_FP_48X32 };

// INT_STATUS / INT_ENAB bits.
#define TMF8829_INT_RESULTS    0x01
#define TMF8829_INT_MOTION     0x02
#define TMF8829_INT_PROXIMITY  0x04
#define TMF8829_INT_HISTOGRAMS 0x08

// Frames in the FIFO: a 5-byte pre-header (FIFOSTATUS, SYSTICK u32) read
// from 0xfa together with the frame header, then the rest of the frame
// from the FIFO register. header.payload counts the bytes after the first
// four (id, layout, payload).
#define TMF8829_PRE_HEADER_SIZE   5
#define TMF8829_FRAME_HEADER_SIZE 16
#define TMF8829_FRAME_FOOTER_SIZE 12
#define TMF8829_FRAME_EOF         0xe0f7
#define TMF8829_FID_MASK          0xf0
#define TMF8829_FID_RESULTS       0x10
#define TMF8829_FID_HISTOGRAMS    0x20
#define TMF8829_FID_REF_SPAD      0x30
#define TMF8829_FIFOSTATUS_TX_EMPTY 0x04   // pre-header byte 0: nothing queued for the host

// Device systick: 125 kHz.
#define TMF8829_TICKS_PER_MS 125

typedef enum {
    TMF8829_OK = 0,
    TMF8829_ERR_NACK = -1,      // nothing answers at the address
    TMF8829_ERR_TIMEOUT = -2,   // a status never came
    TMF8829_ERR_STATE = -3,     // the chip isn't in the expected application
} tmf8829_err_t;

typedef struct {
    i2c_inst_t *i2c;
    uint8_t addr;
    uint8_t app_id;           // 0x80 bootloader, 0x01 measurement application
    uint8_t app_version[4];   // id, major, minor, patch (application only)
    uint8_t chip_id, rev_id;
    uint32_t serial;
    uint32_t download_pos;    // bytes of the image written so far
} tmf8829_t;

void tmf8829_init(tmf8829_t *t, i2c_inst_t *i2c);

int tmf8829_read(tmf8829_t *t, uint8_t reg, uint8_t *buf, size_t n);
int tmf8829_write(tmf8829_t *t, uint8_t reg, const uint8_t *buf, size_t n);

// Power up (or wake from standby) and wait for the CPU. Fills app_id: the
// bootloader after a cold start, the application after a standby. With
// `to_bootloader`, an application that is already running is reset back
// into the bootloader.
int tmf8829_power_up(tmf8829_t *t, bool to_bootloader);

// Firmware download, from the bootloader. Begin, then call step until it
// returns 1 (done) or an error; `chunk` bytes per call. Finish starts the
// application and reads the device info.
int tmf8829_download_begin(tmf8829_t *t);
int tmf8829_download_step(tmf8829_t *t, size_t chunk);
int tmf8829_download_finish(tmf8829_t *t);
uint8_t tmf8829_download_pct(const tmf8829_t *t);

// Application commands.
int tmf8829_load_preset(tmf8829_t *t, uint8_t preset);
int tmf8829_config_read(tmf8829_t *t, uint8_t page[TMF8829_CFG_SIZE]);
int tmf8829_config_write(tmf8829_t *t, const uint8_t page[TMF8829_CFG_SIZE]);
int tmf8829_start(tmf8829_t *t);
int tmf8829_stop(tmf8829_t *t);
int tmf8829_standby(tmf8829_t *t);
int tmf8829_read_device_info(tmf8829_t *t);
uint8_t tmf8829_loaded_preset(tmf8829_t *t);   // CID_RID: the last preset or page loaded

// Interrupt bits that are set (and clears them).
int tmf8829_take_interrupts(tmf8829_t *t, uint8_t mask, uint8_t *set);

// Frame reads: the pre-header and header first, then the rest of the frame
// from the FIFO in any number of pieces.
int tmf8829_read_frame_header(tmf8829_t *t, uint8_t out[TMF8829_PRE_HEADER_SIZE + TMF8829_FRAME_HEADER_SIZE]);
int tmf8829_read_fifo(tmf8829_t *t, uint8_t *buf, size_t n);
int tmf8829_read_fifo_status(tmf8829_t *t, uint8_t *status);
