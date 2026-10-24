#pragma once

// Time-of-flight sensor service: a TMF8829 (MikroE LightRanger 14 Click) on
// the Qwiic port.
//
// The Qwiic port is the I2C target (i2c_target.h) until the sensor is
// switched on; then I2C1 becomes a host for the sensor, and switching it
// off hands the port back to the target. Start-up downloads the sensor's
// application in pieces from tof_poll, and so does every frame read, so
// the core 0 loop keeps running (a 48x32 frame is ~4.6 KB, 46 ms at 1 MHz).
//
// No INT or EN wiring is needed: tof_poll reads the interrupt status over
// I2C, and power-up uses the PON bit. The Click's EN pin (mikroBUS PWM)
// must be high.
//
// Each frame goes out as it is read: as binary protocol frames
// (protocol.h, TOF_*) when a host started the measurement, or as one
// summary line per measurement when the console did.

#include <stdbool.h>
#include <stdint.h>
#include "tmf8829.h"

typedef enum { TOF_OFF, TOF_BOOTING, TOF_IDLE, TOF_MEASURING, TOF_ERROR } tof_state_t;

typedef enum {
    TOF_ERR_NONE,
    TOF_ERR_NO_PORT,      // board has no Qwiic I2C
    TOF_ERR_NO_ANSWER,    // nothing at 0x41: power, EN, pull-ups, wiring
    TOF_ERR_CPU,          // CPU never got ready
    TOF_ERR_BOOTLOADER,   // couldn't reach or use the bootloader
    TOF_ERR_DOWNLOAD,     // image download failed
    TOF_ERR_APP,          // application didn't start or answer
    TOF_ERR_CONFIG,       // preset or configuration refused
    TOF_ERR_MEASURE,      // measurement wouldn't start or stop
    TOF_ERR_BUS,          // repeated bus errors while measuring
} tof_error_t;

typedef enum { TOF_STREAM_TEXT, TOF_STREAM_BINARY } tof_stream_t;

#define TOF_BUS_KHZ_DEFAULT 1000

typedef struct {
    tof_state_t state;
    tof_error_t error;
    uint8_t app_version[4];
    uint8_t chip_id, rev_id;
    uint32_t serial;
    uint32_t bus_khz;
    bool clock_correction;
    bool binary;            // frames go out as protocol frames
    uint8_t preset;         // last preset loaded (0: none yet)
    uint8_t download_pct;
    uint32_t frames, bad_frames, bus_errors;
    float clock_ratio;
    uint32_t frame_read_us; // last result frame, header to footer
} tof_info_t;

bool tof_available(void);
const char *tof_state_name(tof_state_t s);
const char *tof_error_name(tof_error_t e);

// Each returns NULL, or why it was refused.
const char *tof_on(bool reload_firmware);   // asynchronous: watch the state
void tof_off(void);
const char *tof_measure(bool on, tof_stream_t stream);
const char *tof_preset(uint8_t preset);
const char *tof_config_write(const uint8_t page[TMF8829_CFG_SIZE]);
const char *tof_set_bus_khz(uint32_t khz);  // 100-1000
void tof_set_clock_correction(bool on);

// The configuration page as last read back (valid once idle).
const uint8_t *tof_config(void);
void tof_get_info(tof_info_t *out);
void tof_print_status(void);

// Core 0 loop.
void tof_poll(uint32_t now_us);
