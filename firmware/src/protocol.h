#pragma once

// Binary protocol over the USB serial port, alongside the text console. The
// same requests also travel over the Qwiic I2C target without framing
// (i2c_target.h).
// Framing is in frame.h; this file is the message spec, mirrored by
// tools/stepperctl/protocol.py. All fields little-endian. Positions are
// int64 in units of 2^-30 full step; speeds are float32 full steps/s.
// `axes` is a bit mask (bit 0 = axis 1); group ids are 0-3.
//
// Every host request gets an ACK echoing its frame seq. An ACK of OK means
// the command was queued for core 1 (cmd_seq is its number there; status
// and telemetry report the last applied one). Core 1 can still refuse it
// (e.g. a move on a grouped axis); that shows in STATUS.rejected.
//
// Host -> device                         payload
//   0x01 PING                            -                   -> PONG
//   0x02 STATUS_REQ                      -                   -> STATUS
//   0x10 MOVE                            axes u16, pos i64
//   0x11 MOVE_REL                        axes u16, delta i64
//   0x12 VELOCITY                        axes u16, vel f32
//   0x13 JOG                             axes u16, vel f32, timeout_ms u16
//   0x14 STOP                            axes u16
//   0x15 SET_POS                         axes u16, pos i64
//   0x16 LIMITS                          axes u16, vmax f32, amax f32 (<= 0: unchanged)
//   0x17 PROFILE                         axes u16, profile u8, jerk_ms u16
//   0x18 PVT_POINT                       axes u16, pos i64, vel f32, ms u16
//   0x19 PVT_START                       axes u16
//   0x20 GROUP_CREATE                    id u8, axes u16, feed f32, corner_ms f32 (<= 0: default)
//   0x21 GROUP_RELEASE                   id u8
//   0x22 GROUP_LINE                      id u8, feed f32 (<= 0: group default), n u8, pos i64 x n
//   0x23 GROUP_ARC                       id u8, feed f32, tol f32, cx i64, cy i64, angle f64 (rad)
//   0x24 GROUP_HOLD                      id u8, hold u8 (0 resumes)
//   0x25 GROUP_STOP                      id u8
//   0x30 AMPLITUDE                       amp f32 (< 0 automatic, else 0..1)
//   0x31 CLEAR_STOP                      -
//   0x32 CONFIG_SAVE                     -   (ACK BAD_REQUEST if any axis is moving)
//   0x33 DRIVE                           axes u16, amp_low f32, amp_high f32, amp_hold f32,
//                                        low_speed f32, high_speed f32, flags u8
//                                        (amplitudes 0..1; flags: 1 reverse, 2 swap coils)
//   0x34 BOOT_SHOW                       slot u8 (0xff: none); save to keep it
//   0x40 TELEMETRY                       rate_hz u16 (0 off), axes u16, fields u8, sys u8, events u8
//   0x50 SHOW_BEGIN                      slot u8 (0-3), len u32: start an upload
//   0x51 SHOW_DATA                       offset u32, bytes...
//   0x52 SHOW_END                        -   validate and write to flash (axes at rest);
//                                            ACK BAD_REQUEST if invalid or busy
//   0x53 SHOW_RUN                        slot u8   (ACK BAD_REQUEST if it can't run)
//   0x54 SHOW_STOP                       -
//   0x55 SHOW_LIST                       -                   -> SHOW_INFO
//   0x56 SHOW_ERASE                      slot u8
//
// Device -> host
//   0x80 ACK        req_type u8, result u8 (PROTO_OK...), cmd_seq u32
//   0x81 PONG       version u8, axes u8, groups u8, tick_hz u16, pwm_hz u16,
//                   units_log2 u8, firmware name (rest, ASCII)
//   0x82 STATUS     tick u32, flags u8 (bit0 outputs on, bit1 e-stop, bit2 driver fault),
//                   last_seq u32, rejected u32, pvt_underruns u32, vmot_mv u16, ladder_mv u16,
//                   per axis: pos i64, vel f32, mode u8, amp_pct u8,
//                             flags u8 (bit0 holding, bit1 settled), pvt_queue u8, group i8,
//                   per group (4): flags u8 (bit0 active, bit1 running, bit2 held, bit3 arc),
//                             members u16, queued u8, path_vel f32, segments_done u32
//   0x90 TELEMETRY  line u32, tick u32,
//                   system fields in bit order: seq u32, vmot_mv u16, ladder_mv u16, stop u8,
//                   then per selected axis, per field in bit order:
//                             pos i64, vel f32, mode u8, amp_pct u8, pvt_queue u8
//   0x83 SHOW_INFO  playing u8 (slot, 0xff none), then per slot (4): valid u8,
//                   name char[16], duration_ms u32, loop u8, n_tracks u8
//   0x91 EVENT      tick u32, kind u8 (PROTO_EV_...), axis u8 (1-based, 0 = none), pos i64
//
// Telemetry field bits: TF_POS 1, TF_VEL 2, TF_MODE 4, TF_AMP 8, TF_Q 16.
// System field bits:    TS_SEQ 1, TS_VMOT 2, TS_LADDER 4, TS_STOP 8.

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "control.h"

#define PROTO_VERSION 1

enum {
    PROTO_PING = 0x01, PROTO_STATUS_REQ = 0x02,
    PROTO_MOVE = 0x10, PROTO_MOVE_REL, PROTO_VELOCITY, PROTO_JOG, PROTO_STOP, PROTO_SET_POS,
    PROTO_LIMITS, PROTO_PROFILE, PROTO_PVT_POINT, PROTO_PVT_START,
    PROTO_GROUP_CREATE = 0x20, PROTO_GROUP_RELEASE, PROTO_GROUP_LINE, PROTO_GROUP_ARC,
    PROTO_GROUP_HOLD, PROTO_GROUP_STOP,
    PROTO_AMPLITUDE = 0x30, PROTO_CLEAR_STOP, PROTO_CONFIG_SAVE, PROTO_DRIVE,
    PROTO_BOOT_SHOW,
    PROTO_TELEMETRY = 0x40,
    PROTO_SHOW_BEGIN = 0x50, PROTO_SHOW_DATA, PROTO_SHOW_END, PROTO_SHOW_RUN, PROTO_SHOW_STOP,
    PROTO_SHOW_LIST, PROTO_SHOW_ERASE,
    PROTO_ACK = 0x80, PROTO_PONG, PROTO_STATUS, PROTO_SHOW_INFO,
    PROTO_TELEM = 0x90, PROTO_EVENT,
};

enum { PROTO_OK, PROTO_QUEUE_FULL, PROTO_BAD_REQUEST, PROTO_UNKNOWN };

enum { PROTO_EV_DONE = 1, PROTO_EV_UNDERRUN, PROTO_EV_ESTOP, PROTO_EV_FAULT, PROTO_EV_CLEAR };

// Feed one received byte: frame bytes are handled here, everything else
// goes to the text console.
void protocol_input(int c);

// Run one request without framing (the I2C target's transport) and capture
// the reply into `reply` as [type, payload...]. Returns the reply length
// (0 if there is none). Core 0 only.
size_t protocol_request(uint8_t type, const uint8_t *payload, size_t len,
                        uint8_t *reply, size_t reply_max);

// Telemetry and events in binary (called by console.c when the binary
// telemetry mode is on).
void protocol_send_telemetry(const control_snapshot_t *s, uint32_t line, uint32_t axes,
                             uint32_t fields, uint32_t sys);
void protocol_send_event(uint32_t tick, uint8_t kind, uint8_t axis, int64_t pos);
