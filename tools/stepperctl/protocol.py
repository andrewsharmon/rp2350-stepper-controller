"""Binary protocol for the RP2350 stepper controller.

Mirror of firmware/src/frame.h (framing) and firmware/src/protocol.h (the
message spec; read that file for every field). On the wire a frame is
0x00, COBS(type, seq u16, payload, crc16), 0x00, all little-endian, CRC-16/
CCITT-FALSE over type, seq and payload. Positions are int64 units of 2^-30
full step; this module converts to and from full steps.
"""

import struct
from dataclasses import dataclass, field

UNITS_PER_STEP = 1 << 30
VERSION = 1

# Host -> device
PING, STATUS_REQ = 0x01, 0x02
MOVE, MOVE_REL, VELOCITY, JOG, STOP, SET_POS, LIMITS, PROFILE, PVT_POINT, PVT_START = range(0x10, 0x1A)
GROUP_CREATE, GROUP_RELEASE, GROUP_LINE, GROUP_ARC, GROUP_HOLD, GROUP_STOP = range(0x20, 0x26)
AMPLITUDE, CLEAR_STOP, CONFIG_SAVE, DRIVE, BOOT_SHOW = 0x30, 0x31, 0x32, 0x33, 0x34
TELEMETRY = 0x40
SHOW_BEGIN, SHOW_DATA, SHOW_END, SHOW_RUN, SHOW_STOP, SHOW_LIST, SHOW_ERASE = range(0x50, 0x57)
SHOW_INFO = 0x83
CAM_POINTS, CAM_LOAD, CAM_ENGAGE, VLEADER, CAM_STATUS = range(0x60, 0x65)
CAM_INFO = 0x84
VL_OPS = {"vel": 0, "move": 1, "stop": 2, "limits": 3, "zero": 4}
# Device -> host
ACK, PONG, STATUS = 0x80, 0x81, 0x82
TELEM, EVENT = 0x90, 0x91

RESULTS = {0: "ok", 1: "queue full", 2: "bad request", 3: "unknown"}
EVENTS = {1: "done", 2: "underrun", 3: "estop", 4: "fault", 5: "clear"}
MODES = ["idle", "pos", "vel", "jog", "pvt", "grp", "cam"]
PROFILES = ["trap", "scurve", "smooth", "cosine", "quintic"]

# Telemetry field bits.
TF = {"pos": 1, "vel": 2, "mode": 4, "amp": 8, "q": 16}
TS = {"seq": 1, "vmot": 2, "ladder": 4, "stop": 8}


def to_units(steps):
    return int(round(steps * UNITS_PER_STEP))


def to_steps(units):
    return units / UNITS_PER_STEP


# --- framing --------------------------------------------------------------------

def crc16(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def cobs_encode(data):
    out = bytearray([0])
    code_at, code = 0, 1
    for i, b in enumerate(data):
        if b == 0:
            out[code_at] = code
            code_at, code = len(out), 1
            out.append(0)
            continue
        out.append(b)
        code += 1
        if code == 0xFF:
            out[code_at] = code
            if i + 1 == len(data):
                return bytes(out)  # canonical: no empty block after a full one at the end
            code_at, code = len(out), 1
            out.append(0)
    out[code_at] = code
    return bytes(out)


def cobs_decode(data):
    out = bytearray()
    i = 0
    while i < len(data):
        code = data[i]
        i += 1
        if code == 0 or i + code - 1 > len(data):
            raise ValueError("bad COBS")
        out += data[i:i + code - 1]
        i += code - 1
        if code != 0xFF and i < len(data):
            out.append(0)
    if 0 in data:
        raise ValueError("zero inside COBS data")
    return bytes(out)


def build_frame(msg_type, seq, payload=b""):
    raw = struct.pack("<BH", msg_type, seq) + payload
    raw += struct.pack("<H", crc16(raw))
    return b"\x00" + cobs_encode(raw) + b"\x00"


def parse_frame(data):
    """Decode the bytes between two delimiters -> (type, seq, payload)."""
    raw = cobs_decode(data)
    if len(raw) < 5 or crc16(raw[:-2]) != struct.unpack("<H", raw[-2:])[0]:
        raise ValueError("bad frame")
    msg_type, seq = struct.unpack("<BH", raw[:3])
    return msg_type, seq, raw[3:-2]


# --- requests ---------------------------------------------------------------------

def axes_mask(axes):
    """1-based axis numbers, or '*'/None for all -> bit mask."""
    if axes in (None, "*"):
        return 0xFFFF  # trimmed by the caller to the controller's axis count
    if isinstance(axes, int):
        axes = [axes]
    mask = 0
    for a in axes:
        mask |= 1 << (a - 1)
    return mask


def req_axes_pos(axes, steps):
    return struct.pack("<Hq", axes, to_units(steps))


def req_velocity(axes, vel):
    return struct.pack("<Hf", axes, vel)


def req_jog(axes, vel, timeout_ms):
    return struct.pack("<HfH", axes, vel, timeout_ms)


def req_axes(axes):
    return struct.pack("<H", axes)


def req_limits(axes, vmax, amax):
    return struct.pack("<Hff", axes, vmax, amax)


def req_profile(axes, profile, jerk_ms):
    return struct.pack("<HBH", axes, PROFILES.index(profile), jerk_ms)


def req_pvt_point(axes, steps, vel, ms):
    return struct.pack("<HqfH", axes, to_units(steps), vel, ms)


def req_group_create(gid, axes, feed, corner_ms=0.0):
    return struct.pack("<BHff", gid, axes, feed, corner_ms)


def req_group_id(gid):
    return struct.pack("<B", gid)


def req_group_line(gid, positions, feed=0.0):
    return struct.pack("<BfB", gid, feed, len(positions)) + b"".join(
        struct.pack("<q", to_units(p)) for p in positions)


def req_group_arc(gid, cx, cy, angle_rad, feed=0.0, tol=0.0):
    return struct.pack("<Bffqqd", gid, feed, tol, to_units(cx), to_units(cy), angle_rad)


def req_group_hold(gid, hold):
    return struct.pack("<BB", gid, 1 if hold else 0)


def req_amplitude(amp):
    return struct.pack("<f", amp)


def req_drive(axes, amp_low, amp_high, amp_hold, low_speed, high_speed, reverse=False, swap_coils=False):
    flags = (1 if reverse else 0) | (2 if swap_coils else 0)
    return struct.pack("<HfffffB", axes, amp_low, amp_high, amp_hold, low_speed, high_speed, flags)


def req_telemetry(rate_hz, axes, fields, sys_fields, events):
    return struct.pack("<HHBBB", rate_hz, axes, fields, sys_fields, 1 if events else 0)


# --- replies ----------------------------------------------------------------------

@dataclass
class Ack:
    req_type: int
    result: int
    cmd_seq: int

    @property
    def ok(self):
        return self.result == 0


def parse_ack(p):
    return Ack(*struct.unpack("<BBI", p))


@dataclass
class Pong:
    version: int
    axes: int
    groups: int
    tick_hz: int
    pwm_hz: int
    units_log2: int
    firmware: str


def parse_pong(p):
    v = struct.unpack("<BBBHHB", p[:8])
    return Pong(*v, p[8:].decode(errors="replace"))


@dataclass
class AxisStatus:
    pos: float
    vel: float
    mode: str
    amp_pct: int
    holding: bool
    settled: bool
    pvt_queue: int
    group: int


@dataclass
class GroupStatus:
    active: bool
    running: bool
    held: bool
    arc: bool
    members: list
    queued: int
    path_vel: float
    segments_done: int


@dataclass
class Status:
    tick: int
    outputs_on: bool
    estop: bool
    fault: bool
    last_seq: int
    rejected: int
    pvt_underruns: int
    vmot_mv: int
    ladder_mv: int
    axes: list = field(default_factory=list)
    groups: list = field(default_factory=list)


def parse_status(p, n_axes, n_groups=4):
    head = "<IBIIIHH"
    tick, flags, last_seq, rejected, under, vmot, ladder = struct.unpack_from(head, p, 0)
    st = Status(tick, bool(flags & 1), bool(flags & 2), bool(flags & 4), last_seq, rejected, under, vmot, ladder)
    off = struct.calcsize(head)
    for _ in range(n_axes):
        pos, vel, mode, amp, fl, q, grp = struct.unpack_from("<qfBBBBb", p, off)
        off += 17
        st.axes.append(AxisStatus(to_steps(pos), vel, MODES[mode] if mode < len(MODES) else "?",
                                  amp, bool(fl & 1), bool(fl & 2), q, grp))
    for _ in range(n_groups):
        fl, members, queued, v, done = struct.unpack_from("<BHBfI", p, off)
        off += 12
        st.groups.append(GroupStatus(bool(fl & 1), bool(fl & 2), bool(fl & 4), bool(fl & 8),
                                     [i + 1 for i in range(16) if members >> i & 1], queued, v, done))
    return st


TS_SIZES = {"seq": 4, "vmot": 2, "ladder": 2, "stop": 1}
TF_SIZES = {"pos": 8, "vel": 4, "mode": 1, "amp": 1, "q": 1}


def telemetry_size(n_axes, fields, sys_fields):
    return (8 + sum(sz for n, sz in TS_SIZES.items() if sys_fields & TS[n])
            + n_axes * sum(sz for n, sz in TF_SIZES.items() if fields & TF[n]))


def parse_telemetry(p, axes_list, fields, sys_fields):
    """-> dict: line, tick, optional system fields, and per-axis dicts.
    Raises ValueError if the frame doesn't match the configured layout."""
    if len(p) != telemetry_size(len(axes_list), fields, sys_fields):
        raise ValueError("telemetry frame does not match the configured layout")
    line, tick = struct.unpack_from("<II", p, 0)
    off = 8
    out = {"line": line, "tick": tick}
    for name, fmt in (("seq", "<I"), ("vmot", "<H"), ("ladder", "<H"), ("stop", "<B")):
        if sys_fields & TS[name]:
            out[name] = struct.unpack_from(fmt, p, off)[0]
            off += struct.calcsize(fmt)
    out["axes"] = {}
    for a in axes_list:
        d = {}
        for name, fmt in (("pos", "<q"), ("vel", "<f"), ("mode", "<B"), ("amp", "<B"), ("q", "<B")):
            if fields & TF[name]:
                v = struct.unpack_from(fmt, p, off)[0]
                off += struct.calcsize(fmt)
                d[name] = to_steps(v) if name == "pos" else v
        out["axes"][a] = d
    return out


def parse_show_info(p, slots=4):
    playing = p[0]
    out = {"playing": None if playing == 0xFF else playing, "slots": []}
    off = 1
    for _ in range(slots):
        valid, name, duration, loop, n_tracks = struct.unpack_from("<B16sIBB", p, off)
        off += 23
        out["slots"].append({"valid": bool(valid), "name": name.rstrip(b"\0").decode(errors="replace"),
                             "duration_ms": duration, "loop": bool(loop), "tracks": n_tracks} if valid else None)
    return out


def parse_cam_info(p, n_axes, n_leaders=2):
    loaded = p[0]
    off = 1
    axes = []
    for _ in range(n_axes):
        table, leader = struct.unpack_from("<bB", p, off)
        off += 2
        axes.append(None if table < 0 else {"table": table, "leader": leader})
    leaders = []
    for _ in range(n_leaders):
        pos, vel, vmax, amax, mode = struct.unpack_from("<qfffB", p, off)
        off += 21
        leaders.append({"pos": to_steps(pos), "vel": vel, "vmax": vmax, "amax": amax,
                        "mode": MODES[mode] if mode < len(MODES) else "?"})
    return {"loaded": [k for k in range(4) if loaded >> k & 1], "followers": axes, "leaders": leaders}


def parse_event(p):
    tick, kind, axis, pos = struct.unpack("<IBBq", p)
    return {"tick": tick, "event": EVENTS.get(kind, kind), "axis": axis, "pos": to_steps(pos)}
