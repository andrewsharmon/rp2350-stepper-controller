"""Compile shows from JSON into the controller's binary show format.

    {
      "name": "wave",                 # up to 16 characters
      "duration": 8000,               # ms; a looping show repeats with this period
      "loop": true,
      "axes": {                       # axis number -> keyframes [t_ms, pos] or [t_ms, pos, vel]
        "1": [[0, 0], [2000, 200], [6000, -200]]
      },
      "leds": {                       # pixel number -> keyframes [t_ms, "#rrggbb" or [r, g, b]]
        "11": [[0, "#ff0000"], [4000, "#0000ff"]]
      }
    }

Positions are full steps relative to each axis's zero; a show first moves
its axes to their first keyframe. Without a speed, a key's speed comes from
its neighbours (zero at the ends of a non-looping show). Layout: see
firmware/src/show.h.
"""

import json
import math
import struct
import zlib

MAGIC, VERSION, HEADER_SIZE, MAX_SIZE, NAME_LEN = 0x574F4853, 1, 40, 16384, 16
TRACK_AXIS, TRACK_LED, FLAG_LOOP = 1, 2, 1


class ShowError(ValueError):
    pass


def _color(c):
    if isinstance(c, str):
        c = c.lstrip("#")
        if len(c) != 6:
            raise ShowError(f"bad colour {c!r}")
        return tuple(int(c[i:i + 2], 16) for i in (0, 2, 4))
    if len(c) != 3 or not all(0 <= v <= 255 for v in c):
        raise ShowError(f"bad colour {c!r}")
    return tuple(int(v) for v in c)


def _times(keys, what):
    ts = [int(k[0]) for k in keys]
    if not ts:
        raise ShowError(f"{what}: no keyframes")
    if any(b <= a for a, b in zip(ts, ts[1:])) or ts[0] < 0:
        raise ShowError(f"{what}: key times must increase from >= 0")
    return ts


def crc32(blob):
    """CRC-32 of the blob with its crc field (bytes 12-15) taken as zero."""
    return zlib.crc32(blob[:12] + b"\0\0\0\0" + blob[16:]) & 0xFFFFFFFF


def compile_show(spec, n_axes=10, n_pixels=20):
    name = spec.get("name", "show")
    if len(name.encode()) > NAME_LEN:
        raise ShowError("name longer than 16 bytes")
    loop = bool(spec.get("loop", False))
    duration = int(spec.get("duration", 0))
    tracks, last_t = [], 0

    for ax, keys in sorted(spec.get("axes", {}).items(), key=lambda kv: int(kv[0])):
        ch = int(ax) - 1
        if not 0 <= ch < n_axes:
            raise ShowError(f"axis {ax} out of range")
        ts = _times(keys, f"axis {ax}")
        body = b""
        for k in keys:
            vel = float(k[2]) if len(k) > 2 else math.nan
            body += struct.pack("<Iff", int(k[0]), float(k[1]), vel)
        tracks.append(struct.pack("<BBH", TRACK_AXIS, ch, len(keys)) + body)
        last_t = max(last_t, ts[-1])

    for px, keys in sorted(spec.get("leds", {}).items(), key=lambda kv: int(kv[0])):
        p = int(px)
        if not 0 <= p < n_pixels:
            raise ShowError(f"pixel {px} out of range")
        ts = _times(keys, f"pixel {px}")
        body = b"".join(struct.pack("<IBBBB", int(k[0]), *_color(k[1]), 0) for k in keys)
        tracks.append(struct.pack("<BBH", TRACK_LED, p, len(keys)) + body)
        last_t = max(last_t, ts[-1])

    if len(tracks) > 32:
        raise ShowError("more than 32 tracks")
    if loop and duration <= last_t:
        raise ShowError(f"a looping show needs a duration after its last key ({last_t} ms)")

    header = struct.pack("<IHHII16sIBBH", MAGIC, VERSION, HEADER_SIZE, 0, 0,
                         name.encode().ljust(NAME_LEN, b"\0"), duration,
                         FLAG_LOOP if loop else 0, len(tracks), 0)
    blob = bytearray(header + b"".join(tracks))
    if len(blob) > MAX_SIZE:
        raise ShowError(f"show is {len(blob)} bytes, the limit is {MAX_SIZE}")
    struct.pack_into("<I", blob, 8, len(blob))
    struct.pack_into("<I", blob, 12, crc32(bytes(blob)))
    return bytes(blob)


def load(path, **kw):
    with open(path) as f:
        return compile_show(json.load(f), **kw)
