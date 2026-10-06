"""Client for the stepper controller's binary protocol over USB serial.

    from stepperctl import Client
    with Client() as c:                 # first /dev/cu.usbmodem* (or ttyACM*)
        print(c.ping())
        c.move(1, 500)                  # axis 1 to 500 full steps
        c.group_create(0, [1, 2], feed=800)
        c.group_arc(0, cx=-300, cy=0, degrees=360)

Commands return the controller's command number once acknowledged; they
raise CommandError if it refuses them or no ACK arrives. Queries (ping,
status) are retried; commands are not, since a command whose ACK was lost
may already be queued (a resent relative move would run twice). POSIX only,
standard library only.
"""

import glob
import math
import os
import select
import struct
import termios
import threading
import time

from . import protocol as P


class CommandError(Exception):
    pass


def find_port():
    ports = sorted(glob.glob("/dev/cu.usbmodem*")) or sorted(glob.glob("/dev/ttyACM*"))
    if not ports:
        raise CommandError("no serial port found")
    return ports[0]


class Client:
    def __init__(self, port=None, timeout=0.5, retries=3):
        self.port = port or find_port()
        self.timeout = timeout
        self.retries = retries
        self.fd = os.open(self.port, os.O_RDWR | os.O_NOCTTY)
        attrs = termios.tcgetattr(self.fd)
        attrs[0] = 0                                   # iflag: no translation
        attrs[1] = 0                                   # oflag: raw
        attrs[3] = 0                                   # lflag: raw, no echo
        termios.tcsetattr(self.fd, termios.TCSANOW, attrs)

        self._seq = 0
        self._lock = threading.Lock()
        self._replies = {}                             # seq -> (type, payload)
        self._cond = threading.Condition()
        self._telem_cfg = None                         # (axes list, fields, sys)
        self.on_telemetry = None                       # callable(dict)
        self.on_event = None                           # callable(dict)
        self._running = True
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()

        info = self.ping()
        self.n_axes, self.n_groups = info.axes, info.groups
        self.all_axes = (1 << self.n_axes) - 1

    # --- plumbing ---------------------------------------------------------------

    def close(self):
        self._running = False
        try:
            self.telemetry(0)
        except CommandError:
            pass
        self._reader.join(timeout=1)
        os.close(self.fd)

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def _read_loop(self):
        buf = b""
        while self._running:
            r, _, _ = select.select([self.fd], [], [], 0.1)
            if not r:
                continue
            try:
                chunk = os.read(self.fd, 4096)
            except OSError:
                break
            buf += chunk
            *pieces, buf = buf.split(b"\x00")
            for piece in pieces:
                if not piece:
                    continue
                try:
                    msg_type, seq, payload = P.parse_frame(piece)
                except ValueError:
                    continue  # console text or noise between frames
                self._dispatch(msg_type, seq, payload)

    def _dispatch(self, msg_type, seq, payload):
        if msg_type == P.TELEM:
            if self.on_telemetry and self._telem_cfg:
                try:
                    frame = P.parse_telemetry(payload, *self._telem_cfg)
                except ValueError:
                    return  # sent under an older layout
                self.on_telemetry(frame)
        elif msg_type == P.EVENT:
            if self.on_event:
                self.on_event(P.parse_event(payload))
        else:
            with self._cond:
                self._replies[seq] = (msg_type, payload)
                self._cond.notify_all()

    def request(self, msg_type, payload=b"", retries=1):
        """Send a request and wait for its reply (ACK, PONG or STATUS)."""
        for _ in range(retries):
            with self._lock:
                self._seq = (self._seq + 1) & 0xFFFF
                seq = self._seq
            os.write(self.fd, P.build_frame(msg_type, seq, payload))
            deadline = time.monotonic() + self.timeout
            with self._cond:
                while seq not in self._replies:
                    left = deadline - time.monotonic()
                    if left <= 0:
                        break
                    self._cond.wait(left)
                if seq in self._replies:
                    return self._replies.pop(seq)
        raise CommandError(f"no reply to request 0x{msg_type:02x} on {self.port}")

    def command(self, msg_type, payload=b""):
        rtype, p = self.request(msg_type, payload)
        ack = P.parse_ack(p)
        if not ack.ok:
            raise CommandError(f"request 0x{msg_type:02x}: {P.RESULTS.get(ack.result, ack.result)}")
        return ack.cmd_seq

    def _mask(self, axes):
        return P.axes_mask(axes) & self.all_axes

    # --- queries ----------------------------------------------------------------

    def ping(self):
        rtype, p = self.request(P.PING, retries=self.retries)
        return P.parse_pong(p)

    def status(self):
        rtype, p = self.request(P.STATUS_REQ, retries=self.retries)
        return P.parse_status(p, self.n_axes, self.n_groups)

    def wait_applied(self, cmd_seq, timeout=2.0):
        """Wait until core 1 has applied command `cmd_seq`."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if (self.status().last_seq - cmd_seq) & 0xFFFFFFFF < 0x80000000:
                return
            time.sleep(0.005)
        raise CommandError(f"command {cmd_seq} not applied")

    def wait_settled(self, axes="*", after=0, timeout=60.0):
        """Wait until the axes are at rest. Pass the cmd_seq of the command
        that started the motion as `after`: an ACK only means queued, and
        until core 1 applies it the axes still look settled."""
        mask = self._mask(axes)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            st = self.status()
            applied = (st.last_seq - after) & 0xFFFFFFFF < 0x80000000
            if applied and all(st.axes[i].settled for i in range(self.n_axes) if mask >> i & 1):
                return st
            time.sleep(0.02)
        raise CommandError("axes did not settle")

    # --- axis commands (positions in full steps, speeds in steps/s) -----------------

    def move(self, axes, pos):
        return self.command(P.MOVE, P.req_axes_pos(self._mask(axes), pos))

    def move_rel(self, axes, delta):
        return self.command(P.MOVE_REL, P.req_axes_pos(self._mask(axes), delta))

    def velocity(self, axes, vel):
        return self.command(P.VELOCITY, P.req_velocity(self._mask(axes), vel))

    def jog(self, axes, vel, timeout_ms=300):
        return self.command(P.JOG, P.req_jog(self._mask(axes), vel, timeout_ms))

    def stop(self, axes="*"):
        return self.command(P.STOP, P.req_axes(self._mask(axes)))

    def set_position(self, axes, pos=0.0):
        return self.command(P.SET_POS, P.req_axes_pos(self._mask(axes), pos))

    def limits(self, axes, vmax=0.0, amax=0.0):
        return self.command(P.LIMITS, P.req_limits(self._mask(axes), vmax, amax))

    def profile(self, axes, name, jerk_ms=30):
        return self.command(P.PROFILE, P.req_profile(self._mask(axes), name, jerk_ms))

    def pvt_point(self, axes, pos, vel, ms):
        return self.command(P.PVT_POINT, P.req_pvt_point(self._mask(axes), pos, vel, ms))

    def pvt_start(self, axes):
        return self.command(P.PVT_START, P.req_axes(self._mask(axes)))

    # --- groups (ids 0-3) ---------------------------------------------------------

    def group_create(self, gid, axes, feed=1500.0, corner_ms=0.0):
        return self.command(P.GROUP_CREATE, P.req_group_create(gid, self._mask(axes), feed, corner_ms))

    def group_release(self, gid):
        return self.command(P.GROUP_RELEASE, P.req_group_id(gid))

    def group_line(self, gid, positions, feed=0.0):
        return self.command(P.GROUP_LINE, P.req_group_line(gid, positions, feed))

    def group_arc(self, gid, cx, cy, degrees, feed=0.0, tol=0.0):
        return self.command(P.GROUP_ARC, P.req_group_arc(gid, cx, cy, math.radians(degrees), feed, tol))

    def group_hold(self, gid, hold=True):
        return self.command(P.GROUP_HOLD, P.req_group_hold(gid, hold))

    def group_stop(self, gid):
        return self.command(P.GROUP_STOP, P.req_group_id(gid))

    # --- system -------------------------------------------------------------------

    def amplitude(self, amp=None):
        """amp 0..1 for a fixed drive amplitude, None for automatic."""
        return self.command(P.AMPLITUDE, P.req_amplitude(-1.0 if amp is None else amp))

    def clear_stop(self):
        return self.command(P.CLEAR_STOP)

    def drive(self, axes, amp_low=0.40, amp_high=0.60, amp_hold=0.25, low_speed=300.0,
              high_speed=1600.0, reverse=False, swap_coils=False):
        """Automatic amplitude curve (fractions 0..1) and wiring fixes."""
        return self.command(P.DRIVE, P.req_drive(self._mask(axes), amp_low, amp_high, amp_hold,
                                                 low_speed, high_speed, reverse, swap_coils))

    def boot_show(self, slot):
        """Show slot (0-3) to run at power-up, or None; save_config keeps it."""
        return self.command(P.BOOT_SHOW, struct.pack("<B", 0xFF if slot is None else slot))

    def save_config(self):
        """Write limits, profiles and drive settings to flash (axes at rest)."""
        return self.command(P.CONFIG_SAVE)

    # --- shows (slots 0-3) --------------------------------------------------------

    def show_upload(self, slot, blob, chunk=200):
        """Upload a compiled show (stepperctl.show.compile_show) to a slot.
        Writing flash needs every axis at rest and no show running."""
        self.command(P.SHOW_BEGIN, struct.pack("<BI", slot, len(blob)))
        for off in range(0, len(blob), chunk):
            self.command(P.SHOW_DATA, struct.pack("<I", off) + blob[off:off + chunk])
        old_timeout, self.timeout = self.timeout, 3.0  # erasing 16 KB of flash takes a while
        try:
            return self.command(P.SHOW_END)
        finally:
            self.timeout = old_timeout

    def show_run(self, slot):
        return self.command(P.SHOW_RUN, struct.pack("<B", slot))

    def show_stop(self):
        return self.command(P.SHOW_STOP)

    def show_erase(self, slot):
        old_timeout, self.timeout = self.timeout, 3.0
        try:
            return self.command(P.SHOW_ERASE, struct.pack("<B", slot))
        finally:
            self.timeout = old_timeout

    def show_list(self):
        rtype, p = self.request(P.SHOW_LIST, retries=self.retries)
        return P.parse_show_info(p)

    def telemetry(self, rate_hz, axes="*", fields=("pos", "vel"), sys_fields=(), events=False):
        mask = self._mask(axes)
        f = sum(P.TF[n] for n in fields)
        s = sum(P.TS[n] for n in sys_fields)
        if rate_hz:  # turning it off keeps the layout for frames still in flight
            self._telem_cfg = ([i + 1 for i in range(self.n_axes) if mask >> i & 1], f, s)
        return self.command(P.TELEMETRY, P.req_telemetry(rate_hz, mask, f, s, events))
