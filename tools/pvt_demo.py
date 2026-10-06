#!/usr/bin/env python3
"""Stream a travelling wave to the stepper controller as PVT points.

Every axis follows A * envelope(t) * sin(w t - phase_i): the same wave,
shifted along the axes (and the LED strip). The envelope fades in and out
smoothly, so every axis starts and ends at rest. Points go out every --dt ms,
kept --lead s ahead of the controller; all axes start together with `pgo *`.

POSIX only (macOS / Linux), standard library only.

  tools/pvt_demo.py                       # first /dev/cu.usbmodem* port
  tools/pvt_demo.py --amp 300 --hz 0.5 --seconds 20
"""

import argparse
import glob
import math
import os
import sys
import termios
import time


def open_port(path):
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    attrs = termios.tcgetattr(fd)
    attrs[1] = 0                      # oflag: raw output
    attrs[3] = 0                      # lflag: no echo, non-canonical
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    return fd


LOG = None


def drain(fd):
    out = b""
    while True:
        try:
            chunk = os.read(fd, 65536)
        except BlockingIOError:
            chunk = b""
        if not chunk:
            text = out.decode(errors="replace")
            if LOG and text:
                LOG.write(text)
            return text
        out += chunk


def send(fd, line):
    # The controller echoes input; keep reading while the port is busy so
    # neither direction backs up.
    data = (line + "\r").encode()
    while data:
        try:
            data = data[os.write(fd, data):]
        except BlockingIOError:
            drain(fd)
            time.sleep(0.001)


def smoothstep(x):
    x = min(max(x, 0.0), 1.0)
    return x * x * (3 - 2 * x)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="serial port (default: first /dev/cu.usbmodem*)")
    ap.add_argument("--axes", type=int, default=10)
    ap.add_argument("--amp", type=float, default=200.0, help="amplitude, full steps")
    ap.add_argument("--hz", type=float, default=0.5, help="wave frequency")
    ap.add_argument("--seconds", type=float, default=15.0)
    ap.add_argument("--fade", type=float, default=2.0, help="fade in/out time, s")
    ap.add_argument("--dt", type=int, default=20, help="ms between points")
    ap.add_argument("--lead", type=float, default=0.3, help="s of points kept queued ahead")
    ap.add_argument("--telemetry", type=int, default=0, help="request telemetry at this rate (Hz)")
    ap.add_argument("--log", help="save everything the controller sends to this file")
    args = ap.parse_args()

    global LOG
    if args.log:
        LOG = open(args.log, "w")
    port = args.port or (sorted(glob.glob("/dev/cu.usbmodem*")) or [None])[0]
    if not port:
        sys.exit("no serial port found; pass --port")
    fd = open_port(port)
    time.sleep(0.2)
    drain(fd)

    n, w, dt = args.axes, 2 * math.pi * args.hz, args.dt / 1000.0
    total = args.seconds

    def position(i, t):
        env = smoothstep(t / args.fade) * smoothstep((total - t) / args.fade)
        return args.amp * env * math.sin(w * t - 2 * math.pi * i / n)

    def velocity(i, t, h=1e-4):
        return (position(i, t + h) - position(i, t - h)) / (2 * h)

    # Quiet console (no echo of our commands), then zero every axis:
    # positions are relative to where each axis is now.
    send(fd, "echo off")
    send(fd, "s")
    time.sleep(0.5)
    send(fd, "z *")
    time.sleep(0.1)
    drain(fd)

    def send_point(k):
        t = k * dt
        for i in range(n):
            v = 0.0 if k == steps else velocity(i, t)
            send(fd, f"p {i + 1} {position(i, t):.4f} {v:.3f} {args.dt}")

    steps = int(round(total / dt))
    ahead = max(1, int(args.lead / dt))
    for k in range(1, ahead + 1):           # prefill
        send_point(k)
    if args.telemetry:
        send(fd, f"t {args.telemetry}")
    send(fd, "pgo " + ",".join(str(i + 1) for i in range(n)))
    start = time.monotonic()
    print(f"streaming {steps} points x {n} axes over {total:.1f} s on {port}")

    for k in range(ahead + 1, steps + 1):
        # Keep `ahead` points queued: point k is due at k*dt after start.
        due = start + (k - ahead) * dt
        while time.monotonic() < due:
            time.sleep(0.001)
        send_point(k)
        drain(fd)

    time.sleep(args.lead + 0.5)
    if args.telemetry:
        send(fd, "t 0")
        time.sleep(0.1)
    drain(fd)
    send(fd, "?")
    time.sleep(0.4)
    status = drain(fd)
    lines = [l for l in status.splitlines() if "pvt" in l or "busy" in l or "rejected" in l]
    print("\n".join(lines) if lines else "no underruns or dropped points")
    send(fd, "echo on")


if __name__ == "__main__":
    main()
