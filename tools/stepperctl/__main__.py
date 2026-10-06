"""Command line for the stepper controller.

  python3 tools/stepperctl ping
  python3 tools/stepperctl status
  python3 tools/stepperctl move 1 500             # axis 1 to 500 full steps (axes: 1, 1,2 or *)
  python3 tools/stepperctl move --rel '*' -100
  python3 tools/stepperctl vel 2 300 / jog 2 300 --ms 500 / stop [axes]
  python3 tools/stepperctl zero '*'
  python3 tools/stepperctl limits 1 --vmax 1200 --amax 4000
  python3 tools/stepperctl profile '*' quintic
  python3 tools/stepperctl group 1 create 1,2 --feed 800
  python3 tools/stepperctl group 1 line 400 400
  python3 tools/stepperctl group 1 arc -300 0 360
  python3 tools/stepperctl group 1 hold|resume|stop|release
  python3 tools/stepperctl amp auto|40
  python3 tools/stepperctl clear
  python3 tools/stepperctl drive 1 --low 40 --high 60 --hold 25 [--reverse] [--swap-coils]
  python3 tools/stepperctl save                      # limits, profiles, drive -> flash
  python3 tools/stepperctl telem --hz 100 --seconds 5 --fields pos,vel --axes 1,2 > log.csv

Group ids are 1-4 here (0-3 on the wire).
"""

import argparse
import os
import sys
import time

if __package__ in (None, ""):  # run as `python3 tools/stepperctl`
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    __package__ = "stepperctl"

from stepperctl.client import Client, CommandError  # noqa: E402


def parse_axes(s):
    if s == "*":
        return "*"
    return [int(a) for a in s.split(",")]


def print_status(st):
    state = "running" if st.outputs_on else "STOPPED (" + ("e-stop" if st.estop else "driver fault") + ")"
    print(f"{state}  tick {st.tick}  last cmd {st.last_seq}  rejected {st.rejected}  "
          f"pvt underruns {st.pvt_underruns}  vmot {st.vmot_mv} mV  ladder {st.ladder_mv} mV")
    print("  ax  mode  position        speed    amp  flags  pvtq  group")
    for i, a in enumerate(st.axes):
        flags = ("h" if a.holding else "-") + ("s" if a.settled else "-")
        grp = str(a.group + 1) if a.group >= 0 else "-"
        print(f"  {i + 1:2d}  {a.mode:<4s} {a.pos:12.3f} {a.vel:9.1f}  {a.amp_pct:3d}%  {flags:5s}  {a.pvt_queue:4d}  {grp}")
    for k, g in enumerate(st.groups):
        if g.active:
            state = "held" if g.held else "running" if g.running else "idle"
            print(f"  group {k + 1}: axes {','.join(map(str, g.members))}, {state}, path speed {g.path_vel:.1f}, "
                  f"{g.queued} queued{' + arc' if g.arc else ''}, {g.segments_done} segments done")


def main():
    ap = argparse.ArgumentParser(prog="stepperctl", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("ping")
    sub.add_parser("status")
    p = sub.add_parser("move"); p.add_argument("axes"); p.add_argument("pos", type=float); p.add_argument("--rel", action="store_true")
    p.add_argument("--wait", action="store_true", help="wait until the axes settle")
    p = sub.add_parser("vel"); p.add_argument("axes"); p.add_argument("vel", type=float)
    p = sub.add_parser("jog"); p.add_argument("axes"); p.add_argument("vel", type=float); p.add_argument("--ms", type=int, default=300)
    p = sub.add_parser("stop"); p.add_argument("axes", nargs="?", default="*")
    p = sub.add_parser("zero"); p.add_argument("axes"); p.add_argument("pos", type=float, nargs="?", default=0.0)
    p = sub.add_parser("limits"); p.add_argument("axes"); p.add_argument("--vmax", type=float, default=0); p.add_argument("--amax", type=float, default=0)
    p = sub.add_parser("profile"); p.add_argument("axes"); p.add_argument("name"); p.add_argument("--jerk-ms", type=int, default=30)
    p = sub.add_parser("group"); p.add_argument("id", type=int); p.add_argument("action")
    p.add_argument("args", nargs="*"); p.add_argument("--feed", type=float, default=0.0)
    p.add_argument("--corner-ms", type=float, default=0.0); p.add_argument("--tol", type=float, default=0.0)
    p = sub.add_parser("amp"); p.add_argument("value")
    sub.add_parser("clear")
    sub.add_parser("save")
    p = sub.add_parser("drive"); p.add_argument("axes")
    p.add_argument("--low", type=float, default=40); p.add_argument("--high", type=float, default=60)
    p.add_argument("--hold", type=float, default=25); p.add_argument("--low-speed", type=float, default=300)
    p.add_argument("--high-speed", type=float, default=1600)
    p.add_argument("--reverse", action="store_true"); p.add_argument("--swap-coils", action="store_true")
    p = sub.add_parser("telem"); p.add_argument("--hz", type=int, default=100); p.add_argument("--seconds", type=float, default=5)
    p.add_argument("--axes", default="*"); p.add_argument("--fields", default="pos,vel"); p.add_argument("--sys", default="")
    args = ap.parse_args()

    try:
        with Client(args.port) as c:
            run(c, args)
    except CommandError as e:
        sys.exit(f"error: {e}")


def run(c, a):
    if a.cmd == "ping":
        info = c.ping()
        print(f"{info.firmware}: protocol v{info.version}, {info.axes} axes, {info.groups} groups, "
              f"{info.tick_hz} Hz motion tick, {info.pwm_hz} Hz PWM")
    elif a.cmd == "status":
        print_status(c.status())
    elif a.cmd == "move":
        seq = (c.move_rel if a.rel else c.move)(parse_axes(a.axes), a.pos)
        if a.wait:
            print_status(c.wait_settled(parse_axes(a.axes), after=seq))
    elif a.cmd == "vel":
        c.velocity(parse_axes(a.axes), a.vel)
    elif a.cmd == "jog":
        c.jog(parse_axes(a.axes), a.vel, a.ms)
    elif a.cmd == "stop":
        c.stop(parse_axes(a.axes))
    elif a.cmd == "zero":
        c.set_position(parse_axes(a.axes), a.pos)
    elif a.cmd == "limits":
        c.limits(parse_axes(a.axes), a.vmax, a.amax)
    elif a.cmd == "profile":
        c.profile(parse_axes(a.axes), a.name, a.jerk_ms)
    elif a.cmd == "group":
        gid, act, rest = a.id - 1, a.action, a.args
        if act == "create":
            c.group_create(gid, parse_axes(rest[0]), a.feed or 1500.0, a.corner_ms)
        elif act == "line":
            c.group_line(gid, [float(v) for v in rest], a.feed)
        elif act == "arc":
            cx, cy, deg = (float(v) for v in rest)
            c.group_arc(gid, cx, cy, deg, a.feed, a.tol)
        elif act in ("hold", "resume"):
            c.group_hold(gid, act == "hold")
        elif act == "stop":
            c.group_stop(gid)
        elif act == "release":
            c.group_release(gid)
        else:
            sys.exit(f"unknown group action {act}")
    elif a.cmd == "amp":
        c.amplitude(None if a.value == "auto" else float(a.value) / 100.0)
    elif a.cmd == "clear":
        c.clear_stop()
    elif a.cmd == "save":
        c.save_config()
        print("saved")
    elif a.cmd == "drive":
        c.drive(parse_axes(a.axes), a.low / 100, a.high / 100, a.hold / 100, a.low_speed, a.high_speed,
                a.reverse, a.swap_coils)
    elif a.cmd == "telem":
        fields = [f for f in a.fields.split(",") if f]
        sys_fields = [f for f in a.sys.split(",") if f]
        axes = parse_axes(a.axes)
        names = [n for n in sys_fields]
        alist = list(range(1, c.n_axes + 1)) if axes == "*" else axes
        names += [f"{ax}.{f}" for ax in alist for f in fields]
        print("line,tick," + ",".join(names))
        last = [0]

        def show(t):
            vals = [str(t[n]) for n in sys_fields]
            vals += [f"{t['axes'][ax][f]:.4f}" if f == "pos" else f"{t['axes'][ax][f]}"
                     for ax in alist for f in fields]
            print(f"{t['line']},{t['tick']}," + ",".join(vals))
            last[0] = t["line"]

        c.on_telemetry = show
        c.telemetry(a.hz, axes, fields, sys_fields)
        time.sleep(a.seconds)
        c.telemetry(0)


if __name__ == "__main__":
    main()
