"""JLCPCB fabrication + assembly outputs for a routed board.

    <KiCad python> gen/jlcfab.py single_sh [--check]

Run with KiCad's own Python (it needs pcbnew), like gen/pcbgen.py.

Writes <board>/jlcpcb/:
  <board>_gerbers.zip   Gerbers (Protel names, no X2) + Excellon drill (PTH/NPTH)
  <board>_bom.csv       Comment, Designator, Footprint, LCSC Part #
  <board>_cpl.csv       Designator, Mid X, Mid Y, Layer, Rotation

JLCPCB places each part with the footprint from its own (EasyEDA) library, whose
origin and zero rotation often differ from KiCad's. CPL_FIX holds the
correction per LCSC part. --check fetches those footprints from the JLCEDA
library and reports, for every placed part, how far its pads land from the
KiCad pads with the same number. Rerun it whenever a part changes.
"""

import csv
import json
import math
import os
import subprocess
import sys
import tempfile
import time
import zipfile

import pcbnew

HERE = os.path.dirname(os.path.abspath(__file__))
HW = os.path.dirname(HERE)
KICAD_CLI = "/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli"
LAYERS = "F.Cu,In1.Cu,In2.Cu,B.Cu,F.Paste,B.Paste,F.Silkscreen,B.Silkscreen,F.Mask,B.Mask,Edge.Cuts"

# LCSC -> (extra rotation, dx, dy): JLCPCB footprint vs KiCad footprint, in the
# KiCad footprint's frame at 0 degrees (mm, y up). Found with --check on 2026-10-07.
CPL_FIX = {
    "C15127": (180, 0.071, 0.0),      # AO3401A SOT-23
    "C82942": (270, 0.0, 0.0),        # ME6211 SOT-23-5 (LCSC footprint is drawn vertical)
    "C7484": (180, 0.033, 0.0),       # 74AHCT1G125 SOT-23-5
    "C5349955": (180, 0.0, 0.0),      # WS2812B-2020
    "C165948": (0, 0.0, 1.571),       # USB-C TYPE-C-31-M-12
    "C160394": (180, 0.0, -0.063),    # JST SH 8-pin
    "C160390": (0, 0.0, -0.062),      # JST SH 4-pin
    "C131337": (180, 1.0, 0.0),       # JST PH 2-pin (KiCad origin is pin 1)
    "C131339": (180, 2.0, 0.0),       # JST PH 3-pin
    "C144394": (0, 2.5, 0.0),         # JST XH 3-pin
    # AOTA inductor: the white dot is at JLCPCB's pad 2 (their silkscreen and Abracon's top view), so pad 2
    # lands on +1V1 and the dot sits at the VOUT end like the Pico 2 (RP2350 datasheet fig. 26). Their
    # pin-1 marker is at pad 1; the order note asks them to follow the dot.
    "C42411119": (0, 0.0, 0.0),
}

# LCSC -> {KiCad pad: JLCPCB pad} where the library numbers pins differently.
PAD_MAP = {
    "C22452": {"1": "2", "2": "1"},  # SS54: JLCPCB has 1 = anode, 2 = cathode; KiCad's diodes are 1 = cathode
}


def rot(x, y, a):
    c, s = math.cos(math.radians(a)), math.sin(math.radians(a))
    return x * c - y * s, x * s + y * c


def placed(board):
    """Footprints JLCPCB assembles: an LCSC number and not excluded from the position file."""
    out = []
    for fp in board.GetFootprints():
        f = fp.GetField("LCSC")
        if fp.IsExcludedFromPosFiles() or fp.IsDNP():
            continue
        if not f or not f.GetText():
            sys.exit(f"{fp.GetReference()} is placed but has no LCSC number")
        out.append((fp, f.GetText()))
    return sorted(out, key=lambda p: (p[0].GetReference().rstrip("0123456789"),
                                      int(p[0].GetReference().lstrip("ABCDEFGHIJKLMNOPQRSTUVWXYZ") or 0)))


def cpl_row(fp, lcsc):
    """JLCPCB placement: KiCad origin and rotation plus the CPL_FIX correction (y up)."""
    if fp.IsFlipped():
        sys.exit(f"{fp.GetReference()} is on the bottom; bottom-side corrections aren't handled")
    d, dx, dy = CPL_FIX.get(lcsc, (0, 0.0, 0.0))
    th = fp.GetOrientationDegrees()
    ox, oy = rot(dx, dy, th)
    x = pcbnew.ToMM(fp.GetPosition().x) + ox
    y = -pcbnew.ToMM(fp.GetPosition().y) + oy
    return x, y, (th + d) % 360


def export(board_name):
    bdir = os.path.join(HW, board_name)
    pcb = os.path.join(bdir, board_name + ".kicad_pcb")
    out = os.path.join(bdir, "jlcpcb")
    os.makedirs(out, exist_ok=True)

    with tempfile.TemporaryDirectory() as tmp:
        subprocess.run([KICAD_CLI, "pcb", "export", "gerbers", "--check-zones", "--no-x2", "--no-netlist",
                        "--subtract-soldermask", "-l", LAYERS, "-o", tmp, pcb], check=True, capture_output=True)
        subprocess.run([KICAD_CLI, "pcb", "export", "drill", "--format", "excellon", "--drill-origin", "absolute",
                        "--excellon-units", "mm", "--excellon-zeros-format", "decimal",
                        "--excellon-oval-format", "alternate", "--excellon-separate-th", "-o", tmp + "/", pcb],
                       check=True, capture_output=True)
        with zipfile.ZipFile(os.path.join(out, board_name + "_gerbers.zip"), "w", zipfile.ZIP_DEFLATED) as z:
            for name in sorted(os.listdir(tmp)):
                if not name.endswith(".gbrjob"):
                    z.write(os.path.join(tmp, name), name)

    board = pcbnew.LoadBoard(pcb)
    parts = placed(board)

    # One line per LCSC part: JLCPCB unticks parts that appear on several BOM lines.
    groups = {}
    for fp, lcsc in parts:
        g = groups.setdefault(lcsc, {"values": [], "refs": [], "fp": fp.GetFPID().GetLibItemName().wx_str()})
        if fp.GetValue() not in g["values"]:
            g["values"].append(fp.GetValue())
        g["refs"].append(fp.GetReference())
    with open(os.path.join(out, board_name + "_bom.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["Comment", "Designator", "Footprint", "LCSC Part #"])
        for lcsc, g in groups.items():
            w.writerow([" / ".join(g["values"]), ",".join(g["refs"]), g["fp"], lcsc])

    with open(os.path.join(out, board_name + "_cpl.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["Designator", "Mid X", "Mid Y", "Layer", "Rotation"])
        for fp, lcsc in parts:
            x, y, r = cpl_row(fp, lcsc)
            w.writerow([fp.GetReference(), f"{x:.4f}mm", f"{y:.4f}mm", "Top", f"{r:g}"])

    print(f"{out}: {len(parts)} placements, {len(groups)} BOM lines")
    return board, parts


def jlc_footprint(lcsc, cache):
    """Pads of the JLCEDA library footprint for an LCSC part: [(number, x, y)] in mm, y up, from its origin."""
    path = os.path.join(cache, lcsc + ".json")
    if not os.path.exists(path):
        time.sleep(1)  # the library API rate-limits
        subprocess.run(["curl", "-sf", "-A", "Mozilla/5.0", "-o", path,
                        f"https://lceda.cn/api/products/{lcsc}/components?version=6.4.19.5"], check=True)
    res = json.load(open(path)).get("result")
    if not res:
        return None
    data = res["packageDetail"]["dataStr"]
    ox, oy = float(data["head"]["x"]), float(data["head"]["y"])
    pads = []
    for shape in data["shape"]:
        f = shape.split("~")
        if f[0] == "PAD":  # PAD~shape~x~y~w~h~layer~net~number~...; units of 10 mil, y down
            pads.append((f[8], (float(f[2]) - ox) * 0.254, -(float(f[3]) - oy) * 0.254))
    return pads


def check(parts, cache):
    """Where JLCPCB's own footprint lands when placed from our CPL, compared with the KiCad pads.

    offset: distance between the two pad-set centres. Each KiCad pad must also be
    nearest to the JLCPCB pad of the same pin, or the part is rotated wrong.
    Land patterns differ, so single pads can be a few tenths off (SMA: 0.5 mm).
    """
    bad = 0
    for fp, lcsc in parts:
        pads = jlc_footprint(lcsc, cache)
        if pads is None:
            print(f"{fp.GetReference():5s} {lcsc:10s} no library footprint (JLCPCB places it from the CPL as is)")
            continue
        x, y, r = cpl_row(fp, lcsc)
        theirs = {}
        for n, px, py in pads:
            theirs.setdefault(n, []).append(tuple(a + b for a, b in zip(rot(px, py, r), (x, y))))
        pmap = PAD_MAP.get(lcsc, {})
        pairs = []
        for p in fp.Pads():
            n = pmap.get(p.GetNumber(), p.GetNumber())
            if n in theirs and len(theirs[n]) == 1:
                pairs.append((n, (pcbnew.ToMM(p.GetPosition().x), -pcbnew.ToMM(p.GetPosition().y)), theirs[n][0]))
        if len(pairs) < 2:
            print(f"{fp.GetReference():5s} {lcsc:10s} fewer than 2 pads match by number  <-- CHECK")
            bad += 1
            continue
        cx = [sum(p[i][0] for p in pairs) / len(pairs) for i in (1, 2)]
        cy = [sum(p[i][1] for p in pairs) / len(pairs) for i in (1, 2)]
        offset = math.hypot(cx[0] - cx[1], cy[0] - cy[1])
        worst = max(math.hypot(k[0] - j[0], k[1] - j[1]) for _, k, j in pairs)
        wrong = [n for n, k, _ in pairs
                 if min(pairs, key=lambda q: math.hypot(k[0] - q[2][0], k[1] - q[2][1]))[0] != n]
        flag = "  <-- CHECK" if offset > 0.1 or wrong else ""
        bad += bool(flag)
        print(f"{fp.GetReference():5s} {lcsc:10s} rot {r:5g}  {len(pairs):2d} pads, offset {offset:.3f} mm, "
              f"worst pad {worst:.3f} mm{', pins ' + ' '.join(wrong) + ' misplaced' if wrong else ''}{flag}")
    print(f"{bad} part(s) to check")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    board, parts = export(sys.argv[1])
    if "--check" in sys.argv:
        cache = os.path.join(tempfile.gettempdir(), "jlceda-footprints")
        os.makedirs(cache, exist_ok=True)
        check(parts, cache)
