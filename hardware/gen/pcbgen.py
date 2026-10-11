"""Generate a starting PCB for a board from its schematic.

    <KiCad python> gen/pcbgen.py controller

Run with KiCad's own Python (it needs the pcbnew module), e.g. on macOS:
/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3

Creates <board>/<board>.kicad_pcb: 4-layer stackup, JLCPCB design rules, outline,
every footprint with its nets, the mechanically fixed parts placed, and the
rest parked beside the board. Footprints are linked to their schematic
symbols, so "Update PCB from Schematic" works afterwards.

This is a one-shot starting point: it refuses to overwrite an existing
board, since the layout is hand-edited from then on.
"""

import json
import os
import subprocess
import sys
import tempfile

import pcbnew

from sexp import parse

HERE = os.path.dirname(os.path.abspath(__file__))
HW = os.path.dirname(HERE)
KICAD_CLI = "/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli"
FP_ROOT = "/Applications/KiCad/KiCad.app/Contents/SharedSupport/footprints"
LOCAL_LIBS = {"stack": os.path.join(HW, "lib", "stack.pretty")}

MM = pcbnew.FromMM
CX, CY = 100.0, 100.0  # board centre on the sheet


def field(node, key):
    for item in node[1:]:
        if isinstance(item, list) and item and item[0] == key:
            return item[1] if len(item) > 1 else None
    return None


def children(node, key):
    return [i for i in node[1:] if isinstance(i, list) and i and i[0] == key]


def read_netlist(sch):
    with tempfile.TemporaryDirectory() as d:
        out = os.path.join(d, "board.net")
        subprocess.run([KICAD_CLI, "sch", "export", "netlist", "--format", "kicadsexpr", "-o", out, sch],
                       check=True, capture_output=True)
        root = parse(open(out).read())
    comps = []
    for c in children(children(root, "components")[0], "comp"):
        extra = {}
        for f in children(children(c, "fields")[0], "field") if children(c, "fields") else []:
            name = field(f, "name")
            if name in ("LCSC", "MPN", "Note") and len(f) > 2:
                extra[name] = f[2]
        in_bom = not any(field(pr, "name") == "exclude_from_bom" for pr in children(c, "property"))
        comps.append({"ref": field(c, "ref"), "value": field(c, "value"), "in_bom": in_bom,
                      "footprint": field(c, "footprint"), "uuid": field(c, "tstamps"), "fields": extra})
    pads = {}  # (ref, pin) -> net name
    nets = []
    for n in children(children(root, "nets")[0], "net"):
        name = field(n, "name")
        nets.append(name)
        for node in children(n, "node"):
            pads[(field(node, "ref"), field(node, "pin"))] = name
    return comps, nets, pads


def load_footprint(fpid):
    lib, name = fpid.split(":", 1)
    path = LOCAL_LIBS.get(lib, os.path.join(FP_ROOT, lib + ".pretty"))
    fp = pcbnew.FootprintLoad(path, name)
    if fp is None:
        raise SystemExit(f"footprint {fpid} not found in {path}")
    fp.SetFPID(pcbnew.LIB_ID(lib, name))
    return fp


def outline(board, w, h, r):
    """Rounded-rectangle Edge.Cuts outline centred on (CX, CY)."""
    x0, y0, x1, y1 = CX - w / 2, CY - h / 2, CX + w / 2, CY + h / 2

    def seg(a, b):
        s = pcbnew.PCB_SHAPE(board, pcbnew.SHAPE_T_SEGMENT)
        s.SetStart(pcbnew.VECTOR2I(MM(a[0]), MM(a[1])))
        s.SetEnd(pcbnew.VECTOR2I(MM(b[0]), MM(b[1])))
        s.SetLayer(pcbnew.Edge_Cuts)
        s.SetWidth(MM(0.05))
        board.Add(s)

    def arc(c, start, mid, end):
        s = pcbnew.PCB_SHAPE(board, pcbnew.SHAPE_T_ARC)
        s.SetArcGeometry(*(pcbnew.VECTOR2I(MM(p[0]), MM(p[1])) for p in (start, mid, end)))
        s.SetLayer(pcbnew.Edge_Cuts)
        s.SetWidth(MM(0.05))
        board.Add(s)

    k = r * (1 - 0.5 ** 0.5)
    seg((x0 + r, y0), (x1 - r, y0))
    seg((x1, y0 + r), (x1, y1 - r))
    seg((x1 - r, y1), (x0 + r, y1))
    seg((x0, y1 - r), (x0, y0 + r))
    arc(None, (x1 - r, y0), (x1 - k, y0 + k), (x1, y0 + r))
    arc(None, (x1, y1 - r), (x1 - k, y1 - k), (x1 - r, y1))
    arc(None, (x0 + r, y1), (x0 + k, y1 - k), (x0, y1 - r))
    arc(None, (x0, y0 + r), (x0 + k, y0 + k), (x0 + r, y0))


def zone(board, net, layer, w, h, inset=0.3, pts=None, priority=0, clearance=None):
    """Copper pour over the whole board (w x h), or over the polygon pts (sheet mm)."""
    z = pcbnew.ZONE(board)
    z.SetLayer(layer)
    z.SetNet(net)
    z.SetAssignedPriority(priority)
    if not pts:
        x0, y0 = CX - w / 2 + inset, CY - h / 2 + inset
        x1, y1 = CX + w / 2 - inset, CY + h / 2 - inset
        pts = ((x0, y0), (x1, y0), (x1, y1), (x0, y1))
    poly = z.Outline()
    poly.NewOutline()
    for x, y in pts:
        poly.Append(MM(x), MM(y))
    z.SetMinThickness(MM(0.15))
    if clearance is not None:
        z.SetLocalClearance(MM(clearance))
    z.SetPadConnection(pcbnew.ZONE_CONNECTION_THERMAL)
    board.Add(z)


def place(fp, x, y, rot=0, bottom=False):
    if bottom:
        fp.Flip(fp.GetPosition(), pcbnew.FLIP_DIRECTION_TOP_BOTTOM)
    fp.SetPosition(pcbnew.VECTOR2I(MM(x), MM(y)))
    fp.SetOrientationDegrees(rot)


# JLCPCB's cheapest via class (no extra cost): 0.3 mm drill, used with a 0.52 mm
# pad so the annular ring is 0.11 mm (JLCDFM flags 0.1 and under as danger).
BIG_VIAS = {"single_sh_proto"}


def jlc_rules(board, thickness=1.0, big_vias=False):
    """JLCPCB 4-layer capabilities with some margin."""
    ds = board.GetDesignSettings()
    ds.SetBoardThickness(MM(thickness))
    ds.m_TrackMinWidth = MM(0.1)
    ds.m_MinClearance = MM(0.1)
    ds.m_ViasMinSize = MM(0.3)
    ds.m_ViasMinAnnularWidth = MM(0.075)
    ds.m_MinThroughDrill = MM(0.15)
    ds.m_HoleClearance = MM(0.2)
    ds.m_HoleToHoleMin = MM(0.25)
    ds.m_CopperEdgeClearance = MM(0.3)
    ds.m_SilkClearance = MM(0.05)
    ds.m_MinSilkTextHeight = MM(0.6)
    if big_vias:
        ds.m_ViasMinSize = MM(0.45)
        ds.m_MinThroughDrill = MM(0.3)


def jlc_netclasses(pro_path, big_vias=False):
    """Net classes live in the project file."""
    vd, vh = (0.52, 0.3) if big_vias else (0.3, 0.15)    # 0.52 / 0.3: 0.11 mm annular ring (JLCDFM: <= 0.1 is danger)
    pro = json.load(open(pro_path))
    ns = pro.setdefault("net_settings", {})
    ns["classes"] = [
        {"name": "Default", "clearance": 0.1, "track_width": 0.15, "via_diameter": vd, "via_drill": vh,
         "diff_pair_width": 0.2, "diff_pair_gap": 0.15, "microvia_diameter": 0.3, "microvia_drill": 0.1,
         "wire_width": 6, "bus_width": 12, "line_style": 0, "pcb_color": "rgba(0, 0, 0, 0.000)",
         "schematic_color": "rgba(0, 0, 0, 0.000)", "priority": 2147483647},
        {"name": "Power", "clearance": 0.1, "track_width": 0.25, "via_diameter": vd, "via_drill": vh,
         "diff_pair_width": 0.2, "diff_pair_gap": 0.15, "microvia_diameter": 0.3, "microvia_drill": 0.1,
         "wire_width": 6, "bus_width": 12, "line_style": 0, "pcb_color": "rgba(0, 0, 0, 0.000)",
         "schematic_color": "rgba(0, 0, 0, 0.000)", "priority": 0},
    ]
    ns["netclass_patterns"] = [{"netclass": "Power", "pattern": p}
                               for p in ("/V5", "/V5_IN", "/VBUS", "/VBUS_F", "/GND", "/+3V3", "/+1V1", "/VREG_LX")]
    json.dump(pro, open(pro_path, "w"), indent=2)


# ---------------------------------------------------------------------------
# Tier 1: controller. 30 x 30 mm, 4 layers: F signal, In1 GND, In2 3V3/power,
# B signal + BTB headers. The 2.0 mm stack leaves room for low parts (<= ~1 mm)
# on the underside away from the headers.
#
# Top: USB-C on the top edge, MCU centred below it, core regulator in the gap
# between them (its pins are at the MCU's top-right), power path top-left,
# BTN1/BTN2 directly over the BTB headers (a press is carried straight down
# through the mated connector), BOOTSEL below BTN2, Qwiic on the bottom edge,
# status LED and its buffer bottom-right.
# Bottom: BTB A left / B right (offset +/-2 mm so the stack mates one way),
# MCU decoupling under the chip next to each supply pin, crystal below the
# chip, USB ESD and series/CC resistors under the receptacle, LDO and
# soft-start parts under the power path, test pads and jumpers.
# ---------------------------------------------------------------------------

def controller(board, fps):
    W = H = 30.0
    outline(board, W, H, 1.0)
    top, left, right, bottom = CY - H / 2, CX - W / 2, CX + W / 2, CY + H / 2

    # USB-C: receptacle mouth flush with the top edge (front of the courtyard
    # 0.5 mm past it; the enclosure supports the shell).
    usb = fps["J1"]
    # 2 mm left of centre so the core regulator's inductor fits by its pins.
    place(usb, CX - 2.0, top, rot=180)
    bb = usb.GetCourtyard(pcbnew.F_CrtYd).BBox()
    usb.Move(pcbnew.VECTOR2I(0, MM(top - 0.5) - bb.GetTop()))

    ux, uy = CX, CY + 2.0
    place(fps["U1"], ux, uy)
    place(fps["J10"], left + 2.45, CY + 2.0, rot=270, bottom=True)  # pin 1 at the top
    place(fps["J11"], right - 2.45, CY - 2.0, rot=270, bottom=True)
    place(fps["SW1"], left + 3.1, CY + 2.0, rot=90)    # BTN1 over connector A
    place(fps["SW2"], right - 3.1, CY - 2.0, rot=90)   # BTN2 over connector B
    place(fps["SW3"], right - 3.1, CY + 8.3, rot=90)   # BOOTSEL

    top_parts = {
        # Core regulator: pins 61-65 (AVDD, PGND, LX, VIN, 1V1) at the MCU's top-right.
        "C14": (ux + 2.0, uy - 6.85, 90),   # 1V1 out, by pin 65
        "C10": (ux + 3.1, uy - 6.85, 90),   # VREG_VIN, by pin 64
        "L1": (ux + 5.4, uy - 7.0, 0),      # by pin 63 (LX)
        "C15": (ux + 5.4, uy - 9.4, 0),     # VREG_AVDD filter
        "R1": (ux + 5.4, uy - 10.6, 0),
        # Power path: VBUS -> F1 -> D1 -> V5_IN -> Q1 -> V5.
        "F1": (left + 3.5, top + 3.0, 0),
        "D1": (left + 4.0, top + 7.2, 0),
        "Q1": (left + 3.0, top + 11.0, 0),
        "C18": (left + 7.6, top + 11.4, 90),
        # Qwiic on the bottom edge, LED and its buffer bottom-right.
        "J2": (left + 6.5, bottom - 3.5, 0),
        "J3": (CX, bottom - 3.5, 0),
        "U4": (ux + 6.5, bottom - 5.3, 0),
        "D2": (ux + 6.5, bottom - 2.0, 0),
    }
    for ref, (x, y, r) in top_parts.items():
        place(fps[ref], x, y, rot=r)

    # MCU decoupling on the underside, under the package ring next to each
    # supply pin: (ref, pin, offset along the edge to clear its neighbour).
    u1 = fps["U1"]
    pad_at = {p.GetNumber(): p.GetPosition() for p in u1.Pads()}
    # Each cap starts behind its pin and slides along the edge (nearest first)
    # until its courtyard clears the caps already placed.
    decoupling = [("C1", "5"), ("C11", "10"), ("C2", "15"), ("C3", "24"), ("C4", "29"), ("C12", "32"),
                  ("C5", "41"), ("C6", "50"), ("C13", "51"), ("C7", "59"), ("C8", "68"), ("C9", "76")]
    taken = []

    def crtyd(fp):
        bb = fp.GetCourtyard(pcbnew.B_CrtYd).BBox()
        return [pcbnew.ToMM(v) for v in (bb.GetLeft(), bb.GetTop(), bb.GetRight(), bb.GetBottom())]

    def clear(a):
        return all(a[2] <= b[0] or b[2] <= a[0] or a[3] <= b[1] or b[3] <= a[1] for b in taken)

    for ref, pin in decoupling:
        q = pad_at[pin]
        px, py = pcbnew.ToMM(q.x) - ux, pcbnew.ToMM(q.y) - uy
        side = abs(px) > abs(py)
        fp = fps[ref]
        for along in sorted((k * 0.05 for k in range(-60, 61)), key=abs):
            if side:    # left/right edge: cap vertical, 1.6 mm inside the pad
                x, y, r = ux + px - 1.6 * (1 if px > 0 else -1), uy + py + along, 90
            else:       # top/bottom edge: cap horizontal
                x, y, r = ux + px + along, uy + py - 1.6 * (1 if py > 0 else -1), 0
            place(fp, x, y, rot=r, bottom=not fp.IsFlipped())
            if clear(crtyd(fp)):
                break
        else:
            raise SystemExit(f"no room for {ref} by U1 pin {pin}")
        taken.append(crtyd(fp))

    bottom_parts = {
        # Crystal below the chip, by XIN/XOUT (pins 30/31).
        "Y1": (ux, uy + 7.6, 0),
        "C16": (ux - 3.0, uy + 7.6, 270),  # XIN pad towards Y1 pin 1
        "C17": (ux + 3.0, uy + 7.6, 90),
        "R2": (ux + 1.4, uy + 5.3, 0),
        # USB: ESD and series resistors under the receptacle, CC pull-downs.
        "U2": (CX - 2.0, top + 7.6, 0),
        "R3": (CX - 0.5, top + 10.2, 0),
        "R4": (CX - 3.5, top + 10.2, 0),
        "R5": (CX - 5.4, top + 4.0, 90),
        "R6": (CX + 1.4, top + 4.0, 90),
        # LDO and soft-start under the power path.
        "U3": (left + 4.0, top + 3.0, 0),
        "C19": (left + 6.8, top + 3.0, 90),
        "C21": (left + 1.3, top + 3.0, 90),
        "C20": (left + 3.0, top + 9.5, 0),
        "R7": (left + 6.0, top + 9.5, 0),
        # Ladder, board ID, BOOTSEL resistor, LED/buffer decoupling.
        "R10": (ux + 7.6, uy + 2.6, 90),
        "C22": (ux + 7.6, uy + 4.8, 90),
        "R14": (ux + 7.6, uy + 7.0, 90),
        "R13": (ux + 7.6, uy + 9.2, 90),
        "R11": (left + 6.4, CY + 2.0, 90),
        "R12": (right - 6.4, CY - 2.0, 90),
        "C23": (ux + 9.0, bottom - 5.3, 90),
        "C24": (ux + 9.0, bottom - 2.2, 90),
        # Qwiic jumpers and pull-ups.
        "JP1": (left + 3.0, bottom - 4.6, 0),
        "JP2": (left + 3.0, bottom - 1.8, 0),
        "R8": (left + 6.2, bottom - 4.6, 90),
        "R9": (left + 7.4, bottom - 4.6, 90),
        # Test pads along the bottom edge.
        "TP1": (CX - 6.0, bottom - 1.6, 0),
        "TP2": (CX - 3.5, bottom - 1.6, 0),
        "TP3": (CX - 1.0, bottom - 1.6, 0),
        "TP4": (CX + 1.5, bottom - 1.6, 0),
        "TP5": (CX + 4.0, bottom - 1.6, 0),
    }
    for ref, (x, y, r) in bottom_parts.items():
        place(fps[ref], x, y, rot=r, bottom=True)

    return {"U1", "J1", "J10", "J11", "SW1", "SW2", "SW3"} | set(top_parts) | set(bottom_parts) | \
        {ref for ref, _ in decoupling}


# ---------------------------------------------------------------------------
# Tier 2: universal driver. 30 x 30 mm, 4 layers: F/B signals, In1 GND, In2
# V5 (a 2-layer try left 20 nets unrouted and chopped the pours apart). Sockets (top) receive the controller's headers; headers
# (underside) feed tier 3, both at the controller's BTB centres with the same
# rotation so pin n stacks on pin n. Output headers move to the top and
# bottom edges (see driver()). AT8833s in a 2 x 3 grid per side: sense
# resistors at each chip's left corners, VINT/VM/VCP caps on its right.
# ---------------------------------------------------------------------------

BTB_A = (CX - 11.25, CY + 2.0)   # matches the controller's J10
BTB_B = (CX + 11.25, CY)         # and J11


def driver(board, fps):
    outline(board, 30.0, 30.0, 1.0)
    # Inputs: sockets on top at the controller's BTB positions. Outputs:
    # headers on the underside along the top and bottom edges, so the
    # input and output fan-ins don't converge on the same pin field. The
    # +/-1.5 mm x offsets key the tier-3 stack.
    place(fps["J1"], *BTB_A, rot=270)
    place(fps["J2"], *BTB_B, rot=270)
    place(fps["J3"], CX + 1.5, CY - 12.3, rot=0, bottom=True)    # motors 1-7 out
    place(fps["J4"], CX - 1.5, CY + 12.3, rot=0, bottom=True)    # motors 8-10 out + utilities

    cols = (CX - 4.2, CX + 4.4)
    rows = (CY - 6.5, CY, CY + 6.5)
    # Top: motors 1-6 (2 x 3). Underside: 7 on the left; 8-10 on the right,
    # next to connector B and the bottom-edge output header.
    slots = [(c, r, False) for c in cols for r in rows] + [(cols[0], rows[0], True)] + \
            [(cols[1], r, True) for r in rows]
    placed = {"J1", "J2", "J3", "J4"}
    for k, (cx, cy, bot) in zip(range(1, 11), slots):
        place(fps[f"U{k}"], cx, cy, bottom=bot)
        r_a, r_b = f"R{2 * k - 1}", f"R{2 * k}"
        c_vint, c_vcp, c_vm = f"C{3 * k - 2}", f"C{3 * k - 1}", f"C{3 * k}"
        sy = -1 if bot else 1                           # the flip to the underside mirrors y
        place(fps[r_a], cx - 3.1, cy - sy * 1.6, rot=90, bottom=bot)    # AISEN, by pin 1
        place(fps[r_b], cx - 3.1, cy + sy * 1.6, rot=90, bottom=bot)    # BISEN, by pin 4
        place(fps[c_vint], cx + 3.6, cy - sy * 1.15, bottom=bot)        # pin 12
        place(fps[c_vm], cx + 3.6, cy, bottom=bot)                      # pin 10
        place(fps[c_vcp], cx + 3.6, cy + sy * 1.15, bottom=bot)         # pin 9 (to VM)
        placed |= {f"U{k}", r_a, r_b, c_vint, c_vcp, c_vm}
    # Bulk caps and the small parts in the free underside slots (left column).
    for ref, (x, y, r) in {"C31": (cols[0] - 1.5, rows[1], 90), "C32": (cols[0] + 1.5, rows[1], 90),
                           "C33": (cols[0] - 1.5, rows[2], 90), "R21": (cols[0] + 1.5, rows[2] - 1.2, 90),
                           "R22": (cols[0] + 3.0, rows[2] - 1.2, 90), "R23": (cols[0] + 3.0, rows[2] + 1.2, 90),
                           "C34": (cols[0] + 1.5, rows[2] + 1.2, 90)}.items():
        place(fps[ref], x, y, rot=r, bottom=True)
    placed |= {"C31", "C32", "C33", "R21", "R22", "R23", "C34"}
    return placed


# ---------------------------------------------------------------------------
# Single board, dual 8 mm micro steppers (single_sh). All parts on top; 4
# layers: F signals, In1 GND, In2 V5 (3V3 island under the MCU, short motor
# output jumpers), B signals. 20 mm wide, laid out along x:
#
#   USB-C + power | drivers M1-M4 | MCU | drivers M5-M10 | LED strip
#
# Drivers sit either side of the MCU so each motor-input bus is at most 24
# lines wide. The JST-SH motor connectors run along the +y edge under their
# drivers; the misc connectors (Qwiic, BOOTSEL, e-stop, panel, 5 V in, LED
# power) run along the -y edge above them.
#
# Driver cell (AT8833 rotated 90: outputs face the motor connector, inputs on
# the -y half): VINT / VM / VCP caps in a row above, 0603 sense resistors below
# (neighbouring cells' GND pads sit 0.2 mm apart; their courtyards touch).
# Connector pins run B2 B1 A2 A1 (left to right) against the driver's A1 A2 B2
# B1, so B1/B2 stay on F.Cu and A1/A2 hop over them on In2. A pair's two
# drivers sit 6.5 mm apart over 4 mm-apart pin groups: the even motor's driver
# is 1.825 mm left of its pins, the odd one 0.675 mm right.
# ---------------------------------------------------------------------------

SH_L, SH_W = 122.0, 20.0
SH_VD = 9.7                                 # driver row centre (v)
# Connector order follows the MCU's pin order, so each motor bus runs without
# crossing itself: M1 nearest the MCU going -x, M10 nearest it going +x.
SH_PAIRS = [("J12", 3, 4, 29.3), ("J11", 1, 2, 42.3),                       # (conn, odd, even, u)
            ("J15", 9, 10, 78.75), ("J14", 7, 8, 91.75), ("J13", 5, 6, 104.75)]
SH_UM = 59.0                                # MCU centre (u); v = 10


def sh_xy(u, v):
    """Board-local (u from the -x edge, v from the -y edge) to sheet coordinates."""
    return CX - SH_L / 2 + u, CY - SH_W / 2 + v


def single_sh(board, fps):
    outline(board, SH_L, SH_W, 1.0)
    placed = set()
    taken = []      # courtyard boxes of placed parts, sheet mm (x0, y0, x1, y1)

    def crtyd(fp):
        bb = fp.GetCourtyard(pcbnew.F_CrtYd).BBox()
        return [pcbnew.ToMM(v) for v in (bb.GetLeft(), bb.GetTop(), bb.GetRight(), bb.GetBottom())]

    def clear(a, margin=0.0):
        x0, y0 = sh_xy(margin, margin)
        x1, y1 = sh_xy(SH_L - margin, SH_W - margin)
        if a[0] < x0 or a[1] < y0 or a[2] > x1 or a[3] > y1:
            return False
        return all(a[2] <= b[0] or b[2] <= a[0] or a[3] <= b[1] or b[3] <= a[1] for b in taken)

    def put(ref, u, v, rot=0):
        place(fps[ref], *sh_xy(u, v), rot=rot)
        placed.add(ref)
        taken.append(crtyd(fps[ref]))

    def put_all(parts):
        for ref, (u, v, r) in parts.items():
            put(ref, u, v, r)

    def put_near(ref, u, v, rot=0, reach=6.0):
        """Place at (u, v), or the nearest free spot (0.1 mm grid spiral) within reach."""
        fp = fps[ref]
        steps = int(reach / 0.1)
        cands = sorted(((du, dv) for du in range(-steps, steps + 1) for dv in range(-steps, steps + 1)),
                       key=lambda d: d[0] ** 2 + d[1] ** 2)
        for du, dv in cands:
            place(fp, *sh_xy(u + du * 0.1, v + dv * 0.1), rot=rot)
            if clear(crtyd(fp), 0.1):
                if du * du + dv * dv > 100:
                    print(f"  {ref}: moved {du * 0.1:.1f}, {dv * 0.1:.1f}")
                placed.add(ref)
                taken.append(crtyd(fp))
                return
        print(f"  {ref}: no room near ({u}, {v}), left off the board")

    def row(parts, u, v_edge=0.05, gap=0.1):
        """Pack parts left to right along the -y edge from u, courtyards touching v_edge."""
        for ref, rot in parts:
            fp = fps[ref]
            place(fp, 0, 0, rot=rot)
            bb = crtyd(fp)
            x0, y0 = sh_xy(u, v_edge)
            place(fp, x0 - bb[0], y0 - bb[1], rot=rot)
            placed.add(ref)
            taken.append(crtyd(fp))
            u = crtyd(fp)[2] - (CX - SH_L / 2) + gap

    # Driver cells.
    def cell(m, u):
        vd = SH_VD
        put(f"U{m}", u, vd, 90)
        # 0603 sense resistors (250 mW): ISEN pads at u -/+ 0.8, GND pads outboard.
        # BOUT2 runs down between the ISEN pads, BOUT1 under R_B's body.
        put(f"R{2 * m - 1}", u - 1.575, vd + 3.6, 180)    # AISEN: pin 1 at u - 0.8
        put(f"R{2 * m}", u + 1.575, vd + 3.6, 0)          # BISEN: pin 1 at u + 0.8
        put(f"C{3 * m - 2}", u - 1.95, vd - 3.22, 180)     # VINT: pin 1 at u - 1.47
        put(f"C{3 * m}", u, vd - 3.22, 180)                # VM: V5 at u + 0.48, GND at u - 0.48
        put(f"C{3 * m - 1}", u + 1.95, vd - 3.22, 0)       # VCP: pin 1 at u + 1.47

    for conn, odd, even, uc in SH_PAIRS:
        put(conn, uc, SH_W - 2.4, 180)                     # signal pads toward the drivers
        cell(even, uc - 3.825)
        cell(odd, uc + 2.675)
        # Keep small parts out of the driver cells (cap bars down to the connector).
        taken.append([*sh_xy(uc - 7.075, 5.4), *sh_xy(uc + 5.925, SH_W)])

    # USB-C: mouth on the -x edge, courtyard front 0.5 mm past it.
    usb = fps["J1"]
    place(usb, *sh_xy(3.7, 10.0), rot=270)
    bb = usb.GetCourtyard(pcbnew.F_CrtYd).BBox()
    usb.Move(pcbnew.VECTOR2I(MM(sh_xy(-0.5, 0)[0]) - bb.GetLeft(), 0))
    placed.add("J1")
    taken.append(crtyd(usb))

    put_all({
        # BTN1 / BTN2 beside the receptacle.
        "SW1": (3.5, 2.4, 0), "SW2": (3.5, 17.6, 0),
        # ESD and series / CC resistors right behind the receptacle pads.
        "U12": (10.8, 10.0, 0), "R26": (13.25, 9.4, 0), "R27": (13.25, 10.6, 0),
        "R28": (10.2, 8.4, 0), "R29": (10.2, 11.6, 0),
        # Power path in the +y half: VBUS -> F1 -> D1 -> V5_IN -> Q1 -> V5,
        # LDO in the middle.
        "F1": (11.0, 17.7, 0), "D1": (17.7, 17.7, 180), "C52": (22.3, 17.7, 90),
        "Q1": (19.0, 13.9, 0), "C54": (15.6, 13.0, 0), "R30": (15.6, 14.4, 0),
        "U13": (17.2, 9.6, 0), "C53": (20.4, 9.6, 90), "C55": (21.8, 9.6, 90),
    })

    # MCU; core-regulator parts at the -x face (pins 61-65 at its -y end),
    # arranged as on the routed controller.
    um = SH_UM
    put("U11", um, 10.0, 90)
    put_all({
        # LX (pin 63) runs out past pins 61/62 to L1; VREG_VIN cap by pin 64;
        # 1V1 output cap at L1's far pin.
        "C44": (um - 6.85, 7.55, 180), "L1": (um - 7.0, 4.6, 90), "C48": (um - 7.0, 2.4, 180),
        "C49": (um - 9.4, 4.6, 90), "R24": (um - 10.6, 4.6, 90),
        # Crystal off the +x face: XIN / XOUT (pins 30 / 31) run out between
        # the pin-29 and pin-32 caps; XTAL_OUT goes round under the crystal.
        "Y1": (um + 11.05, 10.0, 0), "R25": (um + 8.35, 9.2, 90),
        "C50": (um + 9.95, 12.8, 270), "C51": (um + 12.15, 7.2, 90),
    })

    # MCU decoupling just outside each supply pin, perpendicular to the edge,
    # sliding along the edge until its courtyard clears the parts placed so far.
    u11 = fps["U11"]
    ox, oy = sh_xy(um, 10.0)
    pad_at = {p.GetNumber(): p.GetPosition() for p in u11.Pads()}
    decoupling = [("C35", "5"), ("C45", "10"), ("C36", "15"), ("C37", "24"), ("C38", "29"), ("C46", "32"),
                  ("C39", "41"), ("C40", "50"), ("C47", "51"), ("C41", "59"), ("C42", "68"), ("C43", "76")]
    for ref, pin in decoupling:
        q = pad_at[pin]
        px, py = pcbnew.ToMM(q.x) - ox, pcbnew.ToMM(q.y) - oy
        side = abs(px) > abs(py)
        fp = fps[ref]
        for along in sorted((k * 0.05 for k in range(-60, 61)), key=abs):
            if side:    # left/right edge: cap horizontal, supply pad toward the chip
                x, y, r = ox + px + 1.9 * (1 if px > 0 else -1), oy + py + along, 0 if px > 0 else 180
            else:       # top/bottom edge: cap vertical
                x, y, r = ox + px + along, oy + py + 1.9 * (1 if py > 0 else -1), 270 if py > 0 else 90
            place(fp, x, y, rot=r)
            if clear(crtyd(fp)):
                break
        else:
            raise SystemExit(f"no room for {ref} by U11 pin {pin}")
        taken.append(crtyd(fp))
        placed.add(ref)

    # Misc parts along the -y edge: BOOTSEL, Qwiic x2 and e-stop over
    # drivers M1-M4; panel buttons, 5 V in, status LED and LED power over M5-M10.
    # BOOTSEL sits low in its slot: a LADDER trace runs along v 2.64 above it.
    put("SW3", 13.6, 3.85, 180)
    row([("J2", 0), ("J3", 0), ("J7", 0)], 16.79)
    row([("J8", 0), ("J4", 0), ("D3", 180), ("U14", 0), ("D2", 0), ("J6", 0)], um + 7.0)
    # LED strip output at the +x end.
    put("J5", SH_L - 4.4, 9.0, 90)

    # Small parts: preferred spot, nudged to the nearest free one.
    for ref, (u, v, r) in {
        "R34": (9.6, 6.3, 0), "R35": (10.2, 13.6, 0), "R36": (12.6, 6.4, 0),     # buttons, BOOTSEL
        "R31": (21.0, 6.4, 0), "R32": (23.0, 6.4, 0), "JP2": (41.6, 4.0, 0), "JP1": (45.3, 4.2, 0),
        "JP4": (42.0, 1.5, 0), "R39": (45.0, 1.0, 0),                           # e-stop
        "R33": (um - 4.2, 0.95, 0), "C56": (um - 2.2, 0.95, 0), "R37": (um - 0.2, 0.95, 0),
        "R42": (um + 1.8, 0.95, 0), "R22": (um + 6.0, 0.95, 180), "R23": (um + 5.8, 0.95, 0),
        "C34": (um - 6.0, 1.0, 0),
        "TP1": (um + 8.0, 15.4, 0), "TP2": (um + 10.2, 15.4, 0), "TP3": (um + 12.4, 15.4, 0),
        "TP4": (um + 9.1, 17.6, 0), "TP5": (um - 9.5, 16.2, 0),
        "R40": (um + 9.0, 5.5, 0), "R41": (um + 11.0, 5.5, 0),                  # panel buttons
        "C59": (um + 19.0, 5.6, 0), "C57": (um + 31.0, 5.6, 0), "C58": (um + 34.0, 5.6, 0),
        "U15": (SH_L - 9.0, 12.0, 90), "C60": (SH_L - 9.0, 15.0, 0), "R38": (SH_L - 9.0, 8.8, 0),
        "F2": (SH_L - 3.6, 17.2, 0), "JP3": (SH_L - 9.0, 17.6, 0), "C61": (SH_L - 9.6, 5.6, 90),
        "C31": (SH_L - 9.6, 2.0, 0), "C32": (um - 4.0, 18.8, 0), "C33": (um + 5.0, 18.8, 0),
        "R21": (um - 9.0, 14.2, 0),
    }.items():
        put_near(ref, u, v, r)

    # In2: 3V3 under the MCU, with a thin finger along the -y edge to the LDO;
    # V5 everywhere else, cut out around it (0.3 mm gap) rather than overlapping.
    e, g = 0.3, 0.3
    i0, i1, i3 = um - 11.4, um + 7.6, 17.4
    v3 = [(14.0, e), (i1, e), (i1, i3), (i0, i3), (i0, 1.9), (24.5, 1.9), (24.5, 8.4), (14.0, 8.4)]
    v5 = [(e, e), (14.0 - g, e), (14.0 - g, 8.4 + g), (24.5 + g, 8.4 + g), (24.5 + g, 1.9 + g),
          (i0 - g, 1.9 + g), (i0 - g, i3 + g), (i1 + g, i3 + g), (i1 + g, e), (SH_L - e, e),
          (SH_L - e, SH_W - e), (e, SH_W - e)]
    nets = {p.GetNetname(): p.GetNet() for fp in fps.values() for p in fp.Pads()}
    zone(board, nets["+3V3"], pcbnew.In2_Cu, 0, 0, pts=[sh_xy(*q) for q in v3], priority=1)
    zone(board, nets["V5"], pcbnew.In2_Cu, 0, 0, pts=[sh_xy(*q) for q in v5])
    return placed


# ---------------------------------------------------------------------------
# single_sh_proto: the single_sh circuit with every passive 0603 (JLCPCB basic)
# and JLCPCB's cheapest via class (0.3 mm drill, 0.52 mm pads). Same linear arrangement,
# outline grown to 150 x 30 mm so the larger parts and vias fit:
#
#   USB-C + power | drivers M4-M1 | MCU | drivers M10-M5 | LED strip
#
# Driver cell: AT8833 rotated 90 as on single_sh, but the VINT / VM / VCP caps
# stand vertically above it (0603 is too wide for a row of three), and a
# pair's two drivers sit 7.5 mm apart (6.5 on single_sh) for via room.
# ---------------------------------------------------------------------------

SP_L, SP_W = 150.0, 30.0
SP_VD = SP_W - 10.3                         # driver row centre (v)
SP_PAIRS = [("J12", 3, 4, 36.0), ("J11", 1, 2, 52.0),                       # (conn, odd, even, u)
            ("J15", 9, 10, 94.0), ("J14", 7, 8, 110.0), ("J13", 5, 6, 126.0)]
SP_UM = 73.0                                # MCU centre (u)
SP_VM = 14.0                                # MCU centre (v)


def sp_xy(u, v):
    return CX - SP_L / 2 + u, CY - SP_W / 2 + v


def single_sh_proto(board, fps):
    outline(board, SP_L, SP_W, 1.0)
    placed = set()
    taken = []

    def crtyd(fp):
        bb = fp.GetCourtyard(pcbnew.F_CrtYd).BBox()
        return [pcbnew.ToMM(v) for v in (bb.GetLeft(), bb.GetTop(), bb.GetRight(), bb.GetBottom())]

    def clear(a, margin=0.0):
        x0, y0 = sp_xy(margin, margin)
        x1, y1 = sp_xy(SP_L - margin, SP_W - margin)
        if a[0] < x0 or a[1] < y0 or a[2] > x1 or a[3] > y1:
            return False
        return all(a[2] <= b[0] or b[2] <= a[0] or a[3] <= b[1] or b[3] <= a[1] for b in taken)

    def put(ref, u, v, rot=0):
        place(fps[ref], *sp_xy(u, v), rot=rot)
        placed.add(ref)
        taken.append(crtyd(fps[ref]))

    def put_near(ref, u, v, rot=0, reach=8.0, gap=0.25):
        """Place at (u, v), or the nearest spot (0.1 mm grid spiral) whose courtyard clears the rest by gap."""
        fp = fps[ref]
        steps = int(reach / 0.1)
        cands = sorted(((du, dv) for du in range(-steps, steps + 1) for dv in range(-steps, steps + 1)),
                       key=lambda d: d[0] ** 2 + d[1] ** 2)
        for du, dv in cands:
            place(fp, *sp_xy(u + du * 0.1, v + dv * 0.1), rot=rot)
            a = crtyd(fp)
            if clear([a[0] - gap, a[1] - gap, a[2] + gap, a[3] + gap], 0.1):
                if du * du + dv * dv > 100:
                    print(f"  {ref}: moved {du * 0.1:.1f}, {dv * 0.1:.1f}")
                placed.add(ref)
                taken.append(a)
                return
        print(f"  {ref}: no room near ({u}, {v}), left off the board")

    def tidy(parts):
        """Place at exactly (u, v); report (don't move) parts whose courtyard hits another."""
        for ref, (u, v, r) in parts.items():
            place(fps[ref], *sp_xy(u, v), rot=r)
            if not clear(crtyd(fps[ref]), 0.1):
                print(f"  {ref}: courtyard clash at ({u}, {v})")
            placed.add(ref)
            taken.append(crtyd(fps[ref]))

    def row(parts, u, v_edge=0.05, gap=0.6):
        """Pack parts left to right along the -y edge from u, courtyards touching v_edge."""
        for ref, rot in parts:
            fp = fps[ref]
            place(fp, 0, 0, rot=rot)
            bb = crtyd(fp)
            x0, y0 = sp_xy(u, v_edge)
            place(fp, x0 - bb[0], y0 - bb[1], rot=rot)
            placed.add(ref)
            taken.append(crtyd(fp))
            u = crtyd(fp)[2] - (CX - SP_L / 2) + gap

    def cell(m, u):
        vd = SP_VD
        put(f"U{m}", u, vd, 90)
        # Sense resistors: ISEN pads at u -/+ 0.975, straight below the chip's
        # ISEN pins; the 1.15 mm gap between them takes BOUT2 (straight down)
        # and the AOUT2 via; BOUT1 passes between R_B's pads.
        put(f"R{2 * m - 1}", u - 1.8, vd + 3.6, 180)      # AISEN
        put(f"R{2 * m}", u + 1.8, vd + 3.6, 0)            # BISEN
        # Caps vertical, pin 1 toward the chip: VINT over pin 12, VM over
        # pins 10/11, VCP over pin 9. 1.15 mm from the chip's pins to the cap
        # pads: room for pin 10's 0.5 mm V5 via with 0.12 mm to both.
        put(f"C{3 * m - 2}", u - 1.75, vd - 4.75, 90)       # VINT
        put(f"C{3 * m}", u, vd - 4.75, 90)                  # VM (V5 pin 1)
        put(f"C{3 * m - 1}", u + 1.75, vd - 4.75, 90)       # VCP

    for conn, odd, even, uc in SP_PAIRS:
        put(conn, uc, SP_W - 2.4, 180)
        cell(even, uc - 4.325)
        cell(odd, uc + 3.175)
        # Keep small parts out of the driver cells and the routing space round them.
        taken.append([*sp_xy(uc - 8.0, SP_VD - 6.6), *sp_xy(uc + 6.9, SP_W)])

    usb = fps["J1"]
    # Shell slots 0.1 mm wider than the library's, pads grown to keep a 0.2 mm
    # copper ring all round.
    for p in usb.Pads():
        if p.GetAttribute() == pcbnew.PAD_ATTRIB_PTH:
            d = p.GetDrillSize()
            d = pcbnew.VECTOR2I(d.x + MM(0.1), d.y)
            p.SetDrillSize(d)
            p.SetSize(pcbnew.VECTOR2I(max(p.GetSize().x, d.x + MM(0.4)), max(p.GetSize().y, d.y + MM(0.4))))
    place(usb, *sp_xy(3.7, SP_W / 2), rot=270)
    bb = usb.GetCourtyard(pcbnew.F_CrtYd).BBox()
    usb.Move(pcbnew.VECTOR2I(MM(sp_xy(-0.5, 0)[0]) - bb.GetLeft(), 0))
    placed.add("J1")
    taken.append(crtyd(usb))

    # USB / power end. Columns and rows on a 0.5 mm grid, passives horizontal.
    vc = SP_W / 2
    tidy({
        "SW1": (3.5, 2.4, 0), "SW2": (3.5, SP_W - 2.4, 0),
        # CC pull-downs above / below the ESD diode, USB series resistors after it.
        "R28": (11.5, vc - 3.0, 0), "U12": (11.5, vc, 0), "R29": (11.5, vc + 3.0, 0),
        "R26": (15.0, vc - 0.9, 0), "R27": (15.0, vc + 0.9, 0),
        # LDO row: input cap, LDO, output cap.
        "C53": (18.5, vc - 2.5, 0), "U13": (22.5, vc - 2.5, 0), "C55": (26.5, vc - 2.5, 0),
        # Soft-start row: gate cap and resistor, then the FET.
        "C54": (19.0, vc + 6.0, 0), "R30": (19.0, vc + 8.0, 0), "Q1": (22.5, vc + 7.0, 0),
        # VBUS -> F1 -> D1 -> V5_IN along the edge.
        "F1": (11.5, SP_W - 2.3, 0), "D1": (19.0, SP_W - 2.4, 180), "C52": (24.5, SP_W - 2.4, 90),
    })

    # MCU and core regulator (pins 61-65 at its -x face, -y end).
    um, vm = SP_UM, SP_VM
    put("U11", um, vm, 90)
    for ref, (u, v, r) in {
        "C44": (um - 7.4, vm - 2.6, 180), "L1": (um - 7.6, vm - 5.8, 90), "C48": (um - 7.6, vm - 8.6, 180),
        "C49": (um - 10.2, vm - 5.8, 90), "R24": (um - 11.9, vm - 5.8, 90),
        # Crystal above the MCU's +x corner, clear of the M5-M10 input buses that
        # fan out from the +x face; XIN / XOUT (pins 30 / 31) run up to it.
        "Y1": (um + 10.0, vm - 7.0, 0), "R25": (um + 7.2, vm - 7.0, 90),
        "C50": (um + 12.8, vm - 7.0, 90), "C51": (um + 10.0, vm - 9.6, 0),
    }.items():
        put_near(ref, u, v, r, reach=3.0)

    u11 = fps["U11"]
    ox, oy = sp_xy(um, vm)
    pad_at = {p.GetNumber(): p.GetPosition() for p in u11.Pads()}
    decoupling = [("C35", "5"), ("C45", "10"), ("C36", "15"), ("C37", "24"), ("C38", "29"), ("C46", "32"),
                  ("C39", "41"), ("C40", "50"), ("C47", "51"), ("C41", "59"), ("C42", "68"), ("C43", "76")]
    for ref, pin in decoupling:
        q = pad_at[pin]
        px, py = pcbnew.ToMM(q.x) - ox, pcbnew.ToMM(q.y) - oy
        side = abs(px) > abs(py)
        fp = fps[ref]
        # 2.6 mm out: room for a 0.45 mm via ring between the pins and the cap.
        for along in sorted((k * 0.05 for k in range(-80, 81)), key=abs):
            if side:
                x, y, r = ox + px + 2.6 * (1 if px > 0 else -1), oy + py + along, 0 if px > 0 else 180
            else:
                x, y, r = ox + px + along, oy + py + 2.6 * (1 if py > 0 else -1), 270 if py > 0 else 90
            place(fp, x, y, rot=r)
            a = crtyd(fp)
            if clear([a[0] - 0.15, a[1] - 0.15, a[2] + 0.15, a[3] + 0.15]):
                break
        else:
            raise SystemExit(f"no room for {ref} by U11 pin {pin}")
        taken.append(crtyd(fp))
        placed.add(ref)

    # Misc parts along the -y edge, over the drivers.
    put("SW3", 14.0, 2.4, 180)              # in line with SW1
    row([("J2", 0), ("J3", 0), ("J7", 0)], 28.0)
    row([("J8", 0), ("J4", 0), ("D3", 180), ("U14", 0), ("D2", 0), ("J6", 0)], um + 14.0)
    # D2 is the 5050 WS2812B (the 2020 is JLCPCB Standard-PCBA only): set by hand,
    # J6 shifted right to make room. The routed board also trims D2's courtyard to
    # 0.05 mm round the pads (the library one overlaps U14's) and keeps only a
    # pin-1 silk bar.
    put("D2", 119.35, 2.85, 180)
    put("J6", 125.485, 2.295, 0)
    put("J5", SP_L - 4.4, SP_W / 2 - 1.0, 90)

    tidy({
        # Under the -y edge connectors, v = 7: each part below what it serves.
        "R34": (8.0, 7.0, 0), "R36": (14.0, 7.0, 0),                           # BTN1, BOOTSEL
        "R35": (8.0, SP_W - 7.0, 0),                                            # BTN2
        "R31": (30.0, 7.0, 0), "R32": (33.2, 7.0, 0), "JP2": (37.0, 7.0, 0),   # I2C pull-ups
        "JP1": (41.0, 7.0, 0),                                                  # Qwiic supply
        "JP4": (46.0, 7.0, 0), "R39": (49.5, 7.0, 0),                           # e-stop
        # MCU sense / ladder / ID row along the edge: VMOT divider, LADDER, BOARD_ID.
        # Standing vertically, 2 mm pitch.
        # R22 (V5 side of the divider) just outside the In2 3V3 island, V5 pin
        # down, so its V5 via can land in the plane.
        "R22": (um - 14.5, 1.9, 90), "R23": (um - 4.0, 1.9, 90), "C34": (um - 2.0, 1.9, 90),
        "R33": (um + 0.0, 1.9, 90), "C56": (um + 2.0, 1.9, 90),
        "R37": (um + 4.0, 1.9, 90), "R42": (um + 6.0, 1.9, 90),
        "R21": (um - 10.0, SP_W - 8.0, 0), "TP5": (um - 10.0, SP_W - 5.5, 0),  # nSLEEP, 3V3 pad
        "C32": (um - 7.0, SP_W - 2.4, 0), "C33": (um + 7.0, SP_W - 2.4, 0),    # V5 bulk
        "TP1": (um - 3.3, SP_W - 2.4, 0), "TP2": (um - 1.1, SP_W - 2.4, 0),     # SWD, RUN, GND
        "TP3": (um + 1.1, SP_W - 2.4, 0), "TP4": (um + 3.3, SP_W - 2.4, 0),     # along the edge
        "R40": (89.0, 7.0, 0), "R41": (92.2, 7.0, 0),                           # panel buttons
        "C59": (99.5, 7.0, 0),                                                  # 5 V in
        "C57": (114.0, 7.0, 0), "C58": (118.0, 7.0, 0),                         # LED buffer, LED
        "C31": (129.5, 7.0, 0), "C61": (132.7, 7.0, 0),                         # V5 bulk, strip power
        # LED strip column.
        "R38": (SP_L - 10.0, 12.5, 0), "U15": (SP_L - 10.0, 16.0, 90), "C60": (SP_L - 10.0, 19.5, 0),
        "JP3": (SP_L - 10.0, SP_W - 2.4, 0), "F2": (SP_L - 3.6, SP_W - 2.8, 0),
    })

    # In2: 3V3 under the MCU, joined to the LDO by a finger along the -y edge;
    # V5 everywhere else, cut out round it.
    e, g = 0.3, 0.3
    i0, i1, i3 = um - 12.5, um + 9.0, SP_W - 3.5
    l0, l1, lv = 15.0, 27.0, 10.5            # LDO pocket (u0, u1, v)
    v3 = [(l0, e), (i1, e), (i1, i3), (i0, i3), (i0, 2.4), (l1, 2.4), (l1, lv), (l0, lv)]
    v5 = [(e, e), (l0 - g, e), (l0 - g, lv + g), (l1 + g, lv + g), (l1 + g, 2.4 + g),
          (i0 - g, 2.4 + g), (i0 - g, i3 + g), (i1 + g, i3 + g), (i1 + g, e), (SP_L - e, e),
          (SP_L - e, SP_W - e), (e, SP_W - e)]
    nets = {p.GetNetname(): p.GetNet() for fp in fps.values() for p in fp.Pads()}
    zone(board, nets["+3V3"], pcbnew.In2_Cu, 0, 0, pts=[sp_xy(*q) for q in v3], priority=1, clearance=0.2)
    zone(board, nets["V5"], pcbnew.In2_Cu, 0, 0, pts=[sp_xy(*q) for q in v5], clearance=0.2)
    # +1V1 under the MCU body (inside its pin ring): the four +1V1 pins, one per
    # face, each drop to a via just inboard of the pin instead of ringing the chip.
    k = 4.1
    zone(board, nets["+1V1"], pcbnew.In2_Cu, 0, 0, priority=2, clearance=0.2,
         pts=[sp_xy(um + du, vm + dv) for du, dv in ((-k, -k), (k, -k), (k, k), (-k, k))])
    return placed


BOARDS = {"controller": controller, "driver": driver, "single_sh": single_sh, "single_sh_proto": single_sh_proto}
LAYERS = {"controller": 4, "driver": 4, "single_sh": 4, "single_sh_proto": 4}
# Outline (w, h) in mm and board thickness.
SIZES = {"controller": (30.0, 30.0, 1.0), "driver": (30.0, 30.0, 1.0), "single_sh": (SH_L, SH_W, 1.6),
         "single_sh_proto": (SP_L, SP_W, 1.6)}
ZONES = {"controller": [("/GND", pcbnew.In1_Cu), ("/+3V3", pcbnew.In2_Cu)],
         "driver": [("/GND", pcbnew.In1_Cu), ("/V5", pcbnew.In2_Cu)],
         "single_sh": [("GND", pcbnew.In1_Cu)], "single_sh_proto": [("GND", pcbnew.In1_Cu)]}      # global power nets; In2 zones: see single_sh()


def build(name, out_dir):
    """Create out_dir/<name>.kicad_pcb from out_dir/<name>.kicad_sch."""
    pcb_path = os.path.join(out_dir, name + ".kicad_pcb")
    if os.path.exists(pcb_path):
        raise SystemExit(f"{pcb_path} exists; layout is hand-edited from here, not regenerated")

    comps, nets, pads = read_netlist(os.path.join(out_dir, name + ".kicad_sch"))
    board = pcbnew.NewBoard(pcb_path)
    board.SetCopperLayerCount(LAYERS[name])

    netinfo = {}
    for n in nets:
        ni = pcbnew.NETINFO_ITEM(board, n)
        board.Add(ni)
        netinfo[n] = ni

    fps = {}
    for c in comps:
        fp = load_footprint(c["footprint"])
        fp.SetReference(c["ref"])
        fp.SetValue(c["value"])
        fp.SetPath(pcbnew.KIID_PATH("/" + c["uuid"]))
        for k, v in c["fields"].items():
            fp.SetField(k, v)
        for f in fp.GetFields():
            if f.GetName() in c["fields"]:
                f.SetVisible(False)
        # Match the symbols: solder jumpers and test pads are copper only, so
        # they stay out of the BOM and the placement file.
        fp.SetExcludedFromBOM(not c["in_bom"])
        fp.SetExcludedFromPosFiles(not c["in_bom"])
        board.Add(fp)
        for pad in fp.Pads():
            net = pads.get((c["ref"], pad.GetNumber()))
            if net:
                pad.SetNet(netinfo[net])
        fps[c["ref"]] = fp

    w, h, thickness = SIZES[name]
    placed = BOARDS[name](board, fps)

    # Park everything else to the right of the board, grouped by ref prefix.
    x0, y = CX + w / 2 + 5, CY - h / 2
    x, row_h = x0, 0
    for ref in sorted((r for r in fps if r not in placed),
                      key=lambda r: (r.rstrip("0123456789"), int(r[len(r.rstrip("0123456789")):] or 0))):
        fp = fps[ref]
        bb = fp.GetBoundingBox(False)
        bw, bh = pcbnew.ToMM(bb.GetWidth()) + 1, pcbnew.ToMM(bb.GetHeight()) + 1
        if x + bw > x0 + 40:
            x, y, row_h = x0, y + row_h, 0
        place(fp, x + bw / 2, y + bh / 2)
        x += bw
        row_h = max(row_h, bh)

    jlc_rules(board, thickness, name in BIG_VIAS)
    for net, layer in ZONES[name]:
        # 0.2 mm on the big-via board: the default 0.5 mm round dense via
        # fields (under the MCU) splits the planes into islands.
        zone(board, netinfo[net], layer, w, h, clearance=0.2 if name in BIG_VIAS else None)
    # Zones are left unfilled: the scripted filler doesn't apply the board
    # rules (edge and hole clearance). Press B in the PCB editor to fill.

    board.Save(pcb_path)
    jlc_netclasses(os.path.join(out_dir, name + ".kicad_pro"), name in BIG_VIAS)  # after Save, which rewrites the project
    print(f"{pcb_path}: {len(fps)} footprints, {len(placed)} placed, {len(nets)} nets")


def main():
    name = sys.argv[1] if len(sys.argv) > 1 else "controller"
    build(name, os.path.join(HW, name))


if __name__ == "__main__":
    main()
