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
        comps.append({"ref": field(c, "ref"), "value": field(c, "value"),
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


def zone(board, net, layer, w, h, inset=0.3):
    z = pcbnew.ZONE(board)
    z.SetLayer(layer)
    z.SetNet(net)
    x0, y0 = CX - w / 2 + inset, CY - h / 2 + inset
    x1, y1 = CX + w / 2 - inset, CY + h / 2 - inset
    poly = z.Outline()
    poly.NewOutline()
    for x, y in ((x0, y0), (x1, y0), (x1, y1), (x0, y1)):
        poly.Append(MM(x), MM(y))
    z.SetMinThickness(MM(0.15))
    z.SetPadConnection(pcbnew.ZONE_CONNECTION_THERMAL)
    board.Add(z)


def place(fp, x, y, rot=0, bottom=False):
    if bottom:
        fp.Flip(fp.GetPosition(), pcbnew.FLIP_DIRECTION_TOP_BOTTOM)
    fp.SetPosition(pcbnew.VECTOR2I(MM(x), MM(y)))
    fp.SetOrientationDegrees(rot)


def jlc_rules(board, _unused=None):
    """JLCPCB 4-layer capabilities with some margin."""
    ds = board.GetDesignSettings()
    ds.SetBoardThickness(MM(1.0))
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


def jlc_netclasses(pro_path):
    """Net classes live in the project file."""
    pro = json.load(open(pro_path))
    ns = pro.setdefault("net_settings", {})
    ns["classes"] = [
        {"name": "Default", "clearance": 0.1, "track_width": 0.15, "via_diameter": 0.45, "via_drill": 0.2,
         "diff_pair_width": 0.2, "diff_pair_gap": 0.15, "microvia_diameter": 0.3, "microvia_drill": 0.1,
         "wire_width": 6, "bus_width": 12, "line_style": 0, "pcb_color": "rgba(0, 0, 0, 0.000)",
         "schematic_color": "rgba(0, 0, 0, 0.000)", "priority": 2147483647},
        {"name": "Power", "clearance": 0.1, "track_width": 0.4, "via_diameter": 0.6, "via_drill": 0.3,
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


BOARDS = {"controller": controller}


def main():
    name = sys.argv[1] if len(sys.argv) > 1 else "controller"
    out_dir = os.path.join(HW, name)
    pcb_path = os.path.join(out_dir, name + ".kicad_pcb")
    if os.path.exists(pcb_path):
        raise SystemExit(f"{pcb_path} exists; layout is hand-edited from here, not regenerated")

    comps, nets, pads = read_netlist(os.path.join(out_dir, name + ".kicad_sch"))
    board = pcbnew.NewBoard(pcb_path)
    board.SetCopperLayerCount(4)

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
        fp.SetExcludedFromBOM(False)  # match the symbols (all are in the BOM)
        board.Add(fp)
        for pad in fp.Pads():
            net = pads.get((c["ref"], pad.GetNumber()))
            if net:
                pad.SetNet(netinfo[net])
        fps[c["ref"]] = fp

    placed = BOARDS[name](board, fps)

    # Park everything else to the right of the board, grouped by ref prefix.
    x0, y = CX + 20, CY - 12.5
    x, row_h = x0, 0
    for ref in sorted((r for r in fps if r not in placed),
                      key=lambda r: (r.rstrip("0123456789"), int(r[len(r.rstrip("0123456789")):] or 0))):
        fp = fps[ref]
        bb = fp.GetBoundingBox(False)
        w, h = pcbnew.ToMM(bb.GetWidth()) + 1, pcbnew.ToMM(bb.GetHeight()) + 1
        if x + w > x0 + 40:
            x, y, row_h = x0, y + row_h, 0
        place(fp, x + w / 2, y + h / 2)
        x += w
        row_h = max(row_h, h)

    jlc_rules(board, None)
    zone(board, netinfo["/GND"], pcbnew.In1_Cu, 30, 30)
    zone(board, netinfo["/+3V3"], pcbnew.In2_Cu, 30, 30)
    # Zones are left unfilled: the scripted filler doesn't apply the board
    # rules (edge and hole clearance). Press B in the PCB editor to fill.

    board.Save(pcb_path)
    jlc_netclasses(os.path.join(out_dir, name + ".kicad_pro"))  # after Save, which rewrites the project
    board.Save(pcb_path)
    print(f"{pcb_path}: {len(fps)} footprints, {len(placed)} placed, {len(nets)} nets")


if __name__ == "__main__":
    main()
