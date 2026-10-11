"""Place the single_sh_proto bring-up test pads (TP6-TP17) on the routed board.

    <KiCad python> gen/add_testpoints.py

Each pad is linked to its schematic symbol (so Update PCB from Schematic and
the parity check keep working), put on the side given in SIDES, and joined by
a short stub to an existing via of its net. The script searches round every
via of the net for the nearest spot where the 1 mm pad and the stub clear all
other copper, the board edge and (on top) the other parts' courtyards; the pad
keeps clear of the via itself (no via in pad). A silkscreen label with the
function goes beside each pad. Pads already on the board are left alone.
"""

import math
import os
import sys

import pcbnew

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import pcbgen  # noqa: E402

HW = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BDIR = os.path.join(HW, "single_sh_proto")
PCB = os.path.join(BDIR, "single_sh_proto.kicad_pcb")
MM, T = pcbnew.FromMM, pcbnew.ToMM

# ref -> (side, label). Supplies on top, by the power path / MCU; shared
# signals underneath, where probes reach them with the board in a fixture.
SIDES = {
    "TP6": ("F", "VBUS"), "TP7": ("F", "V5IN"), "TP8": ("F", "V5"), "TP9": ("F", "1V1"), "TP10": ("F", "GND"),
    "TP11": ("B", "LADDER"), "TP12": ("B", "VMOT"), "TP13": ("B", "NSLP"), "TP14": ("B", "SDA"),
    "TP15": ("B", "SCL"), "TP16": ("B", "LED"), "TP17": ("B", "GND"),
}
PAD_R, CLR, STUB_W = 0.5, 0.2, 0.25
VIA_GAP = 0.15                # pad edge to its own via (no via in pad)

comps, nets, pads = pcbgen.read_netlist(os.path.join(BDIR, "single_sh_proto.kicad_sch"))
by_ref = {c["ref"]: c for c in comps}
b = pcbnew.LoadBoard(PCB)
have = {f.GetReference() for f in b.GetFootprints()}
x0, y0 = pcbgen.sp_xy(0, 0)


def seg_d(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    L = dx * dx + dy * dy
    t = 0 if L == 0 else max(0, min(1, ((px - ax) * dx + (py - ay) * dy) / L))
    return math.hypot(px - ax - t * dx, py - ay - t * dy)


def obstacles(layer, net):
    """Other-net copper on a layer as (kind, geometry, half-width); vias and holes on every layer."""
    obs = []
    for t in b.GetTracks():
        if t.GetNetname() == net:
            continue
        if t.GetClass() == "PCB_VIA":
            obs.append(("pt", (T(t.GetPosition().x), T(t.GetPosition().y)), T(t.GetWidth(pcbnew.F_Cu)) / 2))
        elif t.GetLayer() == layer:
            obs.append(("seg", (T(t.GetStart().x), T(t.GetStart().y), T(t.GetEnd().x), T(t.GetEnd().y)),
                        T(t.GetWidth()) / 2))
    for f in b.GetFootprints():
        for p in f.Pads():
            if (p.GetNetname() == net and p.GetNetname()) or not (p.IsOnLayer(layer) or p.GetDrillSize().x):
                continue
            bb = p.GetBoundingBox()
            obs.append(("box", (T(bb.GetLeft()), T(bb.GetTop()), T(bb.GetRight()), T(bb.GetBottom())), 0))
    return obs


def dist(o, x, y):
    k, g, r = o
    if k == "pt":
        return math.hypot(x - g[0], y - g[1]) - r
    if k == "seg":
        return seg_d(x, y, *g) - r
    return math.hypot(max(g[0] - x, 0, x - g[2]), max(g[1] - y, 0, y - g[3]))


def seg_clear(obs, a, c, w):
    for o in obs:
        k, g, r = o
        n = max(2, int(math.hypot(c[0] - a[0], c[1] - a[1]) / 0.05))
        for i in range(n + 1):
            x, y = a[0] + (c[0] - a[0]) * i / n, a[1] + (c[1] - a[1]) * i / n
            if dist(o, x, y) < w / 2 + CLR:
                return False
    return True


courtyards = []
for f in b.GetFootprints():
    bb = f.GetCourtyard(pcbnew.F_CrtYd).BBox()
    if bb.GetWidth():
        courtyards.append((T(bb.GetLeft()), T(bb.GetTop()), T(bb.GetRight()), T(bb.GetBottom())))
placed_tp = []


def spot(net, side):
    layer = pcbnew.F_Cu if side == "F" else pcbnew.B_Cu
    obs = obstacles(layer, net)
    anchors = [(T(v.GetPosition().x), T(v.GetPosition().y), T(v.GetWidth(pcbnew.F_Cu)) / 2) for v in b.GetTracks()
               if v.GetClass() == "PCB_VIA" and v.GetNetname() == net]
    for t in b.GetTracks():
        if t.GetClass() != "PCB_VIA" and t.GetNetname() == net and t.GetLayer() == layer:
            anchors += [(T(q.x), T(q.y), 0.0) for q in (t.GetStart(), t.GetEnd())]
    for f in b.GetFootprints():
        for p in f.Pads():
            if p.GetNetname() == net and p.IsOnLayer(layer) and not p.GetDrillSize().x and f.GetReference() not in SIDES:
                bb = p.GetBoundingBox()
                anchors.append((T(p.GetPosition().x), T(p.GetPosition().y), max(T(bb.GetWidth()), T(bb.GetHeight())) / 2))
    own_vias = [(T(v.GetPosition().x), T(v.GetPosition().y), T(v.GetWidth(pcbnew.F_Cu)) / 2) for v in b.GetTracks()
                if v.GetClass() == "PCB_VIA" and v.GetNetname() == net]
    best = None
    for vx, vy, ar in anchors:
        for d in (0.9, 1.1, 1.3, 1.6, 2.0, 2.5, 3.0):
            for k in range(8):                       # stubs at 0 / 45 / 90 degrees only
                a = math.pi * k / 4
                x, y = vx + d * math.cos(a), vy + d * math.sin(a)
                u, v = x - x0, y - y0
                if not (PAD_R + 0.6 < u < pcbgen.SP_L - PAD_R - 0.6 and PAD_R + 0.6 < v < pcbgen.SP_W - PAD_R - 0.6):
                    continue
                if math.hypot(x - vx, y - vy) < PAD_R + ar + VIA_GAP:
                    continue
                if any(math.hypot(x - qx, y - qy) < PAD_R + qr + VIA_GAP for qx, qy, qr in own_vias):
                    continue                         # no via in (or against) the pad, own net included
                if any(dist(o, x, y) < PAD_R + CLR for o in obs):
                    continue
                if side == "F" and any(c[0] - 1.0 < x < c[2] + 1.0 and c[1] - 1.0 < y < c[3] + 1.0
                                       for c in courtyards):
                    continue
                if any(math.hypot(x - px, y - py) < 2.6 for px, py in placed_tp):
                    continue
                if not seg_clear(obs, (vx, vy), (x, y), STUB_W):
                    continue
                if best is None or d < best[0]:
                    best = (d, (vx, vy), (x, y))

    return best


fp_lib = "/Applications/KiCad/KiCad.app/Contents/SharedSupport/footprints/TestPoint.pretty"
added = []
for ref, (side, label) in SIDES.items():
    if ref in have:
        continue
    c = by_ref[ref]
    net = pads[(ref, "1")]
    found = spot(net, side)
    if not found:                       # no room on its preferred side: try the other
        side = "B" if side == "F" else "F"
        found = spot(net, side)
    if not found:
        print(f"{ref} {net}: no room")
        continue
    _, (vx, vy), (x, y) = found
    fp = pcbnew.FootprintLoad(fp_lib, "TestPoint_Pad_D1.0mm")
    fp.SetFPID(pcbnew.LIB_ID("TestPoint", "TestPoint_Pad_D1.0mm"))
    fp.SetReference(ref)
    fp.SetValue(c["value"])
    fp.SetPath(pcbnew.KIID_PATH("/" + c["uuid"]))
    fp.SetExcludedFromBOM(True)
    fp.SetExcludedFromPosFiles(True)
    b.Add(fp)
    if side == "B":
        fp.Flip(fp.GetPosition(), pcbnew.FLIP_DIRECTION_TOP_BOTTOM)
    fp.SetPosition(pcbnew.VECTOR2I(MM(x), MM(y)))
    nobj = b.FindNet(net)
    for p in fp.Pads():
        p.SetNet(nobj)
    fp.Reference().SetVisible(False)
    fp.Value().SetVisible(False)
    t = pcbnew.PCB_TRACK(b)
    t.SetStart(pcbnew.VECTOR2I(MM(vx), MM(vy)))
    t.SetEnd(pcbnew.VECTOR2I(MM(x), MM(y)))
    t.SetLayer(pcbnew.F_Cu if side == "F" else pcbnew.B_Cu)
    t.SetWidth(MM(STUB_W))
    t.SetNet(nobj)
    b.Add(t)
    # Label: on the pad's own silkscreen, at the first spot round it that keeps
    # 0.15 mm from pads, holes, other silkscreen and the board edge.
    lab = pcbnew.PCB_TEXT(b)
    lab.SetText(label)
    silk = pcbnew.F_SilkS if side == "F" else pcbnew.B_SilkS
    lab.SetLayer(silk)
    lab.SetMirrored(side == "B")
    lab.SetTextSize(pcbnew.VECTOR2I(MM(0.8), MM(0.8)))
    lab.SetTextThickness(MM(0.15))
    lab.SetHorizJustify(pcbnew.GR_TEXT_H_ALIGN_CENTER)
    b.Add(lab)
    mask = pcbnew.F_Mask if side == "F" else pcbnew.B_Mask
    blockers = [p.GetEffectiveShape(mask) for f in b.GetFootprints() for p in f.Pads()
                if p.IsOnLayer(mask) or p.GetDrillSize().x]
    blockers += [pcbnew.SHAPE_CIRCLE(v.GetPosition(), v.GetDrillValue() // 2) for v in b.GetTracks()
                 if v.GetClass() == "PCB_VIA"]
    blockers += [d.GetEffectiveShape(silk) for d in b.GetDrawings() if d.GetLayer() == silk and d is not lab
                 and d.m_Uuid != lab.m_Uuid]
    blockers += [g.GetEffectiveShape(silk) for f in b.GetFootprints() for g in f.GraphicalItems()
                 if g.GetLayer() == silk]
    ok = False
    for dx, dy in ((0, 1.3), (0, -1.3), (1.9, 0), (-1.9, 0), (0, 2.1), (0, -2.1), (2.6, 0), (-2.6, 0)):
        lab.SetPosition(pcbnew.VECTOR2I(MM(x + dx), MM(y + dy)))
        bb = lab.GetBoundingBox()
        if T(bb.GetLeft()) < x0 + 0.3 or T(bb.GetRight()) > x0 + pcbgen.SP_L - 0.3 or \
                T(bb.GetTop()) < y0 + 0.3 or T(bb.GetBottom()) > y0 + pcbgen.SP_W - 0.3:
            continue
        sh = lab.GetEffectiveShape()
        if not any(sh.Collide(o, MM(0.15)) for o in blockers):
            ok = True
            break
    if not ok:
        lab.SetText(ref)
        lab.SetTextSize(pcbnew.VECTOR2I(MM(0.7), MM(0.7)))
        for dx, dy in ((0, 1.2), (0, -1.2), (1.8, 0), (-1.8, 0)):
            lab.SetPosition(pcbnew.VECTOR2I(MM(x + dx), MM(y + dy)))
            if not any(lab.GetEffectiveShape().Collide(o, MM(0.15)) for o in blockers):
                ok = True
                break
    if not ok:
        b.Remove(lab)
        print(f"  {ref}: no room for a label")
    placed_tp.append((x, y))
    if side == "F":
        courtyards.append((x - 1.0, y - 1.0, x + 1.0, y + 1.0))
    added.append(f"{ref} {label} ({side}) at u {x - x0:.1f}, v {y - y0:.1f}")

pcbnew.ZONE_FILLER(b).Fill(b.Zones())
b.Save(PCB)
print("\n".join(added))
