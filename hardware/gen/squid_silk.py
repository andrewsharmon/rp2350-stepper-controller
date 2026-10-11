"""Back-silkscreen art for single_sh_proto: a squid whose ten tentacles reach
the ten motor drivers, plus the "ASH+C" / "2026" mark.

    <KiCad python> gen/squid_silk.py [board]

Draws on B.Silkscreen of the routed board in place, replacing an earlier run
(its items are kept in a group named "squid art"). Lines are cut into short
pieces and any piece that would come within JLCPCB's silkscreen clearance of a
hole or a back-side pad opening is dropped, so the art never needs the
routing to move.
"""

import math
import os
import sys

import pcbnew

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import pcbgen  # noqa: E402

HW = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PCB = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HW, "single_sh_proto", "single_sh_proto.kicad_pcb")
MM, T = pcbnew.FromMM, pcbnew.ToMM
W = 0.18                      # line width
KEEP = 0.13 + W / 2 + 0.05    # line centre to hole / pad-opening edge
GROUP = "squid art"
UM = pcbgen.SP_UM             # squid sits over the MCU

b = pcbnew.LoadBoard(PCB)
if any(g.GetName() == GROUP for g in b.Groups()):
    # Clear an earlier run, then rerun this script: pcbnew's item lists go stale after Remove.
    for g in list(b.Groups()):
        if g.GetName() == GROUP:
            for it in list(g.GetItems()):
                b.Remove(it)
            b.Remove(g)
    b.Save(PCB)
    os.execv(sys.executable, [sys.executable] + sys.argv)   # start fresh on the cleaned board

holes = [(T(v.GetPosition().x), T(v.GetPosition().y), T(v.GetDrillValue()) / 2)
         for v in b.GetTracks() if v.GetClass() == "PCB_VIA"]
openings = []
for f in b.GetFootprints():
    for p in f.Pads():
        if p.GetDrillSize().x:
            holes.append((T(p.GetPosition().x), T(p.GetPosition().y), T(max(p.GetDrillSize().x, p.GetDrillSize().y)) / 2))
        if p.IsOnLayer(pcbnew.B_Mask):
            openings.append(p.GetEffectiveShape(pcbnew.B_Mask))
# Other back silkscreen (test-pad outlines and labels): keep the art off it.
silk_other = [g.GetEffectiveShape(pcbnew.B_SilkS) for f in b.GetFootprints() for g in f.GraphicalItems()
              if g.GetLayer() == pcbnew.B_SilkS]
silk_other += [d.GetEffectiveShape(pcbnew.B_SilkS) for d in b.GetDrawings() if d.GetLayer() == pcbnew.B_SilkS]
x0, y0 = pcbgen.sp_xy(0, 0)


def xy(u, v):
    return x0 + u, y0 + v


def clear(x, y):
    if not (0.6 < x - x0 < pcbgen.SP_L - 0.6 and 0.6 < y - y0 < pcbgen.SP_W - 0.6):
        return False
    if any(math.hypot(x - hx, y - hy) < hr + KEEP for hx, hy, hr in holes):
        return False
    q = pcbnew.VECTOR2I(MM(x), MM(y))
    if any(o.Collide(q, MM(KEEP)) for o in openings):
        return False
    return not any(o.Collide(q, MM(W / 2 + 0.15)) for o in silk_other)


group = pcbnew.PCB_GROUP(b)
group.SetName(GROUP)
b.Add(group)
kept = dropped = 0


def seg(a, c):
    s = pcbnew.PCB_SHAPE(b, pcbnew.SHAPE_T_SEGMENT)
    s.SetStart(pcbnew.VECTOR2I(MM(a[0]), MM(a[1])))
    s.SetEnd(pcbnew.VECTOR2I(MM(c[0]), MM(c[1])))
    s.SetLayer(pcbnew.B_SilkS)
    s.SetWidth(MM(W))
    b.Add(s)
    group.AddItem(s)


def path(pts, step=0.25):
    """Draw a polyline (board u, v), cut into <= step pieces, skipping pieces near holes."""
    global kept, dropped
    fine = []
    for (u0, v0), (u1, v1) in zip(pts, pts[1:]):
        n = max(1, int(math.hypot(u1 - u0, v1 - v0) / step))
        fine += [(u0 + (u1 - u0) * k / n, v0 + (v1 - v0) * k / n) for k in range(n)]
    fine.append(pts[-1])
    run = []
    for u, v in fine:
        p = xy(u, v)
        if clear(*p):
            run.append(p)
            continue
        dropped += 1
        if len(run) > 1:
            seg_run(run)
        run = []
    if len(run) > 1:
        seg_run(run)


def seg_run(run):
    global kept
    # merge collinear-ish points to keep the item count down
    keep = [run[0]]
    for k in range(1, len(run) - 1):
        (ax, ay), (bx, by), (cx, cy) = keep[-1], run[k], run[k + 1]
        if abs((bx - ax) * (cy - ay) - (by - ay) * (cx - ax)) > 0.0004:
            keep.append(run[k])
    keep.append(run[-1])
    for a, c in zip(keep, keep[1:]):
        seg(a, c)
        kept += 1


def bezier(p0, p1, p2, p3, n=60):
    out = []
    for k in range(n + 1):
        t = k / n
        a, bb, c, d = (1 - t) ** 3, 3 * (1 - t) ** 2 * t, 3 * (1 - t) * t * t, t ** 3
        out.append((a * p0[0] + bb * p1[0] + c * p2[0] + d * p3[0], a * p0[1] + bb * p1[1] + c * p2[1] + d * p3[1]))
    return out


def wiggle(pts, amp, waves):
    """Offset a curve sideways by a sine that fades out toward the tip."""
    out = []
    n = len(pts) - 1
    for k, (u, v) in enumerate(pts):
        a, c = pts[max(0, k - 1)], pts[min(n, k + 1)]
        tu, tv = c[0] - a[0], c[1] - a[1]
        L = math.hypot(tu, tv) or 1
        s = amp * math.sin(math.pi * 2 * waves * k / n) * (1 - 0.6 * k / n)
        out.append((u - tv / L * s, v + tu / L * s))
    return out


def circle(cu, cv, r, n=28):
    path([(cu + r * math.cos(2 * math.pi * k / n), cv + r * math.sin(2 * math.pi * k / n)) for k in range(n + 1)], 0.15)


# Mantle: a pointed teardrop above the MCU, tip toward the -y edge, with fins.
mantle = []
for k in range(0, 41):
    t = math.pi * k / 40                                  # 0..pi round the outline
    v = 1.6 + 8.0 * (1 - math.cos(t)) / 2                 # tip at v 1.6, base at v 9.6
    half = 3.4 * math.sin(t) ** 0.8 * (0.35 + 0.65 * (1 - math.cos(t)) / 2) ** 0.5
    mantle.append((v, half))
path([(UM + h, v) for v, h in mantle] + [(UM - h, v) for v, h in reversed(mantle)])
path([(UM + 1.3, 3.2), (UM + 4.2, 4.4), (UM + 2.6, 5.6)])   # fins
path([(UM - 1.3, 3.2), (UM - 4.2, 4.4), (UM - 2.6, 5.6)])
circle(UM - 1.35, 7.6, 0.75)                                # eyes
circle(UM + 1.35, 7.6, 0.75)
circle(UM - 1.15, 7.75, 0.25)
circle(UM + 1.15, 7.75, 0.25)

# Ten tentacles, one per motor driver. Each runs in its own lane: the ones
# reaching farther drivers leave from farther out on the mantle and run
# higher, so no tentacle crosses another; each curls down onto its driver.
drivers = []
for conn, odd, even, uc in pcbgen.SP_PAIRS:
    drivers += [(even, uc - 4.325), (odd, uc + 3.175)]
left = sorted((d for d in drivers if d[1] < UM), key=lambda d: -d[1])     # nearest first
right = sorted((d for d in drivers if d[1] > UM), key=lambda d: d[1])
tips = []
for side, arm in ((-1, left), (1, right)):
    n = len(arm)
    for k, (m, ud) in enumerate(arm):
        su = UM + side * (0.5 + 0.45 * k)                    # root along the mantle base
        lane = 13.6 - (13.6 - 10.4) * k / max(1, n - 1)       # nearest lowest, farthest highest
        tip = pcbgen.SP_VD - 5.3
        a0 = bezier((su, 9.4), (su, lane - 0.5), (su + side * 0.6, lane), (su + side * 1.8, lane), 16)
        run = abs(ud - su) - 4.0
        mid = [(su + side * (1.8 + run * q / 80), lane) for q in range(81)]
        mid = wiggle(mid, 0.28, max(2, run / 7))
        a2 = bezier(mid[-1], (ud - side * 0.8, lane), (ud - side * 0.2, tip - 1.6), (ud, tip), 20)
        path(a0 + mid[1:] + a2[1:])
        curl = [(ud + side * 0.5 * (1 - math.cos(q * math.pi / 8)), tip + 0.5 * math.sin(q * math.pi / 8))
                for q in range(13)]
        path(curl, 0.12)
        tips.append((m, ud + side * 1.6, tip + 0.6))

def text(s, u, v, h, bold=True):
    t = pcbnew.PCB_TEXT(b)
    t.SetText(s)
    t.SetLayer(pcbnew.B_SilkS)
    t.SetMirrored(True)
    t.SetTextSize(pcbnew.VECTOR2I(MM(h), MM(h)))
    t.SetTextThickness(MM(max(0.15, h * 0.15)))
    t.SetBold(bold)
    t.SetPosition(pcbnew.VECTOR2I(*[MM(c) for c in xy(u, v)]))
    t.SetHorizJustify(pcbnew.GR_TEXT_H_ALIGN_CENTER)
    b.Add(t)
    group.AddItem(t)
    return t


def text_clear(t):
    sh = t.GetEffectiveShape()
    if any(sh.Collide(o, MM(0.15)) for o in silk_other):
        return False
    if any(sh.Collide(pcbnew.SHAPE_CIRCLE(pcbnew.VECTOR2I(MM(hx), MM(hy)), MM(hr)), MM(0.15)) for hx, hy, hr in holes):
        return False
    return not any(sh.Collide(o, MM(0.15)) for o in openings)


# "ASH+C" with "2026" under it: the first clear spot along the -y half, near the USB end.
mark = None
for u in [x * 0.5 for x in range(30, 280)]:
    for v in (3.0, 4.0, 5.0, 6.0):
        t1 = text("ASH+C", u, v, 2.0)
        t2 = text("2026", u, v + 2.8, 1.5)
        if text_clear(t1) and text_clear(t2):
            mark = (u, v)
            break
        for t in (t1, t2):
            group.RemoveItem(t)
            b.Remove(t)
    if mark:
        break

for m, u, v in tips:                                          # motor numbers at the tips
    t = text(f"M{m}", u, v, 1.0, bold=False)
    if not text_clear(t):
        group.RemoveItem(t)
        b.Remove(t)

b.Save(PCB)
print(f"squid: {kept} segments, {dropped} short pieces dropped near holes; ASH+C / 2026 at {mark}")
