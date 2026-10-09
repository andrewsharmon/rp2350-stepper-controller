"""Draw a wired, multi-sheet KiCad schematic for parts described in build.py.

build.py owns the circuit: parts, values, fields and the net on every pin.
A layout script (see single_sh_sch.py) places those parts on sheets and draws
wires, power symbols and labels between their pins. check() then traces the
drawing the way KiCad does and fails unless every pin lands on exactly the net
build.py gave it, so the drawing can't drift from the netlist. lint() is a
rougher aid for layout edits: it estimates text and symbol extents and reports
overlaps.

Coordinates are KiCad schematic millimetres: x right, y down. Directions are
'L', 'R', 'U', 'D'.
"""

import os
import uuid

import kilib
from schgen import NS
from sexp import Sym, dump, find, first

DIRS = {"L": (-1, 0), "R": (1, 0), "U": (0, -1), "D": (0, 1)}
# Symbol rotation -> (a, b, c, d): screen = (a*x + b*y, c*x + d*y) for library (x, y), y up.
ROT = {0: (1, 0, 0, -1), 90: (0, -1, -1, 0), 180: (-1, 0, 0, 1), 270: (0, 1, 1, 0)}
FONT = 1.27
CHAR_W = 1.2            # stroke-font advance at 1.27 mm (measured); overlap estimates only
GROUND = {"GND"}
POWER_LIB = {"GND": "power:GND", "+3V3": "power:+3V3", "+1V1": "power:+1V1", "VBUS": "power:VBUS"}
DEFAULT_SUPPLY_LIB = "power:+5V"   # any other rail: an up-arrow showing the net name


def r4(v):
    return round(v + 0.0, 4) + 0.0


def pt(x, y):
    return (r4(x), r4(y))


def add(p, d, k=1.0):
    return pt(p[0] + d[0] * k, p[1] + d[1] * k)


def dir_of(v):
    for k, d in DIRS.items():
        if (round(v[0]), round(v[1])) == d:
            return k
    raise ValueError(v)


def text_w(s, size=FONT):
    s = s.replace("~{", "").replace("}", "")
    return len(s) * CHAR_W * size / FONT


class Box:
    def __init__(self, x1, y1, x2, y2, what):
        self.x1, self.x2 = min(x1, x2), max(x1, x2)
        self.y1, self.y2 = min(y1, y2), max(y1, y2)
        self.what = what

    def overlaps(self, o, gap=0.0):
        return (self.x1 < o.x2 + gap and o.x1 < self.x2 + gap and
                self.y1 < o.y2 + gap and o.y1 < self.y2 + gap)

    def hits_segment(self, a, b, shrink=0.05):
        """True if the axis-aligned segment a-b passes through the box (shrunk by `shrink`)."""
        bx1, bx2, by1, by2 = self.x1 + shrink, self.x2 - shrink, self.y1 + shrink, self.y2 - shrink
        x1, x2 = sorted((a[0], b[0]))
        y1, y2 = sorted((a[1], b[1]))
        in_x = bx1 < x1 < bx2 if x1 == x2 else (x1 < bx2 and bx1 < x2)
        in_y = by1 < y1 < by2 if y1 == y2 else (y1 < by2 and by1 < y2)
        return in_x and in_y


def text_box(x, y, s, hjust, vjust, size, what):
    """Approximate extent of horizontal text anchored at (x, y)."""
    w, h = text_w(s, size), size
    if hjust == "left":
        xa, xb = x, x + w
    elif hjust == "right":
        xa, xb = x - w, x
    else:
        xa, xb = x - w / 2, x + w / 2
    if vjust == "bottom":
        ya, yb = y - h, y
    elif vjust == "top":
        ya, yb = y, y + h
    else:
        ya, yb = y - h / 2, y + h / 2
    return Box(xa, ya, xb, yb, what)


class Pin:
    def __init__(self, number, name, at, out, net, kind):
        self.number, self.name, self.at, self.out, self.net, self.kind = number, name, at, out, net, kind


class Placed:
    """A part placed on a sheet; index it by pin number or name for the pin end."""

    def __init__(self, sheet, part, lib_id, x, y, rot, symdef, mirror=None):
        self.sheet, self.part, self.lib_id = sheet, part, lib_id
        self.ref = part.ref
        self.at, self.rot, self.mirror = pt(x, y), rot, mirror
        self.symdef = symdef
        self.pins = []
        for q in kilib.pins(symdef):
            ang = int(q["angle"]) % 360
            ix, iy = {0: (1, 0), 90: (0, 1), 180: (-1, 0), 270: (0, -1)}[ang]
            dx, dy = self.xf(q["x"], q["y"])
            at = pt(x + dx, y + dy)
            ox, oy = self.xf(ix, iy)
            out = (-ox, -oy)
            net = part.nets.get(q["number"], part.nets.get(q["name"]))
            self.pins.append(Pin(q["number"], q["name"], at, out, net, q["type"]))

    def xf(self, lx, ly):
        """Library offset -> screen offset (rotation, then mirror)."""
        a, b, c, d = ROT[self.rot]
        dx, dy = a * lx + b * ly, c * lx + d * ly
        if self.mirror == "x":
            dy = -dy
        elif self.mirror == "y":
            dx = -dx
        return dx + 0.0, dy + 0.0

    def pin(self, key):
        hits = [p for p in self.pins if p.number == key] or [p for p in self.pins if p.name == key]
        if not hits:
            raise KeyError(f"{self.ref} has no pin {key!r}")
        if len({p.at for p in hits}) > 1:
            raise KeyError(f"{self.ref} pin {key!r} is at several places; use a pin number")
        return hits[0]

    def __getitem__(self, key):
        return self.pin(key).at

    def out(self, key):
        return self.pin(key).out

    def net(self, key):
        return self.pin(key).net

    def body(self):
        """Bounding box of the symbol graphics (no pins), screen coordinates."""
        xs, ys = [], []
        for unit in find(self.symdef, "symbol"):
            for g in unit:
                if not isinstance(g, list):
                    continue
                pts = []
                if g[0] == "rectangle":
                    pts = [first(g, "start")[1:3], first(g, "end")[1:3]]
                elif g[0] in ("polyline", "bezier"):
                    pts = [p[1:3] for p in find(first(g, "pts"), "xy")]
                elif g[0] == "circle":
                    cx, cy = (float(v) for v in first(g, "center")[1:3])
                    r = float(first(g, "radius")[1])
                    pts = [(cx - r, cy - r), (cx + r, cy + r)]
                elif g[0] == "arc":
                    pts = [first(g, k)[1:3] for k in ("start", "mid", "end")]
                for px, py in pts:
                    xs.append(float(px))
                    ys.append(float(py))
        if not xs:
            return None
        corners = [(self.at[0] + self.xf(px, py)[0], self.at[1] + self.xf(px, py)[1])
                   for px in (min(xs), max(xs)) for py in (min(ys), max(ys))]
        return Box(min(c[0] for c in corners), min(c[1] for c in corners),
                   max(c[0] for c in corners), max(c[1] for c in corners), f"{self.ref} body")


class Sheet:
    def __init__(self, design, name, file, title, paper="A3"):
        self.design, self.name, self.file, self.title, self.paper = design, name, file, title, paper
        self.uuid = design.uuid(f"sheet/{name}")
        self.syms, self.wires, self.labels, self.powers, self.ncs, self.graphics = [], [], [], [], [], []
        self.junctions = []     # filled in by Design.check()
        self.sheet_symbols = []
        self.frames = []
        self.text_boxes = []    # estimated extents, for lint()
        self._n = 0

    def _uuid(self):
        self._n += 1
        return self.design.uuid(f"{self.file}/{self._n}")

    # -- parts -------------------------------------------------------------

    def place(self, ref, x, y, rot=0, ref_at=None, val_at=None, lib_id=None, hide_value=False, mirror=None):
        """Place part `ref` with its origin at (x, y).

        ref_at / val_at: (dx, dy, justify) relative to the origin; justify is
        'left', 'right' or 'center'. Defaults suit the symbol's orientation.
        """
        part = self.design.take(ref)
        lib_id = lib_id or part.lib_id
        p = Placed(self, part, lib_id, x, y, rot, self.design.symdef(lib_id), mirror)
        p.ref_at, p.val_at, p.hide_value = ref_at, val_at, hide_value
        self.syms.append(p)
        return p

    def two_pin(self, ref, a, direction="D", side="R", lib_id=None, **kw):
        """Place a two-pin part with pin 1 at point `a`, pin 2 toward `direction`.

        `side` puts the reference/value text to the right ('R') or left ('L')
        of a vertical part. A horizontal part gets its reference above and its
        value below, or both below with side='D'.
        """
        part = self.design.parts[ref]
        symdef = self.design.symdef(lib_id or part.lib_id)
        q = {p["number"]: p for p in kilib.pins(symdef)}
        p1, p2 = q["1"], q["2"]
        half = abs(p1["x"] - p2["x"]) / 2 + abs(p1["y"] - p2["y"]) / 2
        # Library pin 1 is at +y (top) for R/C/L/F, at -x (left) for D/SW/JP.
        lib_vertical = p1["x"] == p2["x"]
        lib_dir = ("U" if p1["y"] > p2["y"] else "D") if lib_vertical else ("L" if p1["x"] < p2["x"] else "R")
        # Direction from pin 1 to pin 2 in the library frame, screen terms.
        lib_12 = {"U": "D", "D": "U", "L": "R", "R": "L"}[lib_dir]
        order = ["R", "U", "L", "D"]   # screen directions, counter-clockwise (KiCad rotation sense)
        rot = ((order.index(direction) - order.index(lib_12)) % 4) * 90
        d = DIRS[direction]
        c = add(a, d, half)
        if "ref_at" not in kw and "val_at" not in kw:
            if direction in "UD":
                off = 2.54 if side == "R" else -2.54
                j = "left" if side == "R" else "right"
                kw["ref_at"] = (off, -1.27, j)
                kw["val_at"] = (off, 1.27, j)
            else:
                if side == "D":
                    kw["ref_at"] = (0, 2.54, "center")
                    kw["val_at"] = (0, 5.08, "center")
                else:
                    kw["ref_at"] = (0, -2.54 if lib_vertical else -3.81, "center")
                    kw["val_at"] = (0, 2.54 if lib_vertical else 3.81, "center")
        p = self.place(ref, c[0], c[1], rot, lib_id=lib_id, **kw)
        assert p["1"] == pt(*a), (ref, p["1"], a)
        return p

    # -- drawing -----------------------------------------------------------

    def wire(self, *pts):
        """Polyline through points; every leg must be horizontal or vertical."""
        pts = [pt(*q) for q in pts]
        for a, b in zip(pts, pts[1:]):
            if a == b:
                continue
            if a[0] != b[0] and a[1] != b[1]:
                raise ValueError(f"{self.name}: diagonal wire {a} -> {b}")
            self.wires.append((a, b))
        return pts[-1]

    def hv(self, a, b):
        """Wire a -> b, horizontal first."""
        return self.wire(a, (b[0], a[1]), b)

    def vh(self, a, b):
        """Wire a -> b, vertical first."""
        return self.wire(a, (a[0], b[1]), b)

    def stub(self, a, d, length=2.54):
        """Wire from point a in direction d; returns the far end."""
        b = add(a, DIRS[d] if isinstance(d, str) else d, length)
        self.wire(a, b)
        return b

    def label(self, text, at, d, kind="local", shape="passive"):
        """Net label anchored at `at`, text running in direction d."""
        self.labels.append({"kind": kind, "text": text, "at": pt(*at), "dir": d, "shape": shape})

    def glabel(self, text, at, d, shape="passive"):
        self.label(text, at, d, "global", shape)

    def pin_label(self, sym, key, kind="local", length=2.54, shape="passive", text=None):
        """Stub out of a pin and label it with the pin's net."""
        p = sym.pin(key)
        end = self.stub(p.at, p.out, length) if length else p.at
        self.label(text or p.net, end, dir_of(p.out), kind, shape)
        return end

    def power(self, net, at, d=None):
        """Power symbol whose pin is at `at`. GND hangs down; supplies point up.

        d overrides the direction the symbol points ('U', 'D', 'L', 'R').
        """
        if d is None:
            d = "D" if net in GROUND else "U"
        rot = {"U": 0, "L": 90, "D": 180, "R": 270}[d]
        if net in GROUND:
            rot = (rot + 180) % 360
        lib = POWER_LIB.get(net, DEFAULT_SUPPLY_LIB)
        self.powers.append({"net": net, "lib": lib, "at": pt(*at), "rot": rot})

    def pin_power(self, sym, key, length=0, d=None):
        """Power symbol on a pin, optionally after a stub."""
        p = sym.pin(key)
        end = self.stub(p.at, p.out, length) if length else p.at
        self.power(p.net, end, d)
        return end

    def flag(self, at, d="U"):
        """PWR_FLAG with its pin at `at`."""
        rot = {"U": 0, "L": 90, "D": 180, "R": 270}[d]
        self.powers.append({"net": None, "lib": "power:PWR_FLAG", "at": pt(*at), "rot": rot})

    def nc(self, sym, key):
        self.ncs.append(sym[key])

    def text(self, x, y, s, size=FONT, bold=False, justify="left", italic=False):
        eff = [Sym("effects"), [Sym("font"), [Sym("size"), size, size]] +
               ([[Sym("bold"), Sym("yes")]] if bold else []) + ([[Sym("italic"), Sym("yes")]] if italic else []),
               [Sym("justify"), Sym(justify), Sym("top")]]
        self.graphics.append([Sym("text"), s, [Sym("exclude_from_sim"), Sym("no")],
                              [Sym("at"), r4(x), r4(y), 0], eff, [Sym("uuid"), self._uuid()]])
        lines = s.split("\n")
        w = max(text_w(line, size) for line in lines)
        x0 = x if justify == "left" else x - w if justify == "right" else x - w / 2
        self.text_boxes.append(Box(x0, y, x0 + w, y + len(lines) * size * 1.6, f"text {lines[0][:20]!r}"))

    def frame(self, x1, y1, x2, y2, title=None):
        """Dashed box around a functional block, title at its top-left."""
        self.graphics.append([Sym("rectangle"), [Sym("start"), r4(x1), r4(y1)], [Sym("end"), r4(x2), r4(y2)],
                              [Sym("stroke"), [Sym("width"), 0], [Sym("type"), Sym("dash")]],
                              [Sym("fill"), [Sym("type"), Sym("none")]], [Sym("uuid"), self._uuid()]])
        self.frames.append((x1, y1, x2, y2))
        if title:
            self.text(x1 + 2.54, y1 + 2.54, title, size=1.8, bold=True)

    def line(self, *pts, dash=False, width=0):
        """Graphic polyline (not a wire)."""
        self.graphics.append([Sym("polyline"),
                              [Sym("pts")] + [[Sym("xy"), r4(x), r4(y)] for x, y in pts],
                              [Sym("stroke"), [Sym("width"), width], [Sym("type"), Sym("dash" if dash else "solid")]],
                              [Sym("uuid"), self._uuid()]])

    def arrow(self, a, b, width=0.3):
        """Graphic arrow from a to b (horizontal or vertical), not a wire."""
        self.line(a, b, width=width)
        d = dir_of(((b[0] > a[0]) - (b[0] < a[0]), (b[1] > a[1]) - (b[1] < a[1])))
        dx, dy = DIRS[d]
        px, py = -dy, dx
        back = (b[0] - dx * 2.0, b[1] - dy * 2.0)
        self.line((back[0] + px, back[1] + py), b, (back[0] - px, back[1] - py), width=width)

    def sheet_symbol(self, child, x, y, w, h):
        """Hierarchical sheet symbol for `child` (a Sheet), on this (root) sheet."""
        self.sheet_symbols.append((child, pt(x, y), (w, h)))


class Design:
    def __init__(self, name, title, parts, rev="0.1"):
        self.name, self.title, self.rev = name, title, rev
        self.parts = {p.ref: p for p in parts}
        self.placed = set()
        self.root = Sheet(self, "root", name + ".kicad_sch", title)
        self.root.uuid = str(uuid.uuid5(NS, name))   # as schgen, so the project's root sheet keeps its UUID
        self.sheets = []
        self._symdefs = {}
        self.extra_symdefs = {}

    def uuid(self, key):
        return str(uuid.uuid5(NS, f"{self.name}/{key}"))

    def symdef(self, lib_id):
        if lib_id not in self._symdefs:
            self._symdefs[lib_id] = self.extra_symdefs[lib_id] if lib_id in self.extra_symdefs else kilib.load(lib_id)
        return self._symdefs[lib_id]

    def sheet(self, name, file, title, paper="A3"):
        s = Sheet(self, name, file, title, paper)
        self.sheets.append(s)
        return s

    def take(self, ref):
        if ref not in self.parts:
            raise KeyError(f"no part {ref}")
        if ref in self.placed:
            raise ValueError(f"{ref} placed twice")
        self.placed.add(ref)
        return self.parts[ref]

    # -- connectivity --------------------------------------------------------

    def check(self, skip_prefix="#"):
        """Trace the drawing as KiCad would; return a list of mismatches with the parts' nets.

        Parts whose reference starts with `skip_prefix` (build.py's PWR_FLAGs)
        need not be placed; the layout adds its own flags.
        """
        errors = []
        missing = sorted(r for r in self.parts if r not in self.placed and not r.startswith(skip_prefix))
        if missing:
            errors.append(f"not placed: {', '.join(missing)}")

        parent = {}

        def find_(k):
            parent.setdefault(k, k)
            while parent[k] != k:
                parent[k] = parent[parent[k]]
                k = parent[k]
            return k

        def union(a, b):
            ra, rb = find_(a), find_(b)
            if ra != rb:
                parent[ra] = rb

        pin_nodes = []   # (node, ref, number, net)
        names = {}       # node -> label and power-symbol names on it
        for sh in [self.root] + self.sheets:
            S_ = sh.name
            # Points where something connects (other than wire interiors).
            hard = {}
            for s in sh.syms:
                for p in s.pins:
                    hard.setdefault(p.at, []).append(("pin", s.ref, p.number, p.net))
            for lb in sh.labels:
                hard.setdefault(lb["at"], []).append(("label", lb["text"]))
            for pw in sh.powers:
                hard.setdefault(pw["at"], []).append(("power", pw["net"]))
            for q in sh.ncs:
                hard.setdefault(q, []).append(("nc",))
            ends = {}
            for a, b in sh.wires:
                ends.setdefault(a, []).append((a, b))
                ends.setdefault(b, []).append((a, b))
            # Split wires at interior points that touch anything.
            touch = set(hard) | set(ends)
            segs = []
            for a, b in sh.wires:
                inner = sorted(q for q in touch if q not in (a, b) and _on_segment(q, a, b))
                chain = [a] + sorted(inner, key=lambda q: abs(q[0] - a[0]) + abs(q[1] - a[1])) + [b]
                segs += list(zip(chain, chain[1:]))
            # Overlapping collinear segments would merge in KiCad; flag them.
            for i, (a, b) in enumerate(segs):
                for c, d in segs[i + 1:]:
                    if _overlap(a, b, c, d):
                        errors.append(f"{S_}: overlapping wires {a}-{b} and {c}-{d}")
            for a, b in segs:
                union((S_, a), (S_, b))
            count = {}
            for a, b in segs:
                count[a] = count.get(a, 0) + 1
                count[b] = count.get(b, 0) + 1
            for q, items in hard.items():
                # Stacked pins of one symbol count once.
                count[q] = count.get(q, 0) + len({it[1] for it in items if it[0] == "pin"}) + \
                    sum(1 for it in items if it[0] == "power")
            # Junction dots where three or more things meet.
            sh.junctions = sorted(q for q, n in count.items() if n >= 3)
            for q, n in count.items():
                if n == 1 and q not in hard:
                    errors.append(f"{S_}: dangling wire end at {q}")
            for q, items in hard.items():
                node = (S_, q)
                find_(node)
                kinds = [it[0] for it in items]
                if "nc" in kinds and (kinds.count("pin") != 1 or len(kinds) != 2 or q in ends):
                    errors.append(f"{S_}: no-connect at {q} touches {items}")
                for it in items:
                    if it[0] == "pin":
                        pin_nodes.append((node,) + it[1:])
                    elif it[0] == "label":
                        names.setdefault(node, set()).add(it[1])
                    elif it[0] == "power" and it[1]:
                        names.setdefault(node, set()).add(it[1])
            for lb in sh.labels:
                key = ("G", lb["text"]) if lb["kind"] == "global" else (S_, "label", lb["text"])
                union((S_, lb["at"]), key)
            pin_points = {q.at for s in sh.syms for q in s.pins}
            seg_points = {q for seg in segs for q in seg}
            for pw in sh.powers:
                if pw["net"]:
                    union((S_, pw["at"]), ("G", pw["net"]))
                if pw["at"] not in pin_points and pw["at"] not in seg_points:
                    errors.append(f"{S_}: {pw['net'] or 'PWR_FLAG'} symbol at {pw['at']} touches no wire or pin")
            for lb in sh.labels:
                if lb["at"] not in pin_points and lb["at"] not in seg_points:
                    errors.append(f"{S_}: label {lb['text']} at {lb['at']} touches no wire or pin")

        groups = {}
        for node, ref, num, net in pin_nodes:
            groups.setdefault(find_(node), set()).add((ref, num, net))
        label_names = {}
        for node, ns in names.items():
            label_names.setdefault(find_(node), set()).update(ns)
        for g in parent:
            if g[0] == "G":
                label_names.setdefault(find_(g), set()).add(g[1])
        net_group = {}
        for g, members in groups.items():
            nets = {m[2] for m in members}
            if None in nets:
                bad = [m for m in members if m[2] is None]
                if len(members) > 1 or g in label_names:
                    errors.append(f"unused pin(s) {bad} connected to {sorted(members - set(bad))}")
                nets.discard(None)
            if len(nets) > 1:
                errors.append(f"short: {sorted(nets)} joined ({sorted(members)[:6]}...)")
            for n in nets:
                net_group.setdefault(n, set()).add(g)
            ln = label_names.get(g, set())
            if ln and nets and ln != nets:
                errors.append(f"labels {sorted(ln)} on net {sorted(nets)}")
            if len(members) == 1 and nets and not ln:
                m = next(iter(members))
                errors.append(f"unconnected pin {m[0]}.{m[1]} (net {m[2]})")
        for n, gs in net_group.items():
            if len(gs) > 1:
                parts_ = [sorted(f"{m[0]}.{m[1]}" for m in groups[g]) for g in gs]
                errors.append(f"net {n} split into {len(gs)} pieces: {parts_}")
        # Unconnected pins with no net must carry a no-connect flag.
        for sh in [self.root] + self.sheets:
            ncs = set(sh.ncs)
            for s in sh.syms:
                for p in s.pins:
                    if p.net is None and p.at not in ncs:
                        g = find_((sh.name, p.at))
                        if len(groups.get(g, ())) <= 1 and g not in label_names:
                            errors.append(f"{sh.name}: {s.ref}.{p.number} ({p.name}) needs a no-connect")
        return errors

    def lint(self):
        """Approximate visual overlap checks (bodies, labels, field text, wires)."""
        out = []
        for sh in [self.root] + self.sheets:
            boxes = []
            for s in sh.syms:
                b = s.body()
                if b:
                    boxes.append(("body", b, s.ref))
            for s in sh.syms:
                for kind, tb in self._field_boxes(s):
                    boxes.append((kind, tb, s.ref))
            for lb in sh.labels:
                boxes.append(("label", _label_box(lb), lb["text"]))
            for pw in sh.powers:
                boxes.append(("power", _power_box(pw), pw["net"] or "PWR_FLAG"))
            for tb in sh.text_boxes:
                boxes.append(("text", tb, ""))
            for i, (k1, b1, o1) in enumerate(boxes):
                for k2, b2, o2 in boxes[i + 1:]:
                    if k1 == "body" and k2 in ("ref", "value") and o1 == o2:
                        continue
                    if k1 in ("ref", "value") and k2 in ("ref", "value") and o1 == o2:
                        continue
                    if b1.overlaps(b2, gap=-0.1):
                        out.append(f"{sh.name}: {b1.what} overlaps {b2.what}")
            for fx1, fy1, fx2, fy2 in sh.frames:
                edges = [((fx1, fy1), (fx2, fy1)), ((fx1, fy2), (fx2, fy2)), ((fx1, fy1), (fx1, fy2)), ((fx2, fy1), (fx2, fy2))]
                for k, bx, o in boxes:
                    if any(bx.hits_segment(a, b, shrink=-0.3) for a, b in edges):
                        out.append(f"{sh.name}: frame edge at ({fx1},{fy1})-({fx2},{fy2}) cuts {bx.what}")
                for a, b in sh.wires:
                    for e1, e2 in edges:
                        if _crosses(a, b, e1, e2):
                            out.append(f"{sh.name}: wire {a}-{b} crosses frame ({fx1},{fy1})-({fx2},{fy2})")
            for a, b in sh.wires:
                for k, bx, o in boxes:
                    if k == "power":
                        continue
                    if bx.hits_segment(a, b):
                        out.append(f"{sh.name}: wire {a}-{b} crosses {bx.what}")
        return out

    def _field_boxes(self, s):
        for kind, txt, at in (("ref", s.ref, s.ref_at), ("value", s.part.value, s.val_at)):
            if kind == "value" and s.hide_value:
                continue
            x, y, j = self._field_pos(s, kind, at)
            yield kind, text_box(x, y, txt, j, "center", FONT, f"{s.ref} {kind}")

    def _field_pos(self, s, kind, at):
        if at is not None:
            dx, dy, j = at
            return s.at[0] + dx, s.at[1] + dy, j
        # Library position, rotated with the symbol.
        prop = next(p for p in find(s.symdef, "property") if p[1] == ("Reference" if kind == "ref" else "Value"))
        lx, ly = float(first(prop, "at")[1]), float(first(prop, "at")[2])
        dx, dy = s.xf(lx, ly)
        eff = first(prop, "effects")
        js = first(eff, "justify") if eff else None
        j = "center"
        if js and "left" in js[1:]:
            j = "left"
        elif js and "right" in js[1:]:
            j = "right"
        return s.at[0] + dx, s.at[1] + dy, j

    # -- output ----------------------------------------------------------------

    def write(self, out_dir, strict=True):
        """Write every sheet; refuses (unless strict=False) if check() finds a mismatch."""
        errors = self.check()
        if errors and strict:
            raise SystemExit("schematic check failed:\n  " + "\n  ".join(errors))
        os.makedirs(out_dir, exist_ok=True)
        refs = _PowerRefs()
        for sh in [self.root] + self.sheets:
            path = "/" + self.root.uuid + ("" if sh is self.root else "/" + sh.uuid)
            text = self._sheet_text(sh, path, refs)
            with open(os.path.join(out_dir, sh.file), "w") as f:
                f.write(text)

    def _sheet_text(self, sh, path, refs):
        items = []
        used = {}
        for s in sh.syms:
            used[s.lib_id] = self.symdef(s.lib_id)
        for pw in sh.powers:
            used[pw["lib"]] = self.symdef(pw["lib"])

        for s in sh.syms:
            items.append(self._symbol(s, path))
        for pw in sh.powers:
            items.append(self._power(sh, pw, path, refs))
        for a, b in sh.wires:
            items.append([Sym("wire"), [Sym("pts"), [Sym("xy"), a[0], a[1]], [Sym("xy"), b[0], b[1]]],
                          [Sym("stroke"), [Sym("width"), 0], [Sym("type"), Sym("default")]],
                          [Sym("uuid"), sh._uuid()]])
        for q in sh.junctions:
            items.append([Sym("junction"), [Sym("at"), q[0], q[1]], [Sym("diameter"), 0],
                          [Sym("color"), 0, 0, 0, 0], [Sym("uuid"), sh._uuid()]])
        for q in sh.ncs:
            items.append([Sym("no_connect"), [Sym("at"), q[0], q[1]], [Sym("uuid"), sh._uuid()]])
        for lb in sh.labels:
            items.append(self._label(sh, lb))
        items += sh.graphics
        for child, at, (w, h) in sh.sheet_symbols:
            items.append(self._sheet_node(sh, child, at, w, h))

        tb = [Sym("title_block"), [Sym("title"), sh.title], [Sym("rev"), self.rev],
              [Sym("company"), "RP2350 stepper controller"],
              [Sym("comment"), 1, "Generated by hardware/gen/build.py - edit the generator, not this file"]]
        lib_symbols = [Sym("lib_symbols")] + [used[k] for k in sorted(used)]
        doc = [Sym("kicad_sch"), [Sym("version"), 20231120], [Sym("generator"), "eeschema"],
               [Sym("generator_version"), "8.0"], [Sym("uuid"), sh.uuid if sh is not self.root else self.root.uuid],
               [Sym("paper"), sh.paper], tb, lib_symbols] + items
        if sh is self.root:
            doc.append([Sym("sheet_instances"), [Sym("path"), "/", [Sym("page"), "1"]]])
        return dump(doc) + "\n"

    def _prop(self, name, value, x, y, angle=0, hide=False, justify=None):
        eff = [Sym("effects"), [Sym("font"), [Sym("size"), FONT, FONT]]]
        if justify and justify != "center":
            eff.append([Sym("justify")] + [Sym(j) for j in justify.split()])
        node = [Sym("property"), name, value, [Sym("at"), r4(x), r4(y), angle]]
        if hide:
            node.append([Sym("hide"), Sym("yes")])
        node.append(eff)
        return node

    def _symbol(self, s, path):
        p = s.part
        sym = [Sym("symbol"), [Sym("lib_id"), s.lib_id], [Sym("at"), s.at[0], s.at[1], s.rot]] + \
              ([[Sym("mirror"), Sym(s.mirror)]] if s.mirror else []) + [[Sym("unit"), 1],
               [Sym("exclude_from_sim"), Sym("no")],
               [Sym("in_bom"), Sym("yes" if p.in_bom else "no")], [Sym("on_board"), Sym("yes")],
               [Sym("dnp"), Sym("yes" if p.dnp else "no")],
               [Sym("uuid"), self.uuid(f"sym/{p.ref}")]]
        # Field text angles are stored relative to the symbol: KiCad turns them
        # 90 degrees for a 90/270 symbol, so store 90 there to read horizontally.
        fang = 90 if s.rot in (90, 270) else 0
        for kind, name, value, at in (("ref", "Reference", p.ref, s.ref_at), ("value", "Value", p.value, s.val_at)):
            x, y, j = self._field_pos(s, kind, at)
            if (s.rot or s.mirror) and j != "center":
                # KiCad re-interprets justification in a turned or mirrored
                # symbol; centre the text on where it should sit instead.
                w = text_w(value)
                x, j = (x + w / 2, "center") if j == "left" else (x - w / 2, "center")
            sym.append(self._prop(name, value, x, y, fang, hide=(kind == "value" and s.hide_value), justify=j))
        sym.append(self._prop("Footprint", p.footprint, s.at[0], s.at[1], fang, hide=True))
        sym.append(self._prop("Datasheet", "", s.at[0], s.at[1], fang, hide=True))
        for k, v in p.fields.items():
            sym.append(self._prop(k, v, s.at[0], s.at[1], fang, hide=True))
        for q in s.pins:
            sym.append([Sym("pin"), q.number, [Sym("uuid"), self.uuid(f"pin/{p.ref}/{q.number}")]])
        sym.append([Sym("instances"), [Sym("project"), self.name,
                    [Sym("path"), path, [Sym("reference"), p.ref], [Sym("unit"), 1]]]])
        return sym

    def _power(self, sh, pw, path, refs):
        flag = pw["net"] is None
        ref = refs.next("#FLG" if flag else "#PWR")
        value = "PWR_FLAG" if flag else pw["net"]
        x, y = pw["at"]
        rot = pw["rot"]
        vx, vy = _power_value_at(pw, value)
        fang = 90 if rot in (90, 270) else 0
        return [Sym("symbol"), [Sym("lib_id"), pw["lib"]], [Sym("at"), x, y, rot], [Sym("unit"), 1],
                [Sym("exclude_from_sim"), Sym("no")], [Sym("in_bom"), Sym("no")], [Sym("on_board"), Sym("yes")],
                [Sym("dnp"), Sym("no")], [Sym("uuid"), sh._uuid()],
                self._prop("Reference", ref, x, y, fang, hide=True),
                self._prop("Value", value, vx, vy, fang),
                self._prop("Footprint", "", x, y, fang, hide=True),
                self._prop("Datasheet", "", x, y, fang, hide=True),
                [Sym("pin"), "1", [Sym("uuid"), sh._uuid()]],
                [Sym("instances"), [Sym("project"), self.name,
                 [Sym("path"), path, [Sym("reference"), ref], [Sym("unit"), 1]]]]]

    def _label(self, sh, lb):
        ang = {"R": 0, "U": 90, "L": 180, "D": 270}[lb["dir"]]
        just = "left" if lb["dir"] in "RU" else "right"
        if lb["kind"] == "global":
            node = [Sym("global_label"), lb["text"], [Sym("shape"), Sym(lb["shape"])],
                    [Sym("at"), lb["at"][0], lb["at"][1], ang], [Sym("fields_autoplaced"), Sym("yes")],
                    [Sym("effects"), [Sym("font"), [Sym("size"), FONT, FONT]], [Sym("justify"), Sym(just)]],
                    [Sym("uuid"), sh._uuid()],
                    [Sym("property"), "Intersheetrefs", "${INTERSHEET_REFS}",
                     [Sym("at"), lb["at"][0], lb["at"][1], 0],
                     [Sym("effects"), [Sym("font"), [Sym("size"), FONT, FONT]], [Sym("hide"), Sym("yes")]]]]
            return node
        return [Sym("label"), lb["text"], [Sym("at"), lb["at"][0], lb["at"][1], ang],
                [Sym("fields_autoplaced"), Sym("yes")],
                [Sym("effects"), [Sym("font"), [Sym("size"), FONT, FONT]], [Sym("justify"), Sym(just), Sym("bottom")]],
                [Sym("uuid"), sh._uuid()]]

    def _sheet_node(self, sh, child, at, w, h):
        x, y = at
        page = str(self.sheets.index(child) + 2)
        return [Sym("sheet"), [Sym("at"), x, y], [Sym("size"), w, h],
                [Sym("exclude_from_sim"), Sym("no")], [Sym("in_bom"), Sym("yes")], [Sym("on_board"), Sym("yes")],
                [Sym("dnp"), Sym("no")], [Sym("fields_autoplaced"), Sym("yes")],
                [Sym("stroke"), [Sym("width"), 0.1524], [Sym("type"), Sym("solid")]],
                [Sym("fill"), [Sym("color"), 0, 0, 0, 0.0]],
                [Sym("uuid"), child.uuid],
                self._prop("Sheetname", child.name, x, y - 0.7116, justify="left bottom"),
                self._prop("Sheetfile", child.file, x, y + h + 0.5846, justify="left top"),
                [Sym("instances"), [Sym("project"), self.name,
                 [Sym("path"), "/" + self.root.uuid, [Sym("page"), page]]]]]


class _PowerRefs:
    def __init__(self):
        self.n = {}

    def next(self, prefix):
        self.n[prefix] = self.n.get(prefix, 0) + 1
        return f"{prefix}{self.n[prefix]:02d}"


def _power_dir(pw):
    """Direction a power symbol points, away from its pin."""
    d = {0: "U", 90: "L", 180: "D", 270: "R"}[pw["rot"]]
    if pw["lib"] == "power:GND":
        d = {"U": "D", "D": "U", "L": "R", "R": "L"}[d]
    return d


def _power_value_at(pw, value):
    """Centre of a power symbol's value text, just beyond the symbol."""
    x, y = pw["at"]
    d = _power_dir(pw)
    reach = 3.81 if pw["lib"] in ("power:GND", "power:PWR_FLAG") else 3.556
    if d in "UD":
        return x, y + (reach if d == "D" else -reach)
    w = text_w(value)
    off = 3.3 + w / 2
    return (x - off if d == "L" else x + off), y


def _crosses(a, b, c, d):
    """True if an axis-aligned segment a-b meets segment c-d anywhere."""
    ax1, ax2 = sorted((a[0], b[0]))
    ay1, ay2 = sorted((a[1], b[1]))
    cx1, cx2 = sorted((c[0], d[0]))
    cy1, cy2 = sorted((c[1], d[1]))
    return ax1 <= cx2 and cx1 <= ax2 and ay1 <= cy2 and cy1 <= ay2


def _on_segment(q, a, b):
    if a[0] == b[0] == q[0]:
        return min(a[1], b[1]) < q[1] < max(a[1], b[1])
    if a[1] == b[1] == q[1]:
        return min(a[0], b[0]) < q[0] < max(a[0], b[0])
    return False


def _overlap(a, b, c, d):
    if a[0] == b[0] == c[0] == d[0]:
        lo1, hi1 = sorted((a[1], b[1]))
        lo2, hi2 = sorted((c[1], d[1]))
        return min(hi1, hi2) - max(lo1, lo2) > 1e-6
    if a[1] == b[1] == c[1] == d[1]:
        lo1, hi1 = sorted((a[0], b[0]))
        lo2, hi2 = sorted((c[0], d[0]))
        return min(hi1, hi2) - max(lo1, lo2) > 1e-6
    return False


def _label_box(lb):
    x, y = lb["at"]
    w = text_w(lb["text"]) + (2.0 * FONT if lb["kind"] == "global" else 0.6)
    h = FONT * (1.8 if lb["kind"] == "global" else 1.2)
    d = lb["dir"]
    if lb["kind"] == "global":
        if d == "R":
            return Box(x, y - h / 2, x + w, y + h / 2, f"glabel {lb['text']}")
        if d == "L":
            return Box(x - w, y - h / 2, x, y + h / 2, f"glabel {lb['text']}")
        if d == "U":
            return Box(x - h / 2, y - w, x + h / 2, y, f"glabel {lb['text']}")
        return Box(x - h / 2, y, x + h / 2, y + w, f"glabel {lb['text']}")
    if d == "R":
        return Box(x + 0.3, y - h, x + w, y - 0.1, f"label {lb['text']}")
    if d == "L":
        return Box(x - w, y - h, x - 0.3, y - 0.1, f"label {lb['text']}")
    if d == "U":
        return Box(x - h, y - w, x - 0.1, y - 0.3, f"label {lb['text']}")
    return Box(x - h, y + 0.3, x - 0.1, y + w, f"label {lb['text']}")


def _power_box(pw):
    x, y = pw["at"]
    name = pw["net"] or "PWR_FLAG"
    w = max(2.6, text_w(name))
    d = _power_dir(pw)
    if d == "U":
        return Box(x - w / 2, y - 4.4, x + w / 2, y - 0.3, f"power {name}")
    if d == "D":
        return Box(x - w / 2, y + 0.3, x + w / 2, y + 4.6, f"power {name}")
    if d == "L":
        return Box(x - 3.3 - w, y - 1.3, x - 0.3, y + 1.3, f"power {name}")
    return Box(x + 0.3, y - 1.3, x + 3.3 + w, y + 1.3, f"power {name}")
