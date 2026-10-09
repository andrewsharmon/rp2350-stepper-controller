"""Write a single-sheet KiCad schematic from a part/net description.

Parts are placed on a grid and connected with net labels on every pin end;
unassigned pins get no-connect flags.
"""

import json
import os
import uuid

import kilib
from sexp import Sym, dump, find, first

GRID = 2.54
LABEL_ANGLE = {0: 180, 180: 0, 90: 270, 270: 90}
NS = uuid.UUID("8f2c0d8e-5b7a-4f1e-9d4b-3a6c2e1f0a11")
FP_LIB_TABLE = ('(fp_lib_table\n  (version 7)\n  (lib (name "stack")(type "KiCad")'
                '(uri "${KIPRJMOD}/../lib/stack.pretty")(options "")(descr "Project footprints"))\n)\n')


def snap(v):
    return round(v / GRID) * GRID


class Part:
    def __init__(self, lib_id, ref, value, footprint="", nets=None, fields=None, dnp=False, in_bom=True):
        self.lib_id, self.ref, self.value = lib_id, ref, value
        self.footprint = footprint
        self.nets = nets or {}
        self.fields = fields or {}
        self.dnp = dnp
        self.in_bom = in_bom


class Schematic:
    def __init__(self, name, title, paper="A2"):
        self.name, self.title, self.paper = name, title, paper
        self.parts = []
        self.counters = {}
        self.root = str(uuid.uuid5(NS, name))
        self._n = 0

    def add(self, lib_id, prefix, value, footprint="", nets=None, ref=None, **fields):
        dnp = fields.pop("dnp", False)
        in_bom = fields.pop("in_bom", True)
        used = {p.ref for p in self.parts}
        if ref is None:
            while ref is None or ref in used:
                self.counters[prefix] = self.counters.get(prefix, 0) + 1
                ref = f"{prefix}{self.counters[prefix]}"
        elif ref in used:
            raise ValueError(f"duplicate reference {ref}")
        p = Part(lib_id, ref, value, footprint, nets, fields, dnp, in_bom)
        self.parts.append(p)
        return p

    def _uuid(self):
        self._n += 1
        return str(uuid.uuid5(NS, f"{self.name}/{self._n}"))

    def _prop(self, name, value, x, y, hide=False):
        eff = [Sym("effects"), [Sym("font"), [Sym("size"), 1.27, 1.27]]]
        node = [Sym("property"), name, value, [Sym("at"), x, y, 0]]
        if hide:
            node.append([Sym("hide"), Sym("yes")])
        node.append(eff)
        return node

    def build(self):
        symdefs = {}
        placed = []
        # Shelf-pack parts left to right, top to bottom.
        width = {"A2": 560, "A3": 400, "A1": 800}[self.paper]
        x0, y0, cursor_x, cursor_y, row_h = 20.0, 30.0, 20.0, 30.0, 0.0
        for p in self.parts:
            if p.lib_id not in symdefs:
                symdefs[p.lib_id] = kilib.load(p.lib_id)
            pins = kilib.pins(symdefs[p.lib_id])
            xs = [q["x"] for q in pins] or [0]
            ys = [q["y"] for q in pins] or [0]
            label_w = 22.0
            w = (max(xs) - min(xs)) + 2 * label_w + 6
            h = (max(ys) - min(ys)) + 14
            if cursor_x + w > width:
                cursor_x, cursor_y, row_h = x0, cursor_y + row_h, 0.0
            sx = snap(cursor_x + label_w - min(xs))
            sy = snap(cursor_y + 8 + max(ys))
            placed.append((p, sx, sy, pins, min(ys), max(ys)))
            cursor_x += w
            row_h = max(row_h, h)

        items = []
        used_nets = {}
        for p, sx, sy, pins, ymin, ymax in placed:
            sym = [Sym("symbol"), [Sym("lib_id"), p.lib_id], [Sym("at"), sx, sy, 0], [Sym("unit"), 1],
                   [Sym("exclude_from_sim"), Sym("no")],
                   [Sym("in_bom"), Sym("yes" if p.in_bom else "no")], [Sym("on_board"), Sym("yes")],
                   [Sym("dnp"), Sym("yes" if p.dnp else "no")],
                   [Sym("uuid"), self._uuid()]]
            sym.append(self._prop("Reference", p.ref, sx, sy - ymax - 3))
            sym.append(self._prop("Value", p.value, sx, sy - ymin + 3))
            sym.append(self._prop("Footprint", p.footprint, sx, sy, hide=True))
            sym.append(self._prop("Datasheet", "", sx, sy, hide=True))
            for k, v in p.fields.items():
                sym.append(self._prop(k, v, sx, sy, hide=True))
            sym.append([Sym("instances"), [Sym("project"), self.name,
                        [Sym("path"), "/" + self.root, [Sym("reference"), p.ref], [Sym("unit"), 1]]]])
            items.append(sym)

            for q in pins:
                px, py = sx + q["x"], sy - q["y"]
                net = p.nets.get(q["number"], p.nets.get(q["name"]))
                if net is None:
                    items.append([Sym("no_connect"), [Sym("at"), px, py], [Sym("uuid"), self._uuid()]])
                    continue
                used_nets.setdefault(net, []).append(f"{p.ref}.{q['number']}")
                ang = LABEL_ANGLE[int(q["angle"]) % 360]
                just = "left" if ang in (0, 90) else "right"
                items.append([Sym("label"), net, [Sym("at"), px, py, ang],
                              [Sym("effects"), [Sym("font"), [Sym("size"), 1.27, 1.27]],
                               [Sym("justify"), Sym(just), Sym("bottom")]],
                              [Sym("uuid"), self._uuid()]])
            for key in p.nets:
                if not any(key in (q["number"], q["name"]) for q in pins):
                    raise ValueError(f"{p.ref} ({p.lib_id}) has no pin {key!r}")

        lib_symbols = [Sym("lib_symbols")] + list(symdefs.values())
        tb = [Sym("title_block"), [Sym("title"), self.title], [Sym("rev"), "0.1"],
              [Sym("comment"), 1, "Generated by hardware/gen/build.py - edit the generator, not this file"]]
        doc = [Sym("kicad_sch"), [Sym("version"), 20231120], [Sym("generator"), "eeschema"],
               [Sym("generator_version"), "8.0"], [Sym("uuid"), self.root],
               [Sym("paper"), self.paper], tb, lib_symbols] + items + \
              [[Sym("sheet_instances"), [Sym("path"), "/", [Sym("page"), "1"]]]]
        return dump(doc) + "\n", used_nets

    def write(self, out_dir):
        os.makedirs(out_dir, exist_ok=True)
        text, nets = self.build()
        with open(os.path.join(out_dir, self.name + ".kicad_sch"), "w") as f:
            f.write(text)
        with open(os.path.join(out_dir, "fp-lib-table"), "w") as f:
            f.write(FP_LIB_TABLE)
        pro = os.path.join(out_dir, self.name + ".kicad_pro")
        if not os.path.exists(pro):
            with open(pro, "w") as f:
                json.dump({"meta": {"filename": self.name + ".kicad_pro", "version": 1}}, f, indent=2)
        singles = sorted(n for n, refs in nets.items() if len(refs) == 1)
        return singles
