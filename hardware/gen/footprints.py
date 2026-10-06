"""Generate project footprints (hardware/lib/stack.pretty).

Hirose DF40 series, 0.4 mm pitch, 2.0 mm stack height. Dimensions from the
DF40 catalogue's recommended PCB layouts:
  socket DF40C(2.0)-40DS-0.4V(51) (LCSC C597934): pads 0.20 wide, rows from
         2.38 to 3.78 mm across, 1.5 mm layout-prohibited strip between rows
  header DF40C-40DP-0.4V(51) (LCSC C424643): pads 0.23 wide, rows from 2.05
         to 3.37 mm across, plus a 0.35 mm metal-fitting tab 0.30-0.65 mm
         beyond each row end
The header is the same for every stack height; the socket sets the height.

Pin 1 at -X, odd pins on one row and even on the other. The header goes on
the underside of the upper board, where KiCad's flip mirrors Y, so its odd
row is drawn at +Y: after the flip, header pin n sits over socket pin n.
The connector is not polarized; the stack is keyed by connector placement.
"""

import os

from sexp import Sym, dump

OUT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "lib", "stack.pretty")

PITCH = 0.40
PINS = 40

# kind: pad w, inner row edge, outer row edge, body length A, body width, odd row sign, MPN
VARIANTS = {
    "socket": (0.20, 2.38, 3.78, {40: 10.6}, 3.38, -1, "DF40C(2.0)-{n}DS-0.4V(51)"),
    "header": (0.23, 2.05, 3.37, {40: 9.52}, 2.97, +1, "DF40C-{n}DP-0.4V(51)"),
}
NAME = {"socket": "Hirose_DF40C-2.0-{n}DS_Socket", "header": "Hirose_DF40C-{n}DP_Header"}


def rect(layer, x1, y1, x2, y2, width):
    return [Sym("fp_rect"), [Sym("start"), x1, y1], [Sym("end"), x2, y2],
            [Sym("stroke"), [Sym("width"), width], [Sym("type"), Sym("solid")]],
            [Sym("fill"), Sym("no")], [Sym("layer"), layer]]


def text(kind, value, y, layer):
    return [Sym("property"), kind, value, [Sym("at"), 0, y, 0], [Sym("layer"), layer],
            [Sym("effects"), [Sym("font"), [Sym("size"), 0.6, 0.6], [Sym("thickness"), 0.1]]]]


def smd(num, x, y, w, h):
    return [Sym("pad"), num, Sym("smd"), Sym("rect"), [Sym("at"), x, y], [Sym("size"), w, h],
            [Sym("layers"), "F.Cu", "F.Paste", "F.Mask"]]


def keepout(x1, y1, x2, y2):
    """Copper keepout (the socket's layout-prohibited strip)."""
    return [Sym("zone"), [Sym("net"), 0], [Sym("net_name"), ""], [Sym("layers"), "F.Cu"],
            [Sym("hatch"), Sym("edge"), 0.5], [Sym("connect_pads"), [Sym("clearance"), 0]],
            [Sym("min_thickness"), 0.25],
            [Sym("keepout"), [Sym("tracks"), Sym("not_allowed")], [Sym("vias"), Sym("not_allowed")],
             [Sym("pads"), Sym("not_allowed")], [Sym("copperpour"), Sym("not_allowed")],
             [Sym("footprints"), Sym("allowed")]],
            [Sym("fill"), [Sym("thermal_gap"), 0.5], [Sym("thermal_bridge_width"), 0.5]],
            [Sym("polygon"), [Sym("pts")] + [[Sym("xy"), x, y] for x, y in
                                              ((x1, y1), (x2, y1), (x2, y2), (x1, y2))]]]


def footprint(kind, n):
    pw, inner, outer, body_len, body_w, odd, mpn = VARIANTS[kind]
    name = NAME[kind].format(n=n)
    ph = round((outer - inner) / 2, 3)
    py = round((outer + inner) / 4, 3)
    span = (n // 2 - 1) * PITCH
    x0 = -span / 2
    fp = [Sym("footprint"), name, [Sym("version"), 20240108], [Sym("generator"), "stack_gen"],
          [Sym("layer"), "F.Cu"],
          [Sym("descr"), f"Hirose {mpn.format(n=n)} board-to-board {kind}, 0.4 mm pitch, {n} pins, "
                         "2.0 mm stack. Corner contacts are metal fittings, not for signal or power."],
          [Sym("attr"), Sym("smd")],
          text("Reference", "REF**", -outer / 2 - 1.0, "F.SilkS"),
          text("Value", name, outer / 2 + 1.0, "F.Fab")]
    for i in range(n):
        x = x0 + (i // 2) * PITCH
        y = odd * py if i % 2 == 0 else -odd * py
        fp.append(smd(str(i + 1), round(x, 3), y, pw, ph))
    ext_x = span / 2 + pw / 2
    if kind == "header":
        # Metal-fitting tabs: 0.35 wide, 0.30-0.65 mm beyond the end contacts.
        tab_x = span / 2 + 0.475
        for sx in (-1, 1):
            for sy in (-1, 1):
                fp.append(smd("", round(sx * tab_x, 3), sy * py, 0.35, ph))
        ext_x = span / 2 + 0.65
    else:
        fp.append(keepout(-span / 2 - 0.2, -0.75, span / 2 + 0.2, 0.75))
    a, w = body_len[n] / 2, body_w / 2
    fp.append(rect("F.Fab", -a, -w, a, w, 0.1))
    cx = round(max(a, ext_x) + 0.25, 3)
    cy = round(max(w, outer / 2) + 0.25, 3)
    fp.append(rect("F.CrtYd", -cx, -cy, cx, cy, 0.05))
    # Pin-1 marker on silk, outside the pads at pin 1's row.
    fp.append([Sym("fp_circle"), [Sym("center"), round(x0 - 0.2, 3), round(odd * (cy + 0.2), 3)],
               [Sym("end"), round(x0 - 0.1, 3), round(odd * (cy + 0.2), 3)],
               [Sym("stroke"), [Sym("width"), 0.1], [Sym("type"), Sym("solid")]],
               [Sym("fill"), Sym("yes")], [Sym("layer"), "F.SilkS"]])
    return name, dump(fp) + "\n"


def main():
    os.makedirs(OUT, exist_ok=True)
    for kind in VARIANTS:
        name, text_ = footprint(kind, PINS)
        with open(os.path.join(OUT, name + ".kicad_mod"), "w") as f:
            f.write(text_)
        print("wrote", name)


if __name__ == "__main__":
    main()
