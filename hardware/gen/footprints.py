"""Generate project footprints (hardware/lib/stack.pretty).

XUNPU BTB0.408 series, 0.40 mm pitch, dimensions from the LCSC datasheets:
  socket BTB0.408-xxPLBDR-M41 (C42420584 for 30P), body 0.80 mm high
  header BTB0.408-xxPLBDR-G41 (C42420579 for 30P)
Odd pins on the -Y row, even pins on the +Y row, pin 1 at -X.
"""

import os

from sexp import Sym, dump

OUT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "lib", "stack.pretty")

PITCH = 0.40

# name: (signal pad w, h, row center y, hold-down w, h, y, x offset beyond pin 1, body length A, body width)
VARIANTS = {
    "socket": (0.20, 0.40, 1.20, 0.90, 0.92, 0.99, 1.00, lambda n: {30: 8.50}[n], 2.50),
    "header": (0.23, 0.65, 0.875, 0.40, 0.53, 0.565, 0.80, lambda n: {30: 7.80}[n], 2.00),
}
MPN = {"socket": "BTB0.408-{n}PLBDR-M41", "header": "BTB0.408-{n}PLBDR-G41"}


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


def footprint(kind, n):
    pw, ph, py, hw, hh, hy, hx, body_len, body_w = VARIANTS[kind]
    body_a = body_len(n)
    name = f"XUNPU_BTB0.408-{n}P_{kind.capitalize()}"
    span = (n // 2 - 1) * PITCH
    x0 = -span / 2
    fp = [Sym("footprint"), name, [Sym("version"), 20240108], [Sym("generator"), "stack_gen"],
          [Sym("layer"), "F.Cu"],
          [Sym("descr"), f"XUNPU {MPN[kind].format(n=n)} board-to-board {kind}, 0.4 mm pitch, {n} pins"],
          [Sym("attr"), Sym("smd")],
          text("Reference", "REF**", -body_w / 2 - 1.0, "F.SilkS"),
          text("Value", name, body_w / 2 + 1.0, "F.Fab")]
    for i in range(n):
        x = x0 + (i // 2) * PITCH
        y = -py if i % 2 == 0 else py
        fp.append(smd(str(i + 1), round(x, 3), y, pw, ph))
    hold_x = span / 2 + hx
    for sx in (-1, 1):
        for sy in (-1, 1):
            fp.append(smd("", round(sx * hold_x, 3), sy * hy, hw, hh))
    a, w = body_a / 2, body_w / 2
    fp.append(rect("F.Fab", -a, -w, a, w, 0.1))
    ext_x = max(a, hold_x + hw / 2) + 0.25
    ext_y = max(w, py + ph / 2, hy + hh / 2) + 0.25
    fp.append(rect("F.CrtYd", -ext_x, -ext_y, ext_x, ext_y, 0.05))
    # Pin-1 marker on silk, outside the pads.
    fp.append([Sym("fp_circle"), [Sym("center"), round(x0 - 0.2, 3), round(-ext_y - 0.2, 3)],
               [Sym("end"), round(x0 - 0.1, 3), round(-ext_y - 0.2, 3)],
               [Sym("stroke"), [Sym("width"), 0.1], [Sym("type"), Sym("solid")]],
               [Sym("fill"), Sym("yes")], [Sym("layer"), "F.SilkS"]])
    return name, dump(fp) + "\n"


def main():
    os.makedirs(OUT, exist_ok=True)
    for kind in VARIANTS:
        name, text_ = footprint(kind, 30)
        with open(os.path.join(OUT, name + ".kicad_mod"), "w") as f:
            f.write(text_)
        print("wrote", name)


if __name__ == "__main__":
    main()
