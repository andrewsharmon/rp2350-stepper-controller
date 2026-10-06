"""Load KiCad library symbols (resolving `extends`) and read their pins."""

import copy
import os

from sexp import Sym, find, first, parse

SYMBOL_DIR = os.environ.get(
    "KICAD_SYMBOL_DIR",
    "/Applications/KiCad/KiCad.app/Contents/SharedSupport/symbols",
)

_libs = {}


def _lib(name):
    if name not in _libs:
        with open(os.path.join(SYMBOL_DIR, name + ".kicad_sym")) as f:
            tree = parse(f.read())
        _libs[name] = {s[1]: s for s in find(tree, "symbol")}
    return _libs[name]


def load(lib_id):
    """Return a flattened symbol definition named `lib_id` (lib:name)."""
    lib, name = lib_id.split(":")
    syms = _lib(lib)
    sym = copy.deepcopy(syms[name])
    ext = first(sym, "extends")
    if ext:
        parent = load(f"{lib}:{ext[1]}")
        props = {p[1]: p for p in find(sym, "property")}
        merged = [Sym("symbol"), lib_id]
        for item in parent[2:]:
            if isinstance(item, list) and item[0] == "property" and item[1] in props:
                merged.append(props.pop(item[1]))
            elif isinstance(item, list) and item[0] == "symbol":
                unit = copy.deepcopy(item)
                unit[1] = name + unit[1][len(ext[1]):] if unit[1].startswith(ext[1]) else unit[1]
                unit[1] = unit[1].split(":")[-1]
                merged.append(unit)
            else:
                merged.append(item)
        merged.extend(props.values())
        return merged
    sym[1] = lib_id
    return sym


def pins(sym):
    """List of dicts: number, name, x, y, angle, type (library coordinates)."""
    out = []
    for unit in find(sym, "symbol"):
        for p in find(unit, "pin"):
            at = first(p, "at")
            out.append({
                "type": str(p[1]),
                "number": first(p, "number")[1],
                "name": first(p, "name")[1],
                "x": float(at[1]),
                "y": float(at[2]),
                "angle": float(at[3]) if len(at) > 3 else 0.0,
            })
    return out


if __name__ == "__main__":
    import sys
    for lid in sys.argv[1:]:
        print("==", lid)
        for p in pins(load(lid)):
            print(f'{p["number"]:>4} {p["name"]:<14} {p["type"]:<14} ({p["x"]},{p["y"]}) {p["angle"]}')
