"""Wired, multi-sheet schematic for the single_sh board.

build.single() defines the circuit; this file only arranges it. Every part is
placed by reference and every pin is wired, labelled or flagged here, and
schdraw checks the drawing against build.py's nets before writing.

Sheets: overview (root), power, MCU, motor drivers, connectors and UI.

    python3 single_sh_sch.py    # check and lint the drawing without writing it
"""

import os

import schdraw
from schgen import FP_LIB_TABLE
from sexp import Sym, dump

AT8833 = "stack:AT8833CQ"
LIB_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "lib")


# ---------------------------------------------------------------------------
# AT8833CQ symbol: same pins as Driver_Motor:DRV8833RTY, arranged for this
# drawing (inputs left, outputs right, supplies on top, sense/GND below).
# ---------------------------------------------------------------------------

_AT8833_PINS = [
    # number, name, type, x, y, angle (library coordinates, y up), hidden
    ("14", "AIN1", "input", -15.24, 10.16, 0, False),
    ("13", "AIN2", "input", -15.24, 7.62, 0, False),
    ("7", "BIN1", "input", -15.24, 2.54, 0, False),
    ("8", "BIN2", "input", -15.24, 0, 0, False),
    ("15", "~{SLEEP}", "input", -15.24, -5.08, 0, False),
    ("6", "~{FAULT}", "open_collector", -15.24, -7.62, 0, False),
    ("16", "AOUT1", "output", 15.24, 10.16, 180, False),
    ("2", "AOUT2", "output", 15.24, 7.62, 180, False),
    ("5", "BOUT1", "output", 15.24, 2.54, 180, False),
    ("3", "BOUT2", "output", 15.24, 0, 180, False),
    ("12", "VINT", "power_out", 15.24, -5.08, 180, False),
    ("9", "VCP", "passive", -5.08, 17.78, 270, False),
    ("10", "VM", "power_in", 5.08, 17.78, 270, False),
    ("1", "AISEN", "passive", -7.62, -20.32, 90, False),
    ("11", "GND", "power_in", 0, -20.32, 90, False),
    ("17", "GND", "passive", 0, -20.32, 90, True),
    ("4", "BISEN", "passive", 7.62, -20.32, 90, False),
]


def _font():
    return [Sym("effects"), [Sym("font"), [Sym("size"), 1.27, 1.27]]]


def at8833_symbol(name=AT8833):
    def prop(k, v, x, y, hide=False):
        node = [Sym("property"), k, v, [Sym("at"), x, y, 0]]
        if hide:
            node.append([Sym("hide"), Sym("yes")])
        node.append(_font())
        return node

    short = name.split(":")[-1]
    pins = []
    for num, pname, kind, x, y, ang, hidden in _AT8833_PINS:
        pin = [Sym("pin"), Sym(kind), Sym("line"), [Sym("at"), x, y, ang], [Sym("length"), 2.54]]
        if hidden:
            pin.append([Sym("hide"), Sym("yes")])
        pin += [[Sym("name"), pname, _font()], [Sym("number"), num, _font()]]
        pins.append(pin)
    return [Sym("symbol"), name,
            [Sym("exclude_from_sim"), Sym("no")], [Sym("in_bom"), Sym("yes")], [Sym("on_board"), Sym("yes")],
            prop("Reference", "U", 0, 2.54), prop("Value", "AT8833CQ", 0, 0),
            prop("Footprint", "Package_DFN_QFN:QFN-16-1EP_4x4mm_P0.65mm_EP2.1x2.1mm", 0, -25.4, True),
            prop("Datasheet", "", 0, 0, True),
            prop("Description", "Dual H-bridge motor driver, DRV8833-compatible, QFN-16 4x4 (Zhongkewei AT8833CQ)",
                 0, 0, True),
            [Sym("symbol"), f"{short}_0_1",
             [Sym("rectangle"), [Sym("start"), -12.7, 15.24], [Sym("end"), 12.7, -17.78],
              [Sym("stroke"), [Sym("width"), 0.254], [Sym("type"), Sym("default")]],
              [Sym("fill"), [Sym("type"), Sym("background")]]]],
            [Sym("symbol"), f"{short}_1_1"] + pins]


def write_symbol_lib(path=os.path.join(LIB_DIR, "stack.kicad_sym")):
    lib = [Sym("kicad_symbol_lib"), [Sym("version"), 20231120], [Sym("generator"), "kicad_symbol_editor"],
           [Sym("generator_version"), "8.0"], at8833_symbol("AT8833CQ")]
    with open(path, "w") as f:
        f.write(dump(lib) + "\n")


# ---------------------------------------------------------------------------
# Shared helpers
# ---------------------------------------------------------------------------

def cap_row(sh, refs, x, y, pitch=10.16, top=None):
    """Caps standing between a top rail (their pin 1 net) and a GND rail."""
    top = top or sh.design.parts[refs[0]].nets["1"]
    end = x + pitch * len(refs)
    sh.wire((x, y), (end, y))
    sh.wire((x, y + 7.62), (end, y + 7.62))
    sh.power(top, (x, y))
    sh.power("GND", (x, y + 7.62))
    for k, ref in enumerate(refs):
        sh.two_pin(ref, (x + pitch * (k + 1), y), "D")


def notes(sh, x1, y1, x2, text, title="Notes", size=1.27):
    """Framed block of notes, sized to its text."""
    lines = text.count("\n") + 1
    sh.frame(x1, y1, x2, y1 + 7.62 + lines * size * 1.6 + 2.54, title)
    sh.text(x1 + 2.54, y1 + 7.62, text, size=size)


# ---------------------------------------------------------------------------
# Power: USB-C, power path, soft-start, 3.3 V LDO, external 5 V, V5 sense
# ---------------------------------------------------------------------------

def draw_power(sh):
    sh.frame(17.78, 48.26, 185.42, 132.08, "USB-C and power path")
    j1 = sh.place("J1", 38.1, 88.9)
    sh.pin_label(j1, "A5", "global", shape="output")      # CC1
    sh.pin_label(j1, "B5", "global", shape="output")      # CC2
    for a, b in (("A7", "B7"), ("A6", "B6")):             # D- pair, D+ pair
        ta, tb = sh.stub(j1[a], "R"), sh.stub(j1[b], "R")
        sh.wire(ta, tb)
        sh.label(j1.net(a), sh.stub(ta, "R"), "R")
    sh.nc(j1, "A8")
    sh.nc(j1, "B8")
    g1, g2 = sh.stub(j1["A1"], "D"), sh.stub(j1["SH"], "D")
    sh.wire(g2, g1)
    sh.power("GND", g1)
    sh.wire(g2, (25.4, g2[1]))
    sh.flag((25.4, g2[1]), "D")

    # VBUS -> polyfuse -> Schottky -> V5_IN.
    y = j1["A4"][1]
    sh.power("VBUS", (58.42, y))
    sh.flag((68.58, y))
    f1 = sh.two_pin("F1", (76.2, y), "R")
    sh.wire(j1["A4"], f1["1"])
    d1 = sh.two_pin("D1", (101.6, y), "L")
    sh.wire(f1["2"], d1["2"])
    sh.label("VBUS_F", (85.09, y), "R")

    # V5_IN: bulk cap, LDO feed, soft-start switch.
    q1 = sh.place("Q1", 147.32, y + 2.54, rot=270, mirror="x",
                  ref_at=(0, -10.16, "center"), val_at=(0, -7.62, "center"))
    sh.wire(d1["1"], q1["S"])
    sh.label("V5_IN", (104.14, y), "R")
    c52 = sh.two_pin("C52", (111.76, y), "D", side="L")
    sh.pin_power(c52, "2")
    sh.flag((116.84, y))
    # Soft-start: C54 holds the gate at the source at plug-in, R30 then pulls it down.
    c54 = sh.two_pin("C54", (132.08, y), "D", side="L")
    gate = (q1["G"][0], y + 12.7)
    sh.vh(c54["2"], gate)
    sh.wire(q1["G"], gate)
    sh.label("SS_GATE", (134.62, gate[1]), "R")
    r30 = sh.two_pin("R30", gate, "D")
    sh.pin_power(r30, "2")
    # V5 bus out of the switch.
    v5 = (165.1, y)
    sh.wire(q1["D"], v5)
    sh.power("V5", v5)
    sh.flag((160.02, y), "D")

    # 3.3 V LDO, fed from V5_IN ahead of the switch.
    u13 = sh.place("U13", 152.4, 114.3, ref_at=(0, -7.62, "center"), val_at=(0, -10.16, "center"))
    vin = u13["1"]
    sh.wire((121.92, y), (121.92, vin[1]), vin)
    sh.wire(vin, u13["3"])
    c53 = sh.two_pin("C53", (129.54, vin[1]), "D")
    sh.pin_power(c53, "2")
    sh.nc(u13, "4")
    sh.pin_power(u13, "2")
    out = (177.8, u13["5"][1])
    sh.wire(u13["5"], out)
    sh.power("+3V3", out)
    c55 = sh.two_pin("C55", (167.64, out[1]), "D")
    sh.pin_power(c55, "2")

    # USB data: series resistors to the MCU, ESD at the connector side.
    sh.frame(17.78, 137.16, 134.62, 177.8, "USB data")
    for k, (r, net) in enumerate((("R27", "USB_DM"), ("R26", "USB_DP"))):
        ly = 152.4 + 7.62 * k
        sh.wire((33.02, ly), (40.64, ly))
        sh.label(net, (33.02, ly), "L")
        rr = sh.two_pin(r, (40.64, ly), "R")
        sh.pin_label(rr, "2", "global", length=7.62, shape="bidirectional")
    u12 = sh.place("U12", 109.22, 156.21, ref_at=(0, -10.16, "center"), val_at=(0, -7.62, "center"))
    sh.pin_label(u12, "5", "local", length=5.08)
    sh.pin_label(u12, "3", "local", length=5.08)
    sh.nc(u12, "1")
    sh.nc(u12, "2")
    sh.pin_power(u12, "4")

    # CC pull-downs: advertise a sink; the MCU reads the source current on ADC0/1.
    sh.frame(139.7, 137.16, 193.04, 177.8, "CC pull-downs")
    for k, r in enumerate(("R28", "R29")):
        x = 160.02 + 20.32 * k
        rr = sh.two_pin(r, (x, 152.4), "D")
        sh.wire((x - 7.62, 152.4), rr["1"])
        sh.glabel(rr.net("1"), (x - 7.62, 152.4), "L", "input")
        sh.pin_power(rr, "2")

    # External 5 V input, straight onto the V5 bus.
    sh.frame(190.5, 48.26, 256.54, 96.52, "External 5 V input")
    j4 = sh.place("J4", 200.66, 73.66, mirror="y", ref_at=(0, -3.81, "center"), val_at=(0, 6.35, "center"))
    p1 = j4["1"]
    d3 = sh.two_pin("D3", (236.22, p1[1]), "L")
    sh.wire(p1, d3["2"])
    sh.flag((208.28, p1[1]))
    sh.label("+5V_EXT", (213.36, p1[1]), "R")
    c59 = sh.two_pin("C59", (223.52, p1[1]), "D", side="L")
    sh.pin_power(c59, "2")
    sh.wire(d3["1"], (243.84, p1[1]))
    sh.power("V5", (243.84, p1[1]))
    sh.pin_power(j4, "2", length=2.54)

    # V5 sense divider for the brownout guard.
    sh.frame(261.62, 48.26, 327.66, 96.52, "V5 sense (GPIO42 / ADC2)")
    x = 274.32
    sh.power("V5", (x, 63.5))
    r22 = sh.two_pin("R22", (x, 63.5), "D")
    node = sh.stub(r22["2"], "D")
    r23 = sh.two_pin("R23", node, "D")
    sh.pin_power(r23, "2")
    c34 = sh.two_pin("C34", (x + 15.24, node[1]), "D")
    sh.pin_power(c34, "2")
    sh.wire(node, c34["1"])
    sh.wire(c34["1"], (x + 25.4, node[1]))
    sh.glabel("VMOT_SENSE", (x + 25.4, node[1]), "R", "output")
    sh.text(264.16, 88.9, "VMOT_SENSE = V5 x 10k / 49k (1:4.9)", size=1.27)

    notes(sh, 198.12, 104.14, 375.92,
            "Power path: VBUS -> PTC F1 (1.5 A hold) -> Schottky D1 -> V5_IN -> soft-start switch Q1 -> V5 bus.\n"
            "Soft-start: at plug-in C54 holds Q1's gate at its source; R30 then pulls the gate down (tau 10 ms),\n"
            "so the ~70 uF on V5 charges at about 0.1 A. USB sees only the ~11 uF on V5_IN at attach.\n"
            "External 5 V reaches V5 through D3, and V5_IN through Q1's body diode, then its channel.\n"
            "The LDO runs from V5_IN, ahead of the switch. The ME6211 conducts from VOUT back to VIN, so the\n"
            "board must never be powered from Qwiic: a Qwiic-fed 3V3 would back-power the V5 bus.\n"
            "VMOT_SENSE can't tell external power from USB: both feed V5 through Schottkys.\n"
            "Wait about 100 ms after power-up before energizing motors (soft-start settling).\n"
            "USB: 27 ohm series resistors at the MCU side; TPD2E2U06 ESD (1.5 pF) at the receptacle.")


# ---------------------------------------------------------------------------
# MCU: RP2354B, supplies, core regulator, crystal, BOOTSEL, SWD
# ---------------------------------------------------------------------------

def draw_mcu(sh):
    sh.frame(76.2, 68.58, 215.9, 223.52, "RP2354B")
    mx, my = 165.1, 152.4
    u = sh.place("U11", mx, my, ref_at=(12.7, 60.96, "left"), val_at=(12.7, 63.5, "left"))

    # Supply pins: 3V3 group, core regulator, 1V1 group.
    top = u["IOVDD"][1]
    rail3 = top - 7.62
    for key in ("USB_OTP_VDD", "ADC_AVDD", "QSPI_IOVDD", "IOVDD", "VREG_VIN"):
        sh.wire(u[key], (u[key][0], rail3))
    sh.wire((u["USB_OTP_VDD"][0], rail3), (u["VREG_VIN"][0], rail3))
    sh.power("+3V3", (157.48, rail3))
    rail1 = top - 5.08
    lx_y = top - 12.7
    l1 = sh.two_pin("L1", (187.96, lx_y), "D", side="L")
    sh.wire(u["VREG_LX"], (u["VREG_LX"][0], lx_y), l1["1"])
    sh.label("VREG_LX", (172.72, lx_y), "R")
    for key in ("VREG_FB", "DVDD"):
        sh.wire(u[key], (u[key][0], rail1))
    sh.wire((u["VREG_FB"][0], rail1), (200.66, rail1))
    sh.power("+1V1", (200.66, rail1))
    sh.flag((195.58, rail1), "D")
    sh.text(186.69, lx_y - 10.16, "L1 polarity dot at the\n+1V1 end, as on the Pico 2", size=1.0, italic=True)
    # VREG_AVDD: RC from 3V3.
    ay = top - 17.78
    r24 = sh.two_pin("R24", (104.14, ay), "R", side="D")
    sh.power("+3V3", sh.stub(r24["1"], "L"))
    sh.wire(r24["2"], (u["VREG_AVDD"][0], ay), u["VREG_AVDD"])
    c49 = sh.two_pin("C49", (116.84, ay), "D")
    sh.pin_power(c49, "2")
    sh.flag((121.92, ay))
    sh.label("VREG_AVDD", (127, ay), "R")
    g1, g2 = sh.stub(u["VREG_PGND"], "D"), sh.stub(u["GND"], "D")
    sh.wire(g1, g2)
    sh.power("GND", g2)

    # Left side: RUN, USB, BOOTSEL, QSPI (unused), crystal, SWD, GPIO40-47.
    run = sh.wire(u["RUN"], (127, u["RUN"][1]))
    sh.label("RUN", (129.54, run[1]), "R")
    sh.place("TP3", run[0], run[1], rot=90, ref_at=(-6.35, 0, "center"), hide_value=True)
    sh.pin_label(u, "USB_DM", "global", shape="bidirectional")
    sh.pin_label(u, "USB_DP", "global", shape="bidirectional")
    ss = u["~{QSPI_SS}"]
    sw3 = sh.two_pin("SW3", (124.46, ss[1]), "L")
    sh.wire(ss, sw3["1"])
    sh.label("QSPI_SS", (127, ss[1]), "R")
    r36 = sh.two_pin("R36", sh.stub(sw3["2"], "L"), "D", side="L")
    sh.pin_power(r36, "2")
    for key in ("QSPI_SCLK", "QSPI_SD0", "QSPI_SD1", "QSPI_SD2", "QSPI_SD3"):
        sh.nc(u, key)

    # Crystal, per the RP2350 hardware design guide.
    yc = 175.26
    y1 = sh.place("Y1", 101.6, yc, ref_at=(0, -7.62, "center"), val_at=(0, -5.08, "center"))
    xin = u["XIN"]
    sh.wire(xin, (91.44, xin[1]), (91.44, yc), y1["1"])
    sh.label("XIN", (129.54, xin[1]), "R")
    c50 = sh.two_pin("C50", (91.44, yc), "D", side="L")
    sh.pin_power(c50, "2")
    c51 = sh.two_pin("C51", (111.76, yc), "D")
    sh.pin_power(c51, "2")
    sh.wire(y1["3"], (111.76, yc))
    xout = u["XOUT"]
    r25 = sh.two_pin("R25", (132.08, xout[1]), "L")
    sh.wire((111.76, yc), (111.76, xout[1]), r25["2"])
    sh.label("XTAL_OUT", (113.03, xout[1]), "R")
    sh.wire(r25["1"], xout)
    sh.label("XOUT", (133.35, xout[1]), "R")
    sh.pin_power(y1, "2")

    # SWD pads.
    for ref, key in (("TP1", "SWCLK"), ("TP2", "SWDIO")):
        end = sh.stub(u[key], "L", 10.16)
        sh.label(key, (end[0] + 2.54, end[1]), "R")
        sh.place(ref, end[0], end[1], rot=90, ref_at=(-6.35, 0, "center"), hide_value=True)

    # GPIO40-47: ADC and peripheral signals. GPIO0-39: motor inputs, four per driver.
    for g in range(40, 48):
        sh.pin_label(u, f"GPIO{g}/ADC{g - 40}", "global", shape="bidirectional")
    for g in range(40):
        sh.pin_label(u, f"GPIO{g}", "global", shape="output")

    # Decoupling, one cap per supply pin.
    sh.frame(76.2, 22.86, 264.16, 63.5, "Supply decoupling")
    sh.text(86.36, 30.48, "3V3: 100 nF at each IOVDD, QSPI_IOVDD, USB_OTP_VDD and ADC_AVDD pin;"
                          " 4.7 uF (C44) at VREG_VIN", size=1.27)
    cap_row(sh, [f"C{n}" for n in range(35, 45)], 86.36, 43.18)
    sh.text(203.2, 30.48, "1V1: 100 nF at each DVDD pin;\n4.7 uF (C48) at the regulator output", size=1.27)
    cap_row(sh, ["C45", "C46", "C47", "C48"], 208.28, 43.18)

    # Supply test pads.
    sh.text(83.82, 198.12, "Test pads", size=1.27, bold=True)
    tp4 = sh.place("TP4", 88.9, 208.28, ref_at=(2.54, -3.81, "left"), val_at=(2.54, -1.27, "left"))
    sh.pin_power(tp4, "1", 2.54)
    tp5 = sh.place("TP5", 104.14, 210.82, rot=180, ref_at=(2.54, 1.27, "left"), val_at=(2.54, 3.81, "left"))
    sh.pin_power(tp5, "1", 2.54)

    notes(sh, 220.98, 68.58, 375.92,
            "RP2354B: RP2350B with 2 MB flash in the package; QSPI_SD0-3 and QSPI_SCLK stay unconnected.\n"
            "Run the ARM cores (FPU).\n"
            "Crystal: Abracon ABM8-272-T3 (10 pF load, ESR <= 50 ohm). The design guide's 15 pF / 1 k values\n"
            "are tuned for it; another crystal needs retuning and testing.\n"
            "BOOTSEL (SW3): hold at reset for USB boot; firmware can also read it at runtime.\n"
            "\n"
            "GPIO 0-39: motor n on GPIO 4(n-1) .. 4(n-1)+3 = AIN1 AIN2 BIN1 BIN2.\n"
            "GPIO40 CC1 (ADC0), GPIO41 CC2 (ADC1), GPIO42 VMOT_SENSE (ADC2), GPIO43 BOARD_ID (ADC3),\n"
            "GPIO44 LADDER (ADC4), GPIO45 status LED data, GPIO46 / 47 Qwiic SDA / SCL (I2C1).\n"
            "\n"
            "PIO0: motors 1-4. PIO1: motors 5-8. PIO2 (GPIOBASE 16): motors 9-10, WS2812, one spare SM.\n"
            "A2 silicon (RP2350-E9): use external pull-ups on all inputs; fixed from A3 (order A4).")


# ---------------------------------------------------------------------------
# Motor drivers
# ---------------------------------------------------------------------------

DRV_COLS = [50.8 + 76.2 * i for i in range(5)]
DRV_ROWS = [63.5, 149.86]


def driver_cell(sh, m, x, y):
    """One AT8833 with its sense resistors and caps; chip origin at (x, y)."""
    n = m - 1
    ra, rb = f"R{2 * n + 1}", f"R{2 * n + 2}"
    c_vint, c_vcp, c_vm = f"C{3 * n + 1}", f"C{3 * n + 2}", f"C{3 * n + 3}"
    g = 4 * n
    conn = 10 + (m + 1) // 2
    pins = "1-4" if m % 2 else "5-8"
    sh.frame(x - 34.29, y - 43.18, x + 34.29, y + 38.1, f"Motor {m}")
    sh.text(x - 31.75, y - 37.47, f"GPIO{g}-{g + 3} -> J{conn} pins {pins}", size=1.27)

    u = sh.place(f"U{m}", x, y, lib_id=AT8833, ref_at=(0, -3.81, "center"), val_at=(0, -1.27, "center"))
    for key in ("AIN1", "AIN2", "BIN1", "BIN2"):
        sh.pin_label(u, key, "global", shape="input")
    for key in ("AOUT1", "AOUT2", "BOUT1", "BOUT2"):
        sh.pin_label(u, key, "global", shape="output")
    sh.pin_label(u, "~{SLEEP}", "local")
    sh.pin_label(u, "~{FAULT}", "global", shape="output")

    # Supplies: VCP cap and VM on a short V5 rail, VM decoupling to its right.
    top = y - 27.94
    vcp = sh.two_pin(c_vcp, sh.stub(u["VCP"], "U"), "U", side="L")
    sh.wire(u["VM"], (x + 5.08, top))
    sh.wire(vcp["2"], (x + 15.24, top))
    sh.power("V5", (x + 5.08, top))
    cvm = sh.two_pin(c_vm, (x + 15.24, top), "D")
    sh.pin_power(cvm, "2")

    # VINT cap off the right side.
    cv = sh.two_pin(c_vint, sh.stub(u["VINT"], "R", 7.62), "D")
    sh.pin_power(cv, "2")

    # Sense resistors and ground below.
    r1 = sh.two_pin(ra, sh.stub(u["AISEN"], "D"), "D", side="L")
    sh.pin_power(r1, "2")
    r2 = sh.two_pin(rb, sh.stub(u["BISEN"], "D"), "D")
    sh.pin_power(r2, "2")
    sh.pin_power(u, "11")


def draw_drivers(sh):
    for i, m in enumerate(range(1, 11)):
        driver_cell(sh, m, DRV_COLS[i % 5], DRV_ROWS[i // 5])

    # Shared bulk on the V5 pour, and the nSLEEP pull-up.
    sh.frame(16.51, 195.58, 85.09, 233.68, "V5 bulk, nSLEEP")
    cap_row(sh, ["C31", "C32", "C33"], 22.86, 215.9)
    r = sh.two_pin("R21", (71.12, 213.36), "D")
    sh.power("+3V3", r["1"])
    end = sh.wire(r["2"], (71.12, 226.06), (73.66, 226.06))
    sh.label("NSLEEP", end, "R")

    notes(sh, 92.71, 195.58, 241.3,
            "AT8833CQ (Zhongkewei, LCSC C5120769): DRV8833-compatible logic, 00 coast, 01 reverse, 10 forward, 11 brake.\n"
            "VM is the V5 bus directly. Inputs have internal 100k pull-downs; nSLEEP is pulled high (always awake).\n"
            "xISEN: 0.82 ohm 0805 to ground; the chip chops at VTRIP / R = 0.2-0.29 A (VTRIP 160-240 mV), slow decay.\n"
            "Give each sense resistor its own return to the star ground.\n"
            "VCP 0.1 uF to VM and VINT 1 uF are the AT8833 datasheet values (TI's DRV8833 uses 0.01 uF and 2.2 uF).\n"
            "nFAULT (open drain) of every driver is wire-ORed onto LADDER: a fault pulls it to 0 V (STOP).\n"
            "Second source: JSMSEMI DRV8833RTYR (same QFN-16 4x4); check its pinout against TI's first.")


# ---------------------------------------------------------------------------
# Connectors and user interface
# ---------------------------------------------------------------------------

def buffer_stage(sh, ref, x, y):
    """74AHCT1G125 with OE tied low and VCC on V5; returns the placed part."""
    u = sh.place(ref, x, y, ref_at=(2.54, -6.35, "left"), val_at=(2.54, 8.89, "left"))
    sh.pin_power(u, "5", 2.54)
    sh.pin_power(u, "3")
    oe = sh.stub(u["1"], "U")
    sh.wire(oe, (x + 7.62, oe[1]))
    sh.power("GND", (x + 7.62, oe[1]))
    return u


def draw_io(sh):
    # Motor connectors.
    sh.frame(17.78, 22.86, 289.56, 78.74, "Motor connectors")
    for k, ref in enumerate(("J11", "J12", "J13", "J14", "J15")):
        x = 48.26 + 53.34 * k
        j = sh.place(ref, x, 50.8, ref_at=(0, -10.16, "center"), val_at=(0, 13.97, "center"))
        for pin in range(1, 9):
            sh.pin_label(j, str(pin), "global", shape="input")
    sh.text(20.32, 69.85,
            "JST-SH 8P (BM08B-SRSS-TB), two 8 mm micro steppers per plug: pins 1-4 = first motor A1 A2 B1 B2, "
            "pins 5-8 = second motor.\n"
            "Firmware remaps coil order and polarity per axis, so lead-colour variations are fixed in software.",
            size=1.27)

    # Status LED, then the buffered strip output.
    sh.frame(17.78, 83.82, 215.9, 142.24, "Status LED and LED strip output")
    y = 116.84
    u14 = buffer_stage(sh, "U14", 50.8, y)
    sh.pin_label(u14, "2", "global", shape="input")
    d2 = sh.place("D2", 81.28, y, ref_at=(7.62, -6.35, "left"), val_at=(7.62, -3.81, "left"))
    sh.wire(u14["4"], d2["DIN"])
    sh.pin_power(d2, "VDD")
    sh.pin_power(d2, "VSS")
    u15 = buffer_stage(sh, "U15", 127, y)
    sh.wire(d2["DOUT"], u15["2"])
    sh.label("LED_DATA", (93.98, y), "R")
    r38 = sh.two_pin("R38", sh.stub(u15["4"], "R"), "R")
    j5 = sh.place("J5", 167.64, y, ref_at=(3.81, -2.54, "left"), val_at=(3.81, 0, "left"))
    sh.wire(r38["2"], j5["2"])
    sh.label("STRIP_DIN", (151.13, y), "R")
    sh.pin_power(j5, "3", 2.54)
    # Strip power: fused V5 (JP3 bridged) or a separate supply on LED PWR.
    py = 96.52
    sh.power("V5", (109.22, py))
    f2 = sh.two_pin("F2", (114.3, py), "R", ref_at=(0, -5.08, "center"), val_at=(0, -2.54, "center"))
    sh.wire((109.22, py), f2["1"])
    jp3 = sh.two_pin("JP3", (142.24, py), "R")
    sh.wire(f2["2"], jp3["1"])
    sh.label("STRIP_V5_FUSED", (123.19, py), "R")
    node = (160.02, py)
    sh.wire(jp3["2"], node)
    sh.wire(node, (node[0], j5["1"][1]), j5["1"])
    sh.label("STRIP_5V", (161.29, py), "R")
    j6 = sh.place("J6", 198.12, py, ref_at=(3.81, -2.54, "left"), val_at=(3.81, 0, "left"))
    sh.wire(node, j6["1"])
    sh.pin_power(j6, "2", 2.54)
    c61 = sh.two_pin("C61", (172.72, py), "D")
    sh.pin_power(c61, "2")
    sh.flag((182.88, py))
    cap_row(sh, ["C57", "C58", "C60"], 167.64, 127)
    sh.text(20.32, 135.89, "C57 at U14, C58 at D2, C60 at U15. Strip power: cut JP3 (STRIP_SHARED) when the strip "
                           "has its own supply on J6.", size=1.27)

    # Qwiic x2 on I2C1. 3V3 only goes out (JP1); the board is never powered from Qwiic.
    sh.frame(220.98, 83.82, 325.12, 142.24, "Qwiic (I2C1)")
    for k, ref in enumerate(("J2", "J3")):
        y = 101.6 + 22.86 * k
        j = sh.place(ref, 246.38, y, mirror="x", ref_at=(0, -7.62, "center"), val_at=(0, 6.35, "center"))
        sh.pin_power(j, "1", 2.54)
        sh.pin_label(j, "2", "global")
        sh.pin_label(j, "3", "global", shape="bidirectional")
        sh.pin_label(j, "4", "global", shape="bidirectional")
    jp1 = sh.two_pin("JP1", (269.24, 96.52), "R")
    sh.power("+3V3", sh.stub(jp1["1"], "L", 5.08))
    sh.pin_label(jp1, "2", "global")
    jp2 = sh.two_pin("JP2", (294.64, 116.84), "U")
    sh.power("+3V3", jp2["2"])
    r31 = sh.two_pin("R31", (287.02, 132.08), "U", side="L")
    r32 = sh.two_pin("R32", (302.26, 132.08), "U")
    sh.wire(r31["2"], (r31["2"][0], jp2["1"][1]), jp2["1"])
    sh.wire(jp2["1"], (r32["2"][0], jp2["1"][1]), r32["2"])
    for r, d in ((r31, "L"), (r32, "R")):
        end = sh.stub(sh.stub(r["1"], "D"), d, 5.08)
        sh.glabel(r.net("1"), end, d, "bidirectional")

    # Analog ladder on LADDER (GPIO44 / ADC4).
    sh.frame(17.78, 144.78, 215.9, 220.98, "Analog ladder (GPIO44 / ADC4)")
    by = 170.18
    r33 = sh.two_pin("R33", (40.64, 154.94), "D")
    sh.power("+3V3", r33["1"])
    sh.wire(r33["2"], (40.64, by))
    sh.wire((30.48, by), (160.02, by))
    sh.glabel("LADDER", (30.48, by), "L", "bidirectional")
    c56 = sh.two_pin("C56", (50.8, by), "D")
    sh.pin_power(c56, "2")
    for x, sw, r in ((63.5, "SW1", "R34"), (81.28, "SW2", "R35")):
        s = sh.two_pin(sw, sh.stub((x, by), "D"), "D")
        rr = sh.two_pin(r, sh.stub(s["2"], "D"), "D")
        sh.pin_power(rr, "2")
    # E-stop: NC switch to the 20k load; NO_ESTOP bridges it when none is fitted.
    jp4 = sh.two_pin("JP4", sh.stub((106.68, by), "D"), "D", side="L")
    ret = sh.stub(jp4["2"], "D")
    r39 = sh.two_pin("R39", ret, "D", side="L")
    sh.pin_power(r39, "2")
    j7 = sh.place("J7", 129.54, by + 5.08, ref_at=(3.81, -1.27, "left"), val_at=(3.81, 1.27, "left"))
    sh.wire(j7["1"], (119.38, j7["1"][1]), (119.38, by))
    sh.wire(j7["2"], (114.3, j7["2"][1]), (114.3, ret[1]), ret)
    # Panel buttons in parallel with the onboard ones.
    j8 = sh.place("J8", 175.26, by + 5.08, ref_at=(3.81, -2.54, "left"), val_at=(3.81, 0, "left"))
    sh.wire(j8["1"], (160.02, j8["1"][1]), (160.02, by))
    r40 = sh.two_pin("R40", (147.32, by + 15.24), "D", side="L")
    r41 = sh.two_pin("R41", (154.94, by + 15.24), "D")
    sh.wire(j8["2"], (147.32, j8["2"][1]), r40["1"])
    sh.wire(j8["3"], (154.94, j8["3"][1]), r41["1"])
    sh.pin_power(r40, "2")
    sh.pin_power(r41, "2")
    sh.text(20.32, 205.74,
            "LADDER: E-stop open or cable cut 3.3 V (STOP) | idle ~2.2 V | Btn1 ~1.3 V | Btn2 ~0.7 V | "
            "driver nFAULT ~0 V (STOP).\n"
            "1% resistors; thresholds midway between levels, debounce over 2-3 samples. A held button hides an\n"
            "open e-stop, so firmware latches a stop on 1.49-1.76 V held 5 ms or any button held over 3 s.\n"
            "J8 panel buttons: from pin 1 to pin 2 (Btn1) / pin 3 (Btn2), in parallel with SW1 / SW2.\n"
            "NO_ESTOP (JP4) ships bridged: CUT IT when an NC e-stop is fitted on J7, or the e-stop does nothing.",
            size=1.27)

    # Board ID: tells firmware which connector variant this is.
    sh.frame(220.98, 144.78, 325.12, 195.58, "Board ID (GPIO43 / ADC3)")
    x = 238.76
    r37 = sh.two_pin("R37", (x, 160.02), "D")
    sh.power("+3V3", r37["1"])
    node = sh.stub(r37["2"], "D")
    r42 = sh.two_pin("R42", node, "D")
    sh.pin_power(r42, "2")
    sh.wire(node, (x + 10.16, node[1]))
    sh.glabel("BOARD_ID", (x + 10.16, node[1]), "R", "output")
    sh.text(223.52, 187.96, "4.7 k to GND = io_sh connectors (dual 8 mm micro steppers).\n"
                            "10 k 1% pull-up; levels in hardware/README.md.", size=1.27)


# ---------------------------------------------------------------------------
# Overview (root sheet)
# ---------------------------------------------------------------------------

def draw_root(sh, pwr, mcu, drv, io):
    sh.text(25.4, 30.48, "RP2350 stepper controller - single board, dual 8 mm micro steppers (single_sh)",
            size=3.0, bold=True)
    sh.text(25.4, 38.1, "Controller, universal driver and io_sh connector boards on one 122 x 20 mm, 4-layer PCB. "
                        "Click a sheet to open it.", size=1.5)
    w, h, y = 66.04, 30.48, 63.5
    xs = [25.4, 121.92, 218.44, 314.96]
    blurbs = [
        "USB-C receptacle, ESD, CC sense\nPTC, Schottky, soft-start P-FET\n3.3 V LDO\nexternal 5 V input\nV5 sense divider",
        "RP2354B, 2 MB flash in package\ncore regulator, 12 MHz crystal\nsupply decoupling\nBOOTSEL, SWD pads",
        "10x AT8833CQ dual H-bridge\n0.82 ohm current-limit sense\nnSLEEP pull-up, V5 bulk\nnFAULT onto LADDER",
        "5x JST-SH 8P motor plugs\nQwiic x2, status LED, LED strip\nbuttons, e-stop, panel ladder\nboard ID",
    ]
    for x, s, blurb in zip(xs, (pwr, mcu, drv, io), blurbs):
        sh.sheet_symbol(s, x, y, w, h)
        sh.text(x + 2.54, y + 5.08, blurb, size=1.5)
    links = [
        ("+3V3, V5\nCC1, CC2\nUSB D+ / D-\nVMOT_SENSE",),
        ("M1-M10\nAIN1/2, BIN1/2\n(GPIO0-39)",),
        ("M1-M10\nAOUT1/2, BOUT1/2",),
    ]
    for k, (txt,) in enumerate(links):
        a, b = xs[k] + w, xs[k + 1]
        sh.arrow((a + 1.27, y + h / 2), (b - 1.27, y + h / 2))
        sh.text(a + 2.54, y + h / 2 + 2.54, txt, size=1.27)
    # Feedback from the IO sheet and the drivers to the MCU.
    yb = y + h + 15.24
    sh.line((xs[3] + w / 2, y + h + 6.35), (xs[3] + w / 2, yb), (xs[1] + w / 2, yb), width=0.3)
    sh.arrow((xs[1] + w / 2, yb), (xs[1] + w / 2, y + h + 6.35))
    sh.text(xs[1] + w / 2 + 2.54, yb + 2.54,
            "LADDER (buttons, e-stop, panel, driver nFAULT), BOARD_ID, SDA / SCL, LED_DIN", size=1.27)

    sh.text(25.4, 132.08, "Board", size=2.0, bold=True)
    sh.text(25.4, 137.16,
            "122 x 20 mm, 4 layers (F.Cu parts and signals, In1 GND, In2 V5 with a 3V3 island, B.Cu signals), 1.6 mm.\n"
            "All parts on top. Along the board: USB-C and power path | drivers M4-M1 | MCU | drivers M10-M5 | LED strip.\n"
            "Motor plugs along one long edge (J12 J11 | J15 J14 J13); Qwiic, buttons, e-stop, 5 V in and LED power along the other.\n"
            "Same circuit as the controller / driver / io_sh stack with the board-to-board connectors removed;\n"
            "the board-ID resistor stays so firmware identifies the connector variant the same way.\n"
            "\n"
            "The circuit is defined in hardware/gen/build.py (single()); the drawing in hardware/gen/single_sh_sch.py.\n"
            "Edit those and rerun build.py rather than editing these sheets. See hardware/README.md.",
            size=1.5)


# ---------------------------------------------------------------------------

def write(sch, out_dir):
    """Draw, check and write the sheets plus their library tables; returns single-pin nets."""
    draw(sch).write(out_dir)
    write_symbol_lib()
    with open(os.path.join(out_dir, "fp-lib-table"), "w") as f:
        f.write(FP_LIB_TABLE)
    with open(os.path.join(out_dir, "sym-lib-table"), "w") as f:
        f.write('(sym_lib_table\n  (version 7)\n  (lib (name "stack")(type "KiCad")'
                '(uri "${KIPRJMOD}/../lib/stack.kicad_sym")(options "")(descr "Project symbols"))\n)\n')
    pins = {}
    for p in sch.parts:
        for n in p.nets.values():
            pins[n] = pins.get(n, 0) + 1
    return sorted(n for n, k in pins.items() if k == 1)


def draw(sch):
    """Build the Design for build.single('single_sh', ...)'s Schematic."""
    d = schdraw.Design(sch.name, sch.title, sch.parts)
    d.root.title = "Single board (single_sh) - overview"   # sch.title is too long for the title block
    d.extra_symdefs[AT8833] = at8833_symbol()
    pwr = d.sheet("Power", "power.kicad_sch", "Single board (single_sh) - power and USB")
    mcu = d.sheet("MCU", "mcu.kicad_sch", "Single board (single_sh) - RP2354B")
    drv = d.sheet("Motor drivers", "drivers.kicad_sch", "Single board (single_sh) - motor drivers")
    io = d.sheet("Connectors and UI", "io.kicad_sch", "Single board (single_sh) - connectors and user interface")
    draw_power(pwr)
    draw_mcu(mcu)
    draw_drivers(drv)
    draw_io(io)
    draw_root(d.root, pwr, mcu, drv, io)
    return d


if __name__ == "__main__":
    import build
    d = draw(build.single("single_sh", "", build.sh_motor_pair, "4.7k"))
    errors, overlaps = d.check(), d.lint()
    print("\n".join([f"error: {e}" for e in errors] + [f"overlap? {o}" for o in overlaps]) or "clean")
