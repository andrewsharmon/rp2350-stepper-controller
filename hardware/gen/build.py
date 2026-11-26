"""Generate the three stack schematics and the single-board versions.

    python3 build.py        # writes ../controller, ../driver, ../io_xh, ../io_sh,
                            # and the one-board versions ../single_xh, ../single_sh

single_sh is drawn as a wired, multi-sheet schematic by single_sh_sch.py; the
rest are one sheet of parts with a net label on every pin.

Passives get their LCSC / MPN from PASSIVES below; the LCSC field is only set
on parts whose number was checked (JLCPCB parts library, 2026-10-07).
"""

import os

import single_sh_sch
from schgen import Schematic

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.dirname(HERE)

R0402 = "Resistor_SMD:R_0402_1005Metric"
R0603 = "Resistor_SMD:R_0603_1608Metric"
C0402 = "Capacitor_SMD:C_0402_1005Metric"
C0603 = "Capacitor_SMD:C_0603_1608Metric"
C0805 = "Capacitor_SMD:C_0805_2012Metric"
SOT23_5 = "Package_TO_SOT_SMD:SOT-23-5"
SMA = "Diode_SMD:D_SMA"
JP_BRIDGED = "Jumper:SolderJumper-2_P1.3mm_Bridged_RoundedPad1.0x1.5mm"
JP_OPEN = "Jumper:SolderJumper-2_P1.3mm_Open_RoundedPad1.0x1.5mm"
# Hirose DF40, 0.4 mm pitch, 40 pins, 2.0 mm stack: header on the underside of
# the upper board mates the socket on top of the lower board. The four corner
# contacts (pins 1, 2, 39, 40) are metal fittings per Hirose (soldered for
# strength, not for signal or power), so they are left unconnected.
BTB_HEADER = ("stack:Hirose_DF40C-40DP_Header", "DF40C-40DP-0.4V(51)", "C424643")
BTB_SOCKET = ("stack:Hirose_DF40C-2.0-40DS_Socket", "DF40C(2.0)-40DS-0.4V(51)", "C597934")
BTB_FIT = [None, None]  # one corner pair (both rows) at each end: no connect
TP = "TestPoint:TestPoint_Pad_D1.0mm"

MOTORS = 10
IN_PINS = ["AIN1", "AIN2", "BIN1", "BIN2"]
OUT_PINS = ["AOUT1", "AOUT2", "BOUT1", "BOUT2"]


def motor_nets(m, kind):
    names = IN_PINS if kind == "in" else OUT_PINS
    return [f"M{m}_{n}" for n in names]


# Pin order follows the MCU's pins so traces fan in without crossing. With the
# connectors placed pin 1 at the top: A runs down the controller's left edge
# (power by the power path, then M1 from the MCU's top-left corner, M2-M5 down
# its left side, M6-M7 round its bottom-left), B down the right edge
# (utilities high on the MCU's right side, then M10, M9, M8 round the
# bottom-right). M6/M7 sit on A so they don't cross the crystal/SWD pins.

def btb_a(kind, top):
    """40-pin connector A: motors 1-7, 3V3, 4x V5, 3x GND."""
    nets = ["V5", "V5", "V5", "V5", "+3V3" if top else "GND", "GND"]
    nets += motor_nets(1, kind) + motor_nets(2, kind) + ["GND"]
    nets += motor_nets(3, kind) + motor_nets(4, kind) + ["GND"]
    nets += motor_nets(5, kind) + motor_nets(6, kind) + motor_nets(7, kind)
    nets = BTB_FIT + nets + BTB_FIT
    assert len(nets) == 40
    return nets


def btb_b(kind, top):
    """40-pin connector B: utilities, motors 10, 9, 8, 9x V5, 11x GND."""
    nets = ["V5", "V5", "V5", "V5", "V5", "GND", "GND"]
    nets += ["LED_DATA", "LADDER", "BOARD_ID", "VMOT_SENSE" if top else "GND", "GND"]
    # The MCU's right side is numbered bottom-to-top, so each group runs reversed.
    nets += motor_nets(10, kind)[::-1] + motor_nets(9, kind)[::-1] + ["GND"] + motor_nets(8, kind)[::-1]
    nets += ["GND", "V5", "V5", "V5", "V5", "GND", "GND", "GND", "GND", "GND", "GND"]
    nets = BTB_FIT + nets + BTB_FIT
    assert len(nets) == 40
    return nets


def add_btb(sch, ref, nets, part, note):
    fp, mpn, lcsc = part
    value = "BTB 40P " + ("header" if part is BTB_HEADER else "socket")
    sch.add("Connector_Generic:Conn_02x20_Odd_Even", "J", value, fp,
            {str(i + 1): n for i, n in enumerate(nets) if n}, ref=ref, MPN=mpn, LCSC=lcsc, Note=note)


# (value, footprint) -> (LCSC, MPN). JLCPCB basic parts unless marked extended.
# Resistors are UNI-ROYAL 0402WGF thick film, 1 %, 62.5 mW.
PASSIVES = {
    ("100n", C0402): ("C1525", "CL05B104KO5NNNC"),       # 16 V X7R
    ("1u", C0402): ("C52923", "CL05A105KA5NQNC"),        # 25 V X5R
    ("2.2u", C0402): ("C170151", "CL05A225KO5NQNC"),     # 16 V X5R, extended (basic 2.2u is 6.3 V)
    ("4.7u", C0402): ("C23733", "CL05A475MP5NRNC"),      # 10 V X5R
    ("15p", C0402): ("C1548", "0402CG150J500NT"),        # 50 V C0G
    ("10u", C0805): ("C15850", "CL21A106KAYNNNE"),       # 25 V X5R
    ("1", R0603): ("C22936", "0603WAF100KT5E"),          # UniOhm 1 %, 100 mW, basic (no basic 0.82R)
    ("22", R0402): ("C25092", "0402WGF220JTCE"),         # mod_mcu USB series (27R 0402 is extended)
    ("27", R0402): ("C25100", "0402WGF270JTCE"),         # extended
    ("33", R0402): ("C25105", "0402WGF330JTCE"),
    ("1k", R0402): ("C11702", "0402WGF1001TCE"),
    ("2.2k", R0402): ("C25879", "0402WGF2201TCE"),       # mod_motor board ID
    ("3.3k 1%", R0402): ("C25890", "0402WGF3301TCE"),
    ("4.7k", R0402): ("C25900", "0402WGF4701TCE"),
    ("5.1k", R0402): ("C25905", "0402WGF5101TCE"),
    ("10k", R0402): ("C25744", "0402WGF1002TCE"),
    ("10k 1%", R0402): ("C25744", "0402WGF1002TCE"),
    ("20k 1%", R0402): ("C25765", "0402WGF2002TCE"),
    ("39k 1%", R0402): ("C25783", "0402WGF3902TCE"),     # extended
    ("47k", R0402): ("C25792", "0402WGF4702TCE"),
    ("100k", R0402): ("C25741", "0402WGF1003TCE"),
    # single_sh_proto: every passive 0603, all JLCPCB basic (checked 2026-10-10).
    ("100n", C0603): ("C14663", "CC0603KRX7R9BB104"),    # 50 V X7R
    ("1u", C0603): ("C15849", "CL10A105KB8NNNC"),        # 50 V X5R
    ("2.2u", C0603): ("C23630", "CL10A225KO8NNNC"),      # 16 V X5R
    ("4.7u", C0603): ("C19666", "CL10A475KO8NNNC"),      # 16 V X5R
    ("15p", C0603): ("C1644", "CL10C150JB8NNNC"),        # 50 V C0G
    ("10u", C0603): ("C19702", "CL10A106KP8NNNC"),       # 10 V X5R
    ("22", R0603): ("C23345", "0603WAF220JT5E"),         # USB series (27R 0603 is extended)
    ("33", R0603): ("C23140", "0603WAF330JT5E"),
    ("1k", R0603): ("C21190", "0603WAF1001T5E"),
    ("3.3k 1%", R0603): ("C22978", "0603WAF3301T5E"),
    ("4.7k", R0603): ("C23162", "0603WAF4701T5E"),
    ("5.1k", R0603): ("C23186", "0603WAF5101T5E"),
    ("10k", R0603): ("C25804", "0603WAF1002T5E"),
    ("10k 1%", R0603): ("C25804", "0603WAF1002T5E"),
    ("20k 1%", R0603): ("C4184", "0603WAF2002T5E"),
    ("39k 1%", R0603): ("C23153", "0603WAF3902T5E"),
    ("47k", R0603): ("C25819", "0603WAF4702T5E"),
    ("100k", R0603): ("C25803", "0603WAF1003T5E"),
}

# Footprint / value substitutions applied to every passive; single(proto=True)
# sets them for the all-0603 single_sh_proto board.
SUBST_FP = {}
SUBST_VALUE = {}
PROTO_FP = {R0402: R0603, C0402: C0603, C0805: C0603}
PROTO_VALUE = {"27": "22"}


def passive(sch, lib_id, prefix, value, fp, nets, kw):
    fp = SUBST_FP.get(fp, fp)
    value = SUBST_VALUE.get(value, value)
    if "LCSC" not in kw and (value, fp) in PASSIVES:
        kw["LCSC"], kw["MPN"] = PASSIVES[(value, fp)]
    return sch.add(lib_id, prefix, value, fp, nets, **kw)


def R(sch, value, a, b, fp=R0402, **kw):
    return passive(sch, "Device:R", "R", value, fp, {"1": a, "2": b}, kw)


def C(sch, value, a, b, fp=C0402, **kw):
    return passive(sch, "Device:C", "C", value, fp, {"1": a, "2": b}, kw)


def flags(sch, *nets):
    for n in nets:
        sch.add("power:PWR_FLAG", "#FLG", "PWR_FLAG", "", {"1": n})


# ---------------------------------------------------------------------------
# Tier 1: controller
# ---------------------------------------------------------------------------

def controller(s=None):
    stacked = s is None
    if stacked:
        s = Schematic("controller", "Stepper controller - tier 1 (RP2354B)")

    mcu = {}
    for g in range(40):
        mcu[f"GPIO{g}"] = motor_nets(g // 4 + 1, "in")[g % 4]
    for g, net in zip(range(40, 48), ["CC1", "CC2", "VMOT_SENSE", "BOARD_ID", "LADDER", "LED_DIN", "SDA", "SCL"]):
        mcu[f"GPIO{g}/ADC{g - 40}"] = net
    mcu.update({
        "IOVDD": "+3V3", "QSPI_IOVDD": "+3V3", "USB_OTP_VDD": "+3V3", "ADC_AVDD": "+3V3", "VREG_VIN": "+3V3",
        "VREG_AVDD": "VREG_AVDD", "DVDD": "+1V1", "VREG_FB": "+1V1", "VREG_LX": "VREG_LX",
        "VREG_PGND": "GND", "GND": "GND",
        "XIN": "XIN", "XOUT": "XOUT", "USB_DP": "USB_DP_MCU", "USB_DM": "USB_DM_MCU",
        "RUN": "RUN", "SWCLK": "SWCLK", "SWDIO": "SWDIO", "~{QSPI_SS}": "QSPI_SS",
    })
    s.add("MCU_RaspberryPi:RP2354B", "U", "RP2354B", "Package_DFN_QFN:QFN-80-1EP_10x10mm_P0.4mm_EP3.4x3.4mm",
          mcu, LCSC="C39843328", MPN="RP2354B",
          Note="2 MB in-package flash; leave QSPI_SD0-3/SCLK unconnected")

    # Supply decoupling (RP2350 hardware design guide).
    for _ in range(9):
        C(s, "100n", "+3V3", "GND")
    C(s, "4.7u", "+3V3", "GND")                      # VREG_VIN
    for _ in range(3):
        C(s, "100n", "+1V1", "GND")
    C(s, "4.7u", "+1V1", "GND")                      # core regulator output
    s.add("Device:L", "L", "3.3u", "Inductor_SMD:L_Cenker_CKCS201610", {"1": "VREG_LX", "2": "+1V1"},
          LCSC="C42411119", MPN="AOTA-B201610S3R3-101-T", Note="Abracon part per RP2350 design guide; 2016 size")
    R(s, "33", "+3V3", "VREG_AVDD")
    C(s, "4.7u", "VREG_AVDD", "GND")

    # Crystal.
    # The design guide's 15 pF / 1k values are tuned for this crystal (10 pF load, ESR <= 50 ohm).
    s.add("Device:Crystal_GND24", "Y", "12MHz", "Crystal:Crystal_SMD_3225-4Pin_3.2x2.5mm",
          {"1": "XIN", "3": "XTAL_OUT", "2": "GND", "4": "GND"}, LCSC="C20625731", MPN="ABM8-272-T3",
          Note="Abracon, per RP2350 design guide; other crystals need retuning and testing")
    C(s, "15p", "XIN", "GND")
    C(s, "15p", "XTAL_OUT", "GND")
    R(s, "1k", "XOUT", "XTAL_OUT")

    # USB-C, ESD, series resistors, CC sense.
    s.add("Connector:USB_C_Receptacle_USB2.0_16P", "J", "USB-C", "Connector_USB:USB_C_Receptacle_HRO_TYPE-C-31-M-12",
          {"VBUS": "VBUS", "GND": "GND", "CC1": "CC1", "CC2": "CC2", "D+": "USB_DP", "D-": "USB_DM", "SHIELD": "GND"},
          ref="J1", LCSC="C165948", MPN="TYPE-C-31-M-12", Note="THT shell legs; enclosure supports receptacle")
    # 2-line ESD, SOT-553: small enough to sit on the D+/D- pair by the receptacle.
    s.add("Power_Protection:TPD2E2U06DRL", "U", "TPD2E2U06", "Package_TO_SOT_SMD:SOT-553",
          {"3": "USB_DP", "5": "USB_DM", "4": "GND"},
          LCSC="C1972959", MPN="TPD2E2U06DRLR", Note="TI; 1.5 pF; pins 1-2 NC")
    R(s, "27", "USB_DP", "USB_DP_MCU")
    R(s, "27", "USB_DM", "USB_DM_MCU")
    R(s, "5.1k", "CC1", "GND")
    R(s, "5.1k", "CC2", "GND")

    # Power path: VBUS -> PTC -> Schottky -> V5_IN -> soft-start switch -> V5 bus.
    # The 3.3 V LDO runs from V5_IN, ahead of the switch.
    s.add("Device:Polyfuse", "F", "1.5A hold", "Fuse:Fuse_1812_4532Metric", {"1": "VBUS", "2": "VBUS_F"},
          MPN="SMD1812P150TF/8", LCSC="C209721", Note="PTTC; 1.5 A hold / 3 A trip, 8 V; alt C21002")
    s.add("Device:D_Schottky", "D", "SS54", SMA, {"2": "VBUS_F", "1": "V5_IN"},
          LCSC="C22452", MPN="SS54", Note="MDD; Vf 0.55 V @ 5 A")
    C(s, "10u", "V5_IN", "GND", C0805)
    s.add("Regulator_Linear:ME6211C33M5", "U", "ME6211C33M5G", SOT23_5,
          {"1": "V5_IN", "3": "V5_IN", "2": "GND", "5": "+3V3"}, LCSC="C82942", MPN="ME6211C33M5G-N")
    C(s, "1u", "V5_IN", "GND")

    # Soft-start: at plug-in the gate cap holds Vgs at 0, then the gate decays
    # through 100k (tau 10 ms), so the ~70 uF on the V5 bus charges over a few
    # ms (~0.1 A) instead of as a USB inrush spike. External 5 V on tier 3
    # back-feeds V5_IN through the body diode, then the channel once Vgs builds.
    s.add("Transistor_FET:AO3401A", "Q", "AO3401A", "Package_TO_SOT_SMD:SOT-23",
          {"S": "V5_IN", "D": "V5", "G": "SS_GATE"}, LCSC="C15127", MPN="AO3401A",
          Note="AOS; 60 mohm @ 4.5 V, Vth ~0.9 V")
    C(s, "100n", "V5_IN", "SS_GATE", Note="soft-start timing with the 100k")
    R(s, "100k", "SS_GATE", "GND", Note="soft-start timing with the 100n")
    C(s, "1u", "+3V3", "GND")

    # Qwiic x2: optional supply out via jumper. No power in: the ME6211 conducts
    # from VOUT back to VIN, so a Qwiic-fed 3V3 would back-power the V5 bus.
    for _ in range(2):
        s.add("Connector_Generic:Conn_01x04", "J", "Qwiic", "Connector_JST:JST_SH_BM04B-SRSS-TB_1x04-1MP_P1.00mm_Vertical",
              {"1": "GND", "2": "QWIIC_3V3", "3": "SDA", "4": "SCL"}, LCSC="C160390", MPN="BM04B-SRSS-TB")
    s.add("Jumper:SolderJumper_2_Open", "JP", "QWIIC_SUPPLY", JP_OPEN, {"1": "+3V3", "2": "QWIIC_3V3"},
          Note="bridge to power downstream Qwiic devices", in_bom=False)
    R(s, "4.7k", "SDA", "I2C_PU")
    R(s, "4.7k", "SCL", "I2C_PU")
    s.add("Jumper:SolderJumper_2_Bridged", "JP", "I2C_PU", JP_BRIDGED, {"1": "I2C_PU", "2": "+3V3"},
          Note="cut to remove I2C pull-ups", in_bom=False)

    # Analog ladder: e-stop load lives on tier 3; fault pulls to 0 V.
    R(s, "10k 1%", "+3V3", "LADDER")
    C(s, "100n", "LADDER", "GND")
    s.add("Switch:SW_Push", "SW", "BTN1", "Button_Switch_SMD:SW_SPST_TS-1088-xR020",
          {"1": "LADDER", "2": "BTN1_N"}, LCSC="C720477", MPN="TS-1088-AR02016",
          Note="XUNPU 3.9x3.0 mm 2-pad; small so the BTB fan-out routes")
    R(s, "10k 1%", "BTN1_N", "GND")
    s.add("Switch:SW_Push", "SW", "BTN2", "Button_Switch_SMD:SW_SPST_TS-1088-xR020",
          {"1": "LADDER", "2": "BTN2_N"}, LCSC="C720477", MPN="TS-1088-AR02016",
          Note="XUNPU 3.9x3.0 mm 2-pad; small so the BTB fan-out routes")
    R(s, "3.3k 1%", "BTN2_N", "GND")

    # BOOTSEL (readable at runtime via QSPI_SS). The single boards use the same
    # small button as BTN1/BTN2; the stacked controller keeps the TS-1187A it
    # was laid out with.
    if stacked:
        s.add("Switch:SW_Push", "SW", "BOOTSEL", "Button_Switch_SMD:SW_Push_1P1T_XKB_TS-1187A",
              {"1": "QSPI_SS", "2": "BOOT_R"}, LCSC="C318884", MPN="TS-1187A-B-A-B")
    else:
        s.add("Switch:SW_Push", "SW", "BOOTSEL", "Button_Switch_SMD:SW_SPST_TS-1088-xR020",
              {"1": "QSPI_SS", "2": "BOOT_R"}, LCSC="C720477", MPN="TS-1088-AR02016",
              Note="XUNPU 3.9x3.0 mm 2-pad, same as BTN1/BTN2")
    R(s, "1k", "BOOT_R", "GND")

    # Board-ID pull-up: tier 3 provides the resistor to GND (none fitted -> 3.3 V).
    R(s, "10k 1%", "+3V3", "BOARD_ID")

    # Status LED, first in the chain; DOUT continues down the stack. Powered from V5
    # (>= 3.7 V needed; a diode drop would leave ~3.5 V on a 4.75 V USB supply), so
    # the 3.3 V data goes through an AHCT buffer to meet VIH = 0.7 VDD.
    s.add("74xGxx:74AHCT1G125", "U", "74AHCT1G125", SOT23_5,
          {"1": "GND", "2": "LED_DIN", "3": "GND", "4": "LED_DIN_5V", "5": "V5"}, LCSC="C7484", MPN="SN74AHCT1G125DBVR")
    C(s, "100n", "V5", "GND")
    s.add("LED:WS2812B-2020", "D", "XL-2020RGBC-WS2812B", "LED_SMD:LED_WS2812B-2020_PLCC4_2.0x2.0mm",
          {"VDD": "V5", "VSS": "GND", "DIN": "LED_DIN_5V", "DOUT": "LED_DATA"}, LCSC="C5349955",
          MPN="XL-2020RGBC-WS2812B",
          Note="XINGLIGHT; WS2812B-2020 pinout and footprint; Worldsemi C965555 was out of stock at JLCPCB")
    C(s, "100n", "V5", "GND")

    # Debug pads.
    for net in ["SWCLK", "SWDIO", "RUN", "GND", "+3V3"]:
        s.add("Connector:TestPoint", "TP", net, TP, {"1": net}, in_bom=False)

    if stacked:
        add_btb(s, "J10", btb_a("in", True), BTB_HEADER, "connector A, board underside")
        add_btb(s, "J11", btb_b("in", True), BTB_HEADER, "connector B, board underside; offset placement keys stack")
        flags(s, "GND", "V5", "V5_IN", "+1V1", "VREG_AVDD", "VBUS")
    return s


# ---------------------------------------------------------------------------
# Tier 2: universal driver
# ---------------------------------------------------------------------------

# mod_driver: three 10 uF per row of five drivers, each in the gap between two
# chips (60 uF nominal, the proto had ~52 uF on V5).
MOD_VM_CAPS = {1, 3, 5, 6, 8, 10}


def driver(s=None, shared_vm=False):
    """10x AT8833CQ. shared_vm (mod_driver): shared 10 uF 0603 VM caps between
    neighbouring chips replace the per-chip 2.2 uF and the separate V5 bulk."""
    stacked = s is None
    if stacked:
        s = Schematic("driver", "Stepper controller - tier 2 (10x AT8833CQ)")
    for m in range(1, MOTORS + 1):
        u = f"U{m}"
        nets = dict(zip(IN_PINS, motor_nets(m, "in")))
        nets.update(zip(OUT_PINS, motor_nets(m, "out")))
        nets.update({"VM": "V5", "GND": "GND", "VINT": f"{u}_VINT", "VCP": f"{u}_VCP",
                     "AISEN": f"{u}_AISEN", "BISEN": f"{u}_BISEN",
                     "~{SLEEP}": "NSLEEP", "~{FAULT}": "LADDER"})
        s.add("Driver_Motor:DRV8833RTY", "U", "AT8833CQ", "Package_DFN_QFN:QFN-16-1EP_4x4mm_P0.65mm_EP2.1x2.1mm",
              nets, ref=u, LCSC="C5120769", MPN="AT8833CQ",
              Note="DRV8833RTY pinout; 2nd source JSMSEMI DRV8833RTYR-JSM C55566425")
        R(s, "1", f"{u}_AISEN", "GND", fp=R0603, Note="chops at 0.16-0.24 A (VTRIP 160-240 mV); Kelvin return")
        R(s, "1", f"{u}_BISEN", "GND", fp=R0603, Note="chops at 0.16-0.24 A (VTRIP 160-240 mV); Kelvin return")
        C(s, "1u", f"{u}_VINT", "GND")
        flags(s, f"{u}_VINT")  # internal regulator output, typed power_in in the symbol
        C(s, "100n", f"{u}_VCP", "V5")
        if not shared_vm:
            C(s, "2.2u", "V5", "GND")
        elif m in MOD_VM_CAPS:
            C(s, "10u", "V5", "GND", C0603, Note="VM, shared by the neighbouring drivers in the row; also the V5 bulk")

    if not shared_vm:
        for _ in range(3):
            C(s, "10u", "V5", "GND", C0805)
    R(s, "47k", "+3V3", "NSLEEP")
    # The 39k 0402 is extended; mod_driver uses the basic 0603.
    R(s, "39k 1%", "V5", "VMOT_SENSE", fp=R0603 if shared_vm else R0402)
    R(s, "10k 1%", "VMOT_SENSE", "GND")
    C(s, "100n", "VMOT_SENSE", "GND")

    if stacked:
        add_btb(s, "J1", btb_a("in", True), BTB_SOCKET, "from controller A, top side")
        add_btb(s, "J2", btb_b("in", True), BTB_SOCKET, "from controller B, top side")
        add_btb(s, "J3", btb_a("out", False), BTB_HEADER, "to tier 3 A, board underside")
        add_btb(s, "J4", btb_b("out", False), BTB_HEADER, "to tier 3 B, board underside")
        flags(s, "GND", "V5", "+3V3")
    return s


# ---------------------------------------------------------------------------
# Tier 3: connector / IO variants
# ---------------------------------------------------------------------------

def io_board(name, title, motor_conn, board_id_r, s=None):
    stacked = s is None
    if stacked:
        s = Schematic(name, title)
        add_btb(s, "J1", btb_a("out", False), BTB_SOCKET, "from driver A, top side")
        add_btb(s, "J2", btb_b("out", False), BTB_SOCKET, "from driver B, top side")

    for m in range(1, MOTORS + 1):
        motor_conn(s, m)

    # External 5 V input onto the V5 bus.
    s.add("Connector_Generic:Conn_01x02", "J", "5V IN", "Connector_JST:JST_PH_B2B-PH-K_1x02_P2.00mm_Vertical",
          {"1": "+5V_EXT", "2": "GND"}, LCSC="C131337", MPN="B2B-PH-K-S")
    s.add("Device:D_Schottky", "D", "SS54", SMA, {"2": "+5V_EXT", "1": "V5"}, LCSC="C22452", MPN="SS54",
          Note="MDD; Vf 0.55 V @ 5 A")
    C(s, "1u", "+5V_EXT", "GND")

    # LED strip output: buffered data, selectable strip power.
    s.add("74xGxx:74AHCT1G125", "U", "74AHCT1G125", SOT23_5,
          {"1": "GND", "2": "LED_DATA", "3": "GND", "4": "LED_BUF", "5": "V5"}, LCSC="C7484", MPN="SN74AHCT1G125DBVR")
    C(s, "100n", "V5", "GND")
    R(s, "33", "LED_BUF", "STRIP_DIN")
    s.add("Device:Polyfuse", "F", "1.5A hold", "Fuse:Fuse_1812_4532Metric", {"1": "V5", "2": "STRIP_V5_FUSED"},
          MPN="SMD1812P150TF/8", LCSC="C209721", Note="PTTC; 1.5 A hold / 3 A trip, 8 V")
    s.add("Jumper:SolderJumper_2_Bridged", "JP", "STRIP_SHARED", JP_BRIDGED, {"1": "STRIP_V5_FUSED", "2": "STRIP_5V"},
          Note="cut when strip has its own supply on LED PWR", in_bom=False)
    s.add("Connector_Generic:Conn_01x03", "J", "LED STRIP", "Connector_JST:JST_XH_B3B-XH-A_1x03_P2.50mm_Vertical",
          {"1": "STRIP_5V", "2": "STRIP_DIN", "3": "GND"}, LCSC="C144394", MPN="B3B-XH-A")
    s.add("Connector_Generic:Conn_01x02", "J", "LED PWR", "Connector_JST:JST_PH_B2B-PH-K_1x02_P2.00mm_Vertical",
          {"1": "STRIP_5V", "2": "GND"}, LCSC="C131337", MPN="B2B-PH-K-S", Note="separate strip supply (optional)")
    C(s, "10u", "STRIP_5V", "GND", C0805)

    # E-stop (NC) with the 20k idle load; jumper bypasses when no e-stop fitted.
    s.add("Connector_Generic:Conn_01x02", "J", "E-STOP", "Connector_JST:JST_PH_B2B-PH-K_1x02_P2.00mm_Vertical",
          {"1": "LADDER", "2": "ESTOP_RET"}, LCSC="C131337", MPN="B2B-PH-K-S")
    s.add("Jumper:SolderJumper_2_Bridged", "JP", "NO_ESTOP", JP_BRIDGED, {"1": "LADDER", "2": "ESTOP_RET"},
          Note="MUST cut when an NC e-stop is fitted, or the e-stop does nothing; silkscreen this", in_bom=False)
    R(s, "20k 1%", "ESTOP_RET", "GND")

    # External panel buttons in parallel with the onboard ones.
    s.add("Connector_Generic:Conn_01x03", "J", "PANEL BTN", "Connector_JST:JST_PH_B3B-PH-K_1x03_P2.00mm_Vertical",
          {"1": "LADDER", "2": "PB1", "3": "PB2"}, LCSC="C131339", MPN="B3B-PH-K-S",
          Note="button from pin1 to pin2/pin3")
    R(s, "10k 1%", "PB1", "GND")
    R(s, "3.3k 1%", "PB2", "GND")

    # Board ID to GND (pull-up on controller).
    R(s, board_id_r, "BOARD_ID", "GND", Note="board-ID level; 5% ok, see hardware/README.md")

    if stacked:
        flags(s, "GND", "V5", "+5V_EXT", "STRIP_5V")
    return s


def xh_motor(s, m):
    # Stock 28BYJ-48 plug (JST-XH 5P housing) after the bipolar mod: 1 blue, 2 pink, 3 yellow, 4 orange, 5 red (unused).
    a1, a2, b1, b2 = motor_nets(m, "out")
    s.add("Connector_Generic:Conn_01x05", "J", f"M{m}", "Connector_JST:JST_XH_B5B-XH-A_1x05_P2.50mm_Vertical",
          {"1": a1, "3": a2, "2": b1, "4": b2}, ref=f"J{10 + m}", LCSC="C157991", MPN="B5B-XH-A")


def sh_motor_pair(s, m):
    # Dual 8 mm micro steppers terminated together in one JST-SH 8-pin housing:
    # pins 1-4 = first motor A1 A2 B1 B2, pins 5-8 = second motor.
    if m % 2 == 0:
        return
    nets = motor_nets(m, "out") + motor_nets(m + 1, "out")
    s.add("Connector_Generic:Conn_01x08", "J", f"M{m}+M{m + 1}",
          "Connector_JST:JST_SH_BM08B-SRSS-TB_1x08-1MP_P1.00mm_Vertical",
          {str(i + 1): n for i, n in enumerate(nets)}, ref=f"J{10 + (m + 1) // 2}",
          LCSC="C160394", MPN="BM08B-SRSS-TB")


def sh_motor_pair_nested(s, m, conn=("Connector_JST:JST_SH_BM08B-SRSS-TB_1x08-1MP_P1.00mm_Vertical",
                                     "C160394", "BM08B-SRSS-TB")):
    # single_sh_proto: the corrected dual-motor cable pinout, where one motor's
    # coils take the outer pins and the other's the inner ones. Coil and
    # polarity per motor are chosen for the layout (the two drivers' outputs
    # reach the pins without crossing); firmware fixes them per axis with
    # CONFIG_FLAG_SWAP_COILS / CONFIG_FLAG_REVERSE (firmware/src/config.h).
    #   outer motor = M(m+1), the pair's left driver: 1 A1, 2 A2, 7 B1, 8 B2
    #   inner motor = M(m), the right driver: 3 B1, 4 B2, 5 A2, 6 A1
    #     (coils swapped and one reversed relative to the cable's A/B)
    if m % 2 == 0:
        return
    a1, a2, b1, b2 = motor_nets(m, "out")
    c1, c2, d1, d2 = motor_nets(m + 1, "out")
    nets = [c1, c2, b1, b2, a2, a1, d1, d2]
    fp, lcsc, mpn = conn
    s.add("Connector_Generic:Conn_01x08", "J", f"M{m + 1}+M{m}", fp,
          {str(i + 1): n for i, n in enumerate(nets)}, ref=f"J{10 + (m + 1) // 2}",
          LCSC=lcsc, MPN=mpn)


def gh_motor_pair_nested(s, m):
    # mod_motor: the same nested dual-motor pinout one pitch up, JST GH 1.25 mm,
    # vertical (the plugs face down out of the stack's bottom layer).
    sh_motor_pair_nested(s, m, ("Connector_JST:JST_GH_BM08B-GHS-TBT_1x08-1MP_P1.25mm_Vertical",
                                "C133062", "BM08B-GHS-TBT"))


# ---------------------------------------------------------------------------
# Mod stack: mod_mcu / mod_driver / mod_motor (docs/PLAN_mod_boards.md)
# ---------------------------------------------------------------------------
#
# The mod bus is two 2x20 2.54 mm headers, A and B, on the board's long edges.
# Every layer sees every pin; a layer connects only the pins it uses (the
# stacking headers pass the rest through). Pin 2k-1 (odd) is the inner row,
# toward the board centre, and carries signals; pin 2k (even) is the outer
# row and carries power, so every signal has a power/GND pin beside it.
# Entries are GPIO numbers or rail names; each board maps a GPIO to its own
# net name. A runs GPIO0-19 (M1-M5), B GPIO20-39 (M6-M10) on the inner row and
# the utilities GPIO40-47 on the outer row.
#
# Seen from the top, the pin numbers run the way the RP2354B's pads do
# (counter-clockwise), so mod_mcu's fan-out doesn't cross: A along the
# board's -y edge with pin 1 at the +x end, B along the +y edge with pin 1 at
# the -x end.

MOD_BUS_A_OUTER = ["V5", "V5", "GND", "+3V3", "GND", "RUN", "GND", "SWCLK", "GND", "SWDIO",
                   "GND", "GND", "GND", "+3V3", "GND", "GND", "GND", "V5", "GND", "V5"]
# LED_DOUT is on B: the status LED sits by GPIO45, on the MCU's B-facing side.
MOD_BUS_B_OUTER = ["V5", "V5", "GND", 40, 41, "GND", 42, 43, "GND", 44,
                   45, "GND", 46, 47, "LED_DOUT", "+3V3", "GND", "GND", "GND", "V5"]
# 2x20 2.54 mm: stacking female with 12.3 mm tails (mod_driver, all three
# headers, so they mate mod_motor at one height), plain male (mod_mcu
# underside), plain female (mod_motor top).
MOD_STACK = ("Connector_PinSocket_2.54mm:PinSocket_2x20_P2.54mm_Vertical", "2.54-2*20PFemale longPC104", "C35165")
MOD_MALE = ("Connector_PinHeader_2.54mm:PinHeader_2x20_P2.54mm_Vertical", "PZ254-2-20-Z-8.5", "C3012260")
MOD_FEMALE = ("Connector_PinSocket_2.54mm:PinSocket_2x20_P2.54mm_Vertical", "2.54-2*20P", "C5124634")


def mod_bus(header, gpio_net):
    """40 nets for bus header 'A' or 'B'; gpio_net(n) names GPIO n on this board (None: not used)."""
    inner = range(0, 20) if header == "A" else range(20, 40)
    outer = MOD_BUS_A_OUTER if header == "A" else MOD_BUS_B_OUTER
    assert len(outer) == 20 and outer.count("GND") == (11 if header == "A" else 7)
    nets = []
    for g, o in zip(inner, outer):
        # Rails are on every board; the named signals (RUN, SWD, LED_DOUT) map like GPIOs.
        nets += [gpio_net(g), o if o in ("V5", "GND", "+3V3") else gpio_net(o)]
    return nets


def mod_motor_out():
    """Motor-output header (mod_driver -> mod_motor), down the board's middle.

    Position p = 0..19 from the pin-1 (west) end; pin 2p+1 (odd row) faces the
    M1-M5 driver row (bus A side), pin 2p+2 the M6-M10 row. The rows run in
    opposite directions (they follow the bus, see mod_bus): M5..M1 west to
    east on the odd row, M6..M10 on the even row. Each motor takes four
    positions in its driver's output-pin order seen from the header (rot 90:
    AOUT1 AOUT2 BOUT2 BOUT1 west to east; rot 270: the reverse), so nothing
    crosses.
    """
    nets = []
    for p in range(20):
        a1, a2, b1, b2 = motor_nets(5 - p // 4, "out")
        nets.append([a1, a2, b2, b1][p % 4])
        a1, a2, b1, b2 = motor_nets(6 + p // 4, "out")
        nets.append([b1, b2, a2, a1][p % 4])
    return nets


def add_mod_header(sch, ref, nets, part, note):
    fp, mpn, lcsc = part
    value = {MOD_STACK: "2x20 stacking", MOD_MALE: "2x20 male", MOD_FEMALE: "2x20 female"}[part]
    sch.add("Connector_Generic:Conn_02x20_Odd_Even", "J", value, fp,
            {str(i + 1): n for i, n in enumerate(nets) if n}, ref=ref, MPN=mpn, LCSC=lcsc, Note=note)


def driver_gpio(g):
    """GPIO -> net on mod_driver: motor inputs, VMOT_SENSE, LADDER; the rest pass through.

    Motor m's GPIOs 4(m-1)..+3 go to BIN1 BIN2 AIN2 AIN1, not AIN1 AIN2 BIN1 BIN2.
    Seen from the top the RP2354B's pads run counter-clockwise, so on the
    stack bus each motor's four GPIOs arrive mirrored relative to the
    driver's input pins; this order lets them in without crossing. The
    firmware's coil A (GPIO +0/+1) then drives bridge B and its coil B drives
    bridge A reversed: (A, B) = (-sin, cos), the same field turned 90 degrees
    electrical. The motor runs the same way, one full step on; the firmware
    needs no change.
    """
    if isinstance(g, int) and g < 40:
        a1, a2, b1, b2 = motor_nets(g // 4 + 1, "in")
        return [b1, b2, a2, a1][g % 4]
    return {42: "VMOT_SENSE", 44: "LADDER"}.get(g)


def mod_driver():
    s = Schematic("mod_driver", "Mod stack - H-bridge layer (10x AT8833CQ)")
    driver(s, shared_vm=True)
    add_mod_header(s, "J1", mod_bus("A", driver_gpio), MOD_STACK, "mod bus A (GPIO0-19 = M1-M5); passes through")
    add_mod_header(s, "J2", mod_bus("B", driver_gpio), MOD_STACK, "mod bus B (GPIO20-47); passes through")
    add_mod_header(s, "J3", mod_motor_out(), MOD_STACK, "motor outputs to mod_motor")
    for net in ["V5", "VMOT_SENSE", "NSLEEP", "LADDER", "GND"]:
        s.add("Connector:TestPoint", "TP", net, TP, {"1": net}, in_bom=False)
    flags(s, "GND", "V5", "+3V3")
    return s


def mod_ffc():
    """60-pin FFC port on mod_mcu, pin 1 at the north (-y) end of the board's
    west edge. Order is what routes without crossing on mod_mcu's B.Cu from
    the bus header pins: header A's GPIOs run south then west (west-most pin
    nearest the north), header B's north then west (east-most pin nearest the
    north; the outer-row utilities squeeze between the inner pins). A GND
    after every 6 signals; V5_FFC (behind a solder jumper) at the north end."""
    a = list(range(19, -1, -1))
    keys = {20 + p: p for p in range(20)}
    keys.update({g: k + 0.5 for k, g in enumerate(MOD_BUS_B_OUTER) if isinstance(g, int)})
    b = sorted(keys, key=lambda g: -keys[g])
    sig = a + b
    nets = ["V5_FFC", "V5_FFC", "GND"]
    for i, g in enumerate(sig):
        nets.append(f"GPIO{g}")
        if i % 6 == 5 and i < len(sig) - 1:
            nets.append("GND")
    nets += ["+3V3", "GND"]
    assert len(nets) == 60 and nets.count("GND") == 9, (len(nets), nets.count("GND"))
    return nets


def mod_mcu():
    """General-purpose RP2354B layer: controller() minus the stack-specific
    parts, 0402 passives, all 48 GPIO on the mod bus and the FFC. Every
    on-board function on GPIO40-47 sits behind a bridged solder jumper."""
    s = Schematic("mod_mcu", "Mod stack - RP2354B layer (general purpose)")
    gp = lambda g: f"GPIO{g}"
    mcu = {f"GPIO{g}": gp(g) for g in range(40)}
    mcu.update({f"GPIO{g}/ADC{g - 40}": gp(g) for g in range(40, 48)})
    mcu.update({
        "IOVDD": "+3V3", "QSPI_IOVDD": "+3V3", "USB_OTP_VDD": "+3V3", "ADC_AVDD": "+3V3", "VREG_VIN": "+3V3",
        "VREG_AVDD": "VREG_AVDD", "DVDD": "+1V1", "VREG_FB": "+1V1", "VREG_LX": "VREG_LX",
        "VREG_PGND": "GND", "GND": "GND",
        "XIN": "XIN", "XOUT": "XOUT", "USB_DP": "USB_DP_MCU", "USB_DM": "USB_DM_MCU",
        "RUN": "RUN", "SWCLK": "SWCLK", "SWDIO": "SWDIO", "~{QSPI_SS}": "QSPI_SS",
    })
    s.add("MCU_RaspberryPi:RP2354B", "U", "RP2354B", "Package_DFN_QFN:QFN-80-1EP_10x10mm_P0.4mm_EP3.4x3.4mm",
          mcu, LCSC="C39843328", MPN="RP2354B",
          Note="2 MB in-package flash; leave QSPI_SD0-3/SCLK unconnected")

    # Supply decoupling and core regulator (RP2350 hardware design guide).
    for _ in range(9):
        C(s, "100n", "+3V3", "GND")
    C(s, "4.7u", "+3V3", "GND")
    for _ in range(3):
        C(s, "100n", "+1V1", "GND")
    C(s, "4.7u", "+1V1", "GND")
    # The Abracon AOTA (the part the design guide validated) is back in stock
    # at JLCPCB (5130, 2026-11-26), so mod_mcu uses it, not the proto's CMLW.
    s.add("Device:L", "L", "3.3u", "Inductor_SMD:L_Cenker_CKCS201610", {"1": "VREG_LX", "2": "+1V1"},
          LCSC="C42411119", MPN="AOTA-B201610S3R3-101-T",
          Note="Abracon part per RP2350 design guide; dot at the +1V1 end, as on the Pico 2")
    R(s, "33", "+3V3", "VREG_AVDD")
    C(s, "4.7u", "VREG_AVDD", "GND")

    s.add("Device:Crystal_GND24", "Y", "12MHz", "Crystal:Crystal_SMD_3225-4Pin_3.2x2.5mm",
          {"1": "XIN", "3": "XTAL_OUT", "2": "GND", "4": "GND"}, LCSC="C20625731", MPN="ABM8-272-T3",
          Note="Abracon, per RP2350 design guide; other crystals need retuning and testing")
    C(s, "15p", "XIN", "GND")
    C(s, "15p", "XTAL_OUT", "GND")
    R(s, "1k", "XOUT", "XTAL_OUT")

    # USB-C, ESD, series resistors (22R: the 27R 0402 is extended), CC.
    s.add("Connector:USB_C_Receptacle_USB2.0_16P", "J", "USB-C", "Connector_USB:USB_C_Receptacle_HRO_TYPE-C-31-M-12",
          {"VBUS": "VBUS", "GND": "GND", "CC1": "CC1", "CC2": "CC2", "D+": "USB_DP", "D-": "USB_DM", "SHIELD": "GND"},
          ref="J1", LCSC="C165948", MPN="TYPE-C-31-M-12", Note="THT shell legs")
    s.add("Power_Protection:TPD2E2U06DRL", "U", "TPD2E2U06", "Package_TO_SOT_SMD:SOT-553",
          {"3": "USB_DP", "5": "USB_DM", "4": "GND"},
          LCSC="C1972959", MPN="TPD2E2U06DRLR", Note="TI; 1.5 pF; pins 1-2 NC")
    R(s, "22", "USB_DP", "USB_DP_MCU")
    R(s, "22", "USB_DM", "USB_DM_MCU")
    R(s, "5.1k", "CC1", "GND")
    R(s, "5.1k", "CC2", "GND")
    s.add("Jumper:SolderJumper_2_Bridged", "JP", "CC1", JP_BRIDGED, {"1": "CC1", "2": gp(40)},
          Note="cut to free GPIO40 (CC1 sense)", in_bom=False)
    s.add("Jumper:SolderJumper_2_Bridged", "JP", "CC2", JP_BRIDGED, {"1": "CC2", "2": gp(41)},
          Note="cut to free GPIO41 (CC2 sense)", in_bom=False)

    # Power path: VBUS -> PTC -> Schottky -> V5_IN -> soft-start -> V5 bus; LDO from V5_IN.
    s.add("Device:Polyfuse", "F", "1.5A hold", "Fuse:Fuse_1206_3216Metric", {"1": "VBUS", "2": "VBUS_F"},
          MPN="SMD1206B150TFT", LCSC="C269120", Note="PTTC; 1.5 A hold / 3 A trip, 8 V; 1206 for room")
    s.add("Device:D_Schottky", "D", "SS54", SMA, {"2": "VBUS_F", "1": "V5_IN"},
          LCSC="C22452", MPN="SS54", Note="MDD; Vf 0.55 V @ 5 A (every basic >= 2 A Schottky is SMA)")
    C(s, "10u", "V5_IN", "GND", C0603)
    s.add("Regulator_Linear:ME6211C33M5", "U", "ME6211C33M5G", SOT23_5,
          {"1": "V5_IN", "3": "V5_IN", "2": "GND", "5": "+3V3"}, LCSC="C82942", MPN="ME6211C33M5G-N")
    C(s, "1u", "V5_IN", "GND")
    s.add("Transistor_FET:AO3401A", "Q", "AO3401A", "Package_TO_SOT_SMD:SOT-23",
          {"S": "V5_IN", "D": "V5", "G": "SS_GATE"}, LCSC="C15127", MPN="AO3401A",
          Note="AOS; 60 mohm @ 4.5 V, Vth ~0.9 V")
    C(s, "100n", "V5_IN", "SS_GATE", Note="soft-start timing with the 100k")
    R(s, "100k", "SS_GATE", "GND", Note="soft-start timing with the 100n")
    C(s, "1u", "+3V3", "GND")

    # One Qwiic on I2C1 (GPIO46/47; Qwiic devices daisy-chain, and the second
    # one the proto had didn't fit); no power in (the LDO back-feeds V5).
    for _ in range(1):
        s.add("Connector_Generic:Conn_01x04", "J", "Qwiic", "Connector_JST:JST_SH_BM04B-SRSS-TB_1x04-1MP_P1.00mm_Vertical",
              {"1": "GND", "2": "QWIIC_3V3", "3": gp(46), "4": gp(47)}, LCSC="C160390", MPN="BM04B-SRSS-TB")
    s.add("Jumper:SolderJumper_2_Open", "JP", "QWIIC_SUPPLY", JP_OPEN, {"1": "+3V3", "2": "QWIIC_3V3"},
          Note="bridge to power downstream Qwiic devices", in_bom=False)
    R(s, "4.7k", gp(46), "I2C_PU")
    R(s, "4.7k", gp(47), "I2C_PU")
    s.add("Jumper:SolderJumper_2_Bridged", "JP", "I2C_PU", JP_BRIDGED, {"1": "I2C_PU", "2": "+3V3"},
          Note="cut to remove the I2C pull-ups", in_bom=False)

    # Ladder on GPIO44: pull-up, filter and the two buttons; nFAULT (mod_driver)
    # and the e-stop / panel buttons (mod_motor) join on the bus.
    R(s, "10k 1%", "+3V3", "LADDER")
    C(s, "100n", "LADDER", "GND")
    s.add("Switch:SW_Push", "SW", "BTN1", "Button_Switch_SMD:SW_SPST_TS-1088-xR020",
          {"1": "LADDER", "2": "BTN1_N"}, LCSC="C720477", MPN="TS-1088-AR02016", Note="XUNPU 3.9x3.0 mm 2-pad")
    R(s, "10k 1%", "BTN1_N", "GND")
    s.add("Switch:SW_Push", "SW", "BTN2", "Button_Switch_SMD:SW_SPST_TS-1088-xR020",
          {"1": "LADDER", "2": "BTN2_N"}, LCSC="C720477", MPN="TS-1088-AR02016", Note="XUNPU 3.9x3.0 mm 2-pad")
    R(s, "3.3k 1%", "BTN2_N", "GND")
    s.add("Jumper:SolderJumper_2_Bridged", "JP", "LADDER", JP_BRIDGED, {"1": "LADDER", "2": gp(44)},
          Note="cut to free GPIO44 (ladder, buttons)", in_bom=False)

    s.add("Switch:SW_Push", "SW", "BOOTSEL", "Button_Switch_SMD:SW_SPST_TS-1088-xR020",
          {"1": "QSPI_SS", "2": "BOOT_R"}, LCSC="C720477", MPN="TS-1088-AR02016",
          Note="XUNPU 3.9x3.0 mm 2-pad, same as BTN1/BTN2")
    R(s, "1k", "BOOT_R", "GND")

    # Board-ID pull-up on GPIO43 (the ID resistor to GND is on mod_motor).
    R(s, "10k 1%", "+3V3", "ID_PU")
    s.add("Jumper:SolderJumper_2_Bridged", "JP", "BOARD_ID", JP_BRIDGED, {"1": "ID_PU", "2": gp(43)},
          Note="cut to free GPIO43 (board-ID pull-up)", in_bom=False)

    # Status LED on GPIO45, first in the chain; DOUT goes out on the bus as
    # LED_DOUT. 5050 part: every WS2812 is JLCPCB Standard-PCBA only, so on an
    # Economic order it is hand-soldered, as on the proto.
    s.add("Jumper:SolderJumper_2_Bridged", "JP", "LED", JP_BRIDGED, {"1": gp(45), "2": "LED_DIN"},
          Note="cut to free GPIO45 (status LED)", in_bom=False)
    s.add("74xGxx:74AHCT1G125", "U", "74AHCT1G125", SOT23_5,
          {"1": "GND", "2": "LED_DIN", "3": "GND", "4": "LED_DIN_5V", "5": "V5"}, LCSC="C7484", MPN="SN74AHCT1G125DBVR")
    C(s, "100n", "V5", "GND")
    s.add("LED:WS2812B", "D", "XL-5050RGBC-2812B", "LED_SMD:LED_WS2812B_PLCC4_5.0x5.0mm_P3.2mm",
          {"VDD": "V5", "VSS": "GND", "DIN": "LED_DIN_5V", "DOUT": "LED_DOUT"}, LCSC="C2843785",
          MPN="XL-5050RGBC-2812B", Note="XINGLIGHT WS2812B 5050; JLCPCB Standard-PCBA only: hand-solder on Economic")
    C(s, "100n", "V5", "GND")

    # Stack bus (male headers, board underside) and the FFC port.
    bus_net = lambda g: gp(g) if isinstance(g, int) else g
    add_mod_header(s, "J4", mod_bus("A", bus_net), MOD_MALE, "mod bus A, board underside")
    add_mod_header(s, "J5", mod_bus("B", bus_net), MOD_MALE, "mod bus B, board underside")
    s.add("Connector_Generic:Conn_01x60", "J", "FFC 60P", "stack:FFC_0.5mm_60P_FlipTop",
          {str(i + 1): n for i, n in enumerate(mod_ffc())}, ref="J6", LCSC="C22435645", MPN="05A20L60P",
          Note="HanElectricity 0.5 mm flip-top; footprint from Hirose FH12, check against its drawing")
    s.add("Jumper:SolderJumper_2_Open", "JP", "FFC_V5", JP_OPEN, {"1": "V5", "2": "V5_FFC"},
          Note="bridge to put V5 on the FFC (2 conductors, 400 mA each; unfused)", in_bom=False)

    for net in ["SWCLK", "SWDIO", "RUN", "GND", "+3V3", "VBUS", "V5_IN", "V5", "+1V1"]:
        s.add("Connector:TestPoint", "TP", net, TP, {"1": net}, in_bom=False)
    flags(s, "GND", "V5", "V5_IN", "+1V1", "VREG_AVDD", "VBUS", "V5_FFC")
    return s


def motor_gpio(g):
    """GPIO / bus signal -> net on mod_motor: board ID, ladder, the LED chain."""
    return {43: "BOARD_ID", 44: "LADDER", "LED_DOUT": "LED_DATA"}.get(g)


def mod_motor():
    """Motor plugs and the stepper IO: io_board() with JST GH 8P plugs, 0402
    passives, the 1206 polyfuse, and a new board ID (2.2k = GH 8P nested).
    Bottom of the stack: female sockets on top, everything else underneath."""
    s = Schematic("mod_motor", "Mod stack - motor plugs and stepper IO (JST GH 8P)")
    add_mod_header(s, "J1", mod_bus("A", motor_gpio), MOD_FEMALE, "mod bus A, top side")
    add_mod_header(s, "J2", mod_bus("B", motor_gpio), MOD_FEMALE, "mod bus B, top side")
    add_mod_header(s, "J3", mod_motor_out(), MOD_FEMALE, "motor outputs from mod_driver, top side")
    SUBST_FP.clear(), SUBST_VALUE.clear()
    SUBST_FP.update({C0805: C0603})          # 10 uF on the strip supply: 0603 10 V
    io_board("mod_motor", "", gh_motor_pair_nested, "2.2k", s)
    SUBST_FP.clear()
    for part in s.parts:
        if part.fields.get("LCSC") == "C209721":     # 1812 polyfuse -> the 1206 mod_mcu uses
            part.footprint = "Fuse:Fuse_1206_3216Metric"
            part.fields.update(LCSC="C269120", MPN="SMD1206B150TFT",
                               Note="PTTC; 1.5 A hold / 3 A trip, 8 V; same part as mod_mcu F1")
    for net in ["V5", "GND", "LADDER", "BOARD_ID", "LED_DATA"]:
        s.add("Connector:TestPoint", "TP", net, TP, {"1": net}, in_bom=False)
    flags(s, "GND", "V5", "+5V_EXT", "STRIP_5V")
    return s


# ---------------------------------------------------------------------------
# Single board: all three tiers on one PCB, no BTB connectors
# ---------------------------------------------------------------------------

def single(name, title, motor_conn, board_id_r, proto=False):
    """Controller + driver + one IO variant on one board.

    Same circuit as the stack, with the BTBs removed: their nets join directly.
    The board-ID resistor is kept so firmware identifies the motor connector
    variant the same way.
    """
    SUBST_FP.clear(), SUBST_VALUE.clear()
    if proto:   # all passives 0603, JLCPCB basic
        SUBST_FP.update(PROTO_FP), SUBST_VALUE.update(PROTO_VALUE)
    s = Schematic(name, title, paper="A1")
    driver(s)  # fixed refs U1-U10 first; the other parts number around them
    controller(s)
    io_board(name, title, motor_conn, board_id_r, s)
    flags(s, "GND", "V5", "V5_IN", "+1V1", "VREG_AVDD", "VBUS", "+5V_EXT", "STRIP_5V")
    SUBST_FP.clear(), SUBST_VALUE.clear()
    if proto:
        # Bring-up test pads (copper only, like TP1-TP5): every supply rail, the
        # shared ADC / open-drain lines, I2C and the LED chain.
        for net in ["VBUS", "V5_IN", "V5", "+1V1", "GND", "LADDER", "VMOT_SENSE", "NSLEEP",
                    "SDA", "SCL", "LED_DATA", "GND"]:
            s.add("Connector:TestPoint", "TP", net, TP, {"1": net}, in_bom=False)
        # The Abracon AOTA inductor ran out at JLCPCB (4 left, 2026-10-11): a
        # stocked shielded 3.3 uH 2016 part instead. Not the part Raspberry Pi
        # validated; no polarity mark.
        for part in s.parts:
            if part.fields.get("LCSC") == "C42411119":
                part.fields.update(LCSC="C411344", MPN="CMLW201610S3R3MST",
                                   Note="substitute for AOTA-B201610S3R3-101-T (out of stock); 245 mohm, 1.05 A sat")
            # The 2020 LED is "Standard PCBA only" at JLCPCB: the 5050 part of the
            # same family runs on Economic.
            if part.fields.get("LCSC") == "C5349955":
                part.lib_id, part.value = "LED:WS2812B", "XL-5050RGBC-2812B"
                part.footprint = "LED_SMD:LED_WS2812B_PLCC4_5.0x5.0mm_P3.2mm"
                part.fields.update(LCSC="C2843785", MPN="XL-5050RGBC-2812B",
                                   Note="XINGLIGHT WS2812B 5050; the 2020 part is JLCPCB Standard-PCBA only")
    return s


def main():
    boards = [
        controller(),
        driver(),
        io_board("io_xh", "Stepper controller - tier 3, 28BYJ-48 (JST-XH)", xh_motor, "10k"),
        io_board("io_sh", "Stepper controller - tier 3, dual 8 mm micro steppers (JST-SH 8P)", sh_motor_pair, "4.7k"),
        single("single_xh", "Stepper controller - single board, 28BYJ-48 (JST-XH)", xh_motor, "10k"),
        single("single_sh", "Stepper controller - single board, dual 8 mm micro steppers (JST-SH 8P)",
               sh_motor_pair, "4.7k"),
        single("single_sh_proto", "Stepper controller - single board prototype, all-0603 basic parts (JST-SH 8P)",
               sh_motor_pair_nested, "4.7k", proto=True),
        mod_driver(),
        mod_mcu(),
        mod_motor(),
    ]
    for b in boards:
        out = os.path.join(OUT, b.name)
        # single_sh(_proto) have a hand-arranged, wired drawing; the others are label-only
        # (mod_* too, for now: their wired drawings follow the placement).
        singles = single_sh_sch.write(b, out) if b.name.startswith("single_sh") else b.write(out)
        print(f"{b.name}: {len(b.parts)} parts; single-pin nets: {', '.join(singles) or 'none'}")


if __name__ == "__main__":
    main()
