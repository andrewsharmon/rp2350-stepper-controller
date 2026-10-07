"""Generate the three stack schematics and the single-board versions.

    python3 build.py        # writes ../controller, ../driver, ../io_xh, ../io_sh,
                            # and the one-board versions ../single_xh, ../single_sh

Passives get their LCSC / MPN from PASSIVES below; the LCSC field is only set
on parts whose number was checked (JLCPCB parts library, 2026-10-07).
"""

import os

from schgen import Schematic

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.dirname(HERE)

R0402 = "Resistor_SMD:R_0402_1005Metric"
C0402 = "Capacitor_SMD:C_0402_1005Metric"
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
    ("0.82", R0402): ("C728435", "RL0402FR-070R82L"),    # Yageo 1 %, 62.5 mW, extended (no basic 0.82R)
    ("27", R0402): ("C25100", "0402WGF270JTCE"),         # extended
    ("33", R0402): ("C25105", "0402WGF330JTCE"),
    ("1k", R0402): ("C11702", "0402WGF1001TCE"),
    ("3.3k 1%", R0402): ("C25890", "0402WGF3301TCE"),
    ("4.7k", R0402): ("C25900", "0402WGF4701TCE"),
    ("5.1k", R0402): ("C25905", "0402WGF5101TCE"),
    ("10k", R0402): ("C25744", "0402WGF1002TCE"),
    ("10k 1%", R0402): ("C25744", "0402WGF1002TCE"),
    ("20k 1%", R0402): ("C25765", "0402WGF2002TCE"),
    ("39k 1%", R0402): ("C25783", "0402WGF3902TCE"),     # extended
    ("47k", R0402): ("C25792", "0402WGF4702TCE"),
    ("100k", R0402): ("C25741", "0402WGF1003TCE"),
}


def passive(sch, lib_id, prefix, value, fp, nets, kw):
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

def driver(s=None):
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
        R(s, "0.82", f"{u}_AISEN", "GND", Note="chops at 0.2-0.29 A (VTRIP 160-240 mV); Kelvin return")
        R(s, "0.82", f"{u}_BISEN", "GND", Note="chops at 0.2-0.29 A (VTRIP 160-240 mV); Kelvin return")
        C(s, "1u", f"{u}_VINT", "GND")
        flags(s, f"{u}_VINT")  # internal regulator output, typed power_in in the symbol
        C(s, "100n", f"{u}_VCP", "V5")
        C(s, "2.2u", "V5", "GND")

    for _ in range(3):
        C(s, "10u", "V5", "GND", C0805)
    R(s, "47k", "+3V3", "NSLEEP")
    R(s, "39k 1%", "V5", "VMOT_SENSE")
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


# ---------------------------------------------------------------------------
# Single board: all three tiers on one PCB, no BTB connectors
# ---------------------------------------------------------------------------

def single(name, title, motor_conn, board_id_r):
    """Controller + driver + one IO variant on one board.

    Same circuit as the stack, with the BTBs removed: their nets join directly.
    The board-ID resistor is kept so firmware identifies the motor connector
    variant the same way.
    """
    s = Schematic(name, title, paper="A1")
    driver(s)  # fixed refs U1-U10 first; the other parts number around them
    controller(s)
    io_board(name, title, motor_conn, board_id_r, s)
    flags(s, "GND", "V5", "V5_IN", "+1V1", "VREG_AVDD", "VBUS", "+5V_EXT", "STRIP_5V")
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
    ]
    for b in boards:
        singles = b.write(os.path.join(OUT, b.name))
        print(f"{b.name}: {len(b.parts)} parts; single-pin nets: {', '.join(singles) or 'none'}")


if __name__ == "__main__":
    main()
