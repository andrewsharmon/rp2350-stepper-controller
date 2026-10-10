# Hardware specification

> **Status: unreviewed feasibility first pass.** These schematics, part picks
> and pin maps were put together quickly to check that the concept fits. They
> have not been reviewed, checked against datasheets, laid out or built.
> Expect errors; don't fabricate from them as-is.

Low-cost, compact, up-to-10-axis stepper controller for small high-resistance
steppers (8 mm micro steppers, 28BYJ-48 converted to bipolar, <= 0.2 A/coil).
The MCU generates sine PWM (voltage-mode microstepping) directly into cheap
dual H-bridges; there is no stepper-driver IC.

Prefer LCSC / JLCPCB "basic" parts wherever possible. Part picks marked TBD
are open.

## Stack overview

Three boards, 30 x 30 mm core, joined by fine-pitch board-to-board (BTB)
connectors at opposite edges. No screws: the enclosure captures the stack.

| Tier | Board | Layers | Size |
|---|---|---|---|
| 1 | Controller | 4, 1.0 mm thick | 30 x 30 mm |
| 2 | Universal driver | 2 | 30 x 30 mm |
| 3 | Connector / IO (variant-specific) | 2 | sized by connectors, may overhang |

## Tier 1 - Controller

- MCU: **RP2354B** (QFN-80, 48 GPIO, 2 MB in-package flash). Run the ARM cores (FPU).
- 12 MHz crystal, core-regulator inductor and decoupling per the RP2350 hardware design guide.
  - Crystal: Abracon ABM8-272-T3 (C20625731, 10 pF load, ESR <= 50 ohm). The guide's 15 pF / 1 k values are tuned for it; other crystals need retuning and testing.
  - Inductor: Abracon AOTA-B201610S3R3-101-T has a polarity dot. Lay it out in the same orientation as the Pico 2.
- USB-C receptacle with THT shell legs. The enclosure supports the receptacle so plug forces never reach the BTBs.
- CC1 / CC2: 5.1 k pull-downs, also routed to ADC (GPIO40/41) to read the source current advertisement.
- Power path: `VBUS -> PTC (~1.5 A hold, 1812) -> low-Vf Schottky -> V5_IN -> soft-start switch -> V5 bus -> BTB`.
- 3.3 V LDO (ME6211 class, <= 6 V input) fed from V5_IN, ahead of the switch.
- Soft-start: AO3401A P-FET (C15127) with 100 nF gate-source and 100 k gate-ground (tau 10 ms). At plug-in the cap holds the FET off, then it turns on over a few ms, so the ~70 uF on the V5 bus charges at about 0.1 A. USB sees only the ~11 uF on V5_IN at attach. External 5 V (tier 3) reaches V5_IN through the FET's body diode, then its channel.
- 2x Qwiic (JST-SH 4-pin) on I2C1:
  - The board is not powered from Qwiic. The ME6211 conducts from VOUT back to VIN, so a Qwiic-fed 3V3 would back-power the V5 bus (drivers, LEDs).
  - Solder jumpers for pull-ups (2.2-4.7 k) and for supplying 3V3 to downstream Qwiic devices.
- WS2812-class addressable LED on GPIO45. Its DOUT continues down the stack as LED_DATA.
  - Powered from V5 (needs >= 3.7 V). The 3.3 V data goes through a 74AHCT1G125 on V5 to meet VIH = 0.7 VDD.
- Buttons:
  - BOOTSEL (also readable at runtime), TS-1187A, right edge below connector B. The single boards use the TS-1088-AR02016 instead (same part as Btn1/Btn2).
  - Btn1 and Btn2 on the analog ladder (see below): XUNPU TS-1088-AR02016 (C720477, 3.9 x 3.0 mm, 2 pads), stacked in the top-right corner. They started over the BTB headers for rigidity, but their pads blocked the connector fan-out, so the case should support the board under them.
- SWD test pads.

## Tier 2 - Universal driver

- 10x **AT8833CQ** (Zhongkewei, LCSC C5120769, QFN-16 4 x 4).
  - DRV8833-compatible logic: 00 coast, 01 reverse, 10 forward, 11 brake.
  - VM 2.7-15 V, UVLO <= 2.5 V, 1.0 A RMS. Inputs have 100 k pull-downs.
  - Second source: JSMSEMI DRV8833RTYR / JSM8833RTYR (same QFN-16 4 x 4). Verify its pinout against TI DRV8833RTY before relying on it.
- Passives per chip:
  - AISEN / BISEN: 0.68-0.82 ohm to ground. Above I = 0.2 V / R (Vtrip 160-240 mV) the chip chops the current in slow decay; it does not report a fault. With 0.82 ohm that is 0.2-0.29 A. Give it a separate return to the star ground.
  - VCP: 0.1 uF to VM.
  - VINT: 1 uF.
  - These are the AT8833 datasheet values. TI's DRV8833 uses 0.01 uF and 2.2 uF, so don't copy from it.
  - VM: 1-2.2 uF ceramic at the pin.
- Shared bulk: 2-4x 10 uF 0805 spread over the V5 pour.
- nSLEEP: pulled high through 20-75 k (always awake, <= 3.5 mA idle per chip).
- nFAULT (open drain): wire-OR all chips onto the LADDER line.
- VMOTOR is the V5 bus directly.
- VMOTOR_SENSE: 1:5 divider to GPIO42.

AT8833CQ QFN-16 pinout: 1 AISEN, 2 AOUT2, 3 BOUT2, 4 BISEN, 5 BOUT1, 6 nFAULT,
7 BIN1, 8 BIN2, 9 VCP, 10 VM, 11 GND, 12 VINT, 13 AIN2, 14 AIN1, 15 nSLEEP,
16 AOUT1, plus the exposed pad to GND.

## Tier 3 - Connector / IO

- Motor connectors per variant. Each uses the motor's stock plug:
  - `io_xh`, 28BYJ-48: one JST-XH 5-pin (B5B-XH-A, C157991) per motor, ten total.
    - Pins: 1 blue, 2 pink, 3 yellow, 4 orange. Pin 5 (red centre tap) is not connected after the bipolar mod.
    - Coils: A = pins 1/3, B = pins 2/4.
  - `io_sh`, dual 8 mm micro steppers: one JST-SH 8-pin (BM08B-SRSS-TB, C160394) per motor pair, five total.
    - Pins 1-4 = first motor (A1 A2 B1 B2), pins 5-8 = second motor.
  - Firmware remaps coil order and polarity per axis, so lead-colour variations can be fixed in software.
- Optional external 5 V input -> Schottky -> V5 bus. This powers everything without USB.
- LED strip output (V5 / DATA / GND):
  - Data: 74AHCT1G125 buffer (powered from V5), then a 33-100 ohm series resistor.
  - Strip power, chosen by footprint: shared V5 through a polyfuse, or a separate LED power input.
- E-stop jack and panel buttons, in parallel with the ladder.
- Board ID: one resistor to GND, read through the controller pull-up on GPIO43. See the board-ID table below.

## Interconnect (BTB)

- **Part:** Hirose DF40, 0.4 mm pitch, 40 pins, **2.0 mm stack height**:
  - **Header** DF40C-40DP-0.4V(51) (C424643). Goes on the **underside of the upper board**. The header is the same for every DF40 stack height.
  - **Socket** DF40C(2.0)-40DS-0.4V(51) (C597934). Goes on the **top of the lower board**. The socket sets the stack height.
  - Rated 0.3 A per contact. The four corner contacts (pins 1, 2, 39, 40) are metal fittings per Hirose (catalogue note 3): they are soldered for strength but left unconnected, leaving 36 usable pins.
  - The 2.0 mm stack leaves room between boards for low parts (0402 passives, QFNs, SOT-23) on the facing sides. Keep anything taller than ~1 mm off them, and don't put a part opposite another one on the facing board if the two heights add up to more than ~1.7 mm.
- **Footprints** are in `lib/stack.pretty`, generated by `gen/footprints.py` from the DF40 catalogue's recommended layouts:
  - Socket: 0.20 x 0.70 pads (rows 2.38-3.78 mm across), plus the 1.5 mm layout-prohibited strip between the rows as a copper keepout.
  - Header: 0.23 x 0.66 pads (rows 2.05-3.37 mm across), plus 0.35 mm metal-fitting tabs 0.30-0.65 mm beyond each row end.
  - Pin 1 at -X. The socket's odd row is at -Y; the header's is drawn at +Y, so after the flip onto the underside header pin n sits over socket pin n. Still check it in the 3D viewer before ordering.
- **Not polarized** (Hirose note 5). Two connectors per interface at offset positions key the stack so the boards only mate one way: A is 2 mm below the board centre line, B on it.
- BTB pins carry roughly 0.3-0.5 A each.
- The authoritative pin lists are `btb_a()` and `btb_b()` in `gen/build.py`.

| Connector | Controller <-> driver | Driver <-> tier 3 |
|---|---|---|
| A | motors 1-7 inputs (28), 4x V5, 3x GND, 3V3, 4 corner NC | motors 1-7 outputs (28), 4x V5, 4x GND, 4 corner NC |
| B | motors 8-10 inputs (12), LED_DATA, LADDER, BOARD_ID, VMOT_SENSE, 9x V5, 11x GND, 4 corner NC | motors 8-10 outputs (12), LED_DATA, LADDER, BOARD_ID, 9x V5, 12x GND, 4 corner NC |

Pin order follows the MCU's pin order, so the controller's traces fan in without crossing. Connectors are placed pin 1 at the top: A down the left edge (power, then M1 from the MCU's top-left corner, M2-M5 down its left side, M6-M7 round its bottom-left), B down the right edge (utilities, then M10, M9, M8). M6/M7 are on A so they don't cross the crystal and SWD pins on the MCU's bottom side. The driver board shares V5 between both connectors, so the uneven V5 split doesn't matter. The driver's sockets copy the controller headers' positions and rotation.

**Board ID:** the 10 k 1% pull-up to 3V3 is on the controller. Each tier-3 variant fits one resistor to GND, 5% is fine. With no tier 3 attached, the line reads 3.3 V.

The values are common 0402 parts. Ranges are worst case for a 5% ID resistor and the 1% pull-up; adjacent levels stay at least 249 mV apart, and the thresholds sit midway between ranges.

| ID resistor | Nominal | Range (mV) | Threshold above (mV) | Board |
|---|---|---|---|---|
| 0 ohm | 0 mV | 0 | 150 | |
| 1 k | 300 mV | 284-316 | 440 | |
| 2.2 k | 595 mV | 566-624 | 820 | |
| 4.7 k | 1055 mV | 1012-1098 | 1350 | io_sh (dual 8 mm micro steppers) |
| 10 k | 1650 mV | 1599-1699 | 1960 | io_xh (28BYJ-48) |
| 22 k | 2269 mV | 2225-2310 | 2500 | |
| 47 k | 2721 mV | 2691-2749 | 3020 | |
| not fitted | 3300 mV | 3300 | - | no tier 3 attached |

## GPIO map (RP2354B)

| GPIO | Function |
|---|---|
| 0-39 | Motor n (1-10) on GPIO 4(n-1)..4(n-1)+3 = AIN1 AIN2 BIN1 BIN2 |
| 40 | USB-C CC1 (ADC0) |
| 41 | USB-C CC2 (ADC1) |
| 42 | VMOTOR_SENSE, 1:5 (ADC2). Can't tell external power from USB (both feed V5 through Schottkys) |
| 43 | Tier-3 board ID (ADC3) |
| 44 | Ladder (ADC4) |
| 45 | WS2812 data |
| 46 | Qwiic SDA (I2C1) |
| 47 | Qwiic SCL (I2C1) |

PIO allocation:

| PIO block | State machines | Use |
|---|---|---|
| PIO0 | SM0-3 | Motors 1-4 |
| PIO1 | SM0-3 | Motors 5-8 |
| PIO2 (GPIOBASE = 16) | SM0-1 | Motors 9-10 |
| PIO2 | SM2 | WS2812 |
| PIO2 | SM3 | Spare |

Use external pull-ups on all inputs on A2 silicon (RP2350-E9 erratum; fixed from A3).

## Ladder on GPIO44

```
3V3 --[10k]--+---------> GPIO44 (ADC4), 100 nF to GND
             |-- E-stop (NC) --[20k]-- GND   (solder jumper bypass when no e-stop)
             |-- Btn1 ---------[10k]-- GND
             |-- Btn2 ---------[3.3k]- GND
             '-- nFAULT (open drain, all drivers)
```

| Condition | Voltage | Meaning |
|---|---|---|
| E-stop open or cable cut | 3.3 V | STOP |
| Idle | ~2.2 V | normal |
| Btn1 | ~1.3 V | user 1 |
| Btn2 | ~0.7 V | user 2 |
| nFAULT | ~0 V | driver fault, STOP |

Use 1% resistors. Thresholds sit midway between the levels; debounce over 2-3 samples.

A held button hides an open e-stop (e-stop open + Btn1 = 1.65 V, + Btn2 = 0.82 V). Firmware latches a stop on 1.49-1.76 V held 5 ms, or on any button held over 3 s; see `firmware/src/ladder.h`.

## Electrical notes

- PWM frequency is about 20 kHz. The AT8833 input deglitch (about 450 ns) sets a minimum pulse of about 0.5 us, roughly 1% duty. Firmware dithers duties below that.
- Wait about 100 ms after power-up before energizing motors (V5 soft-start settling).
- USB budget comes from CC; brownout guard from VMOTOR_SENSE.
- RP2350-E9 (a GPIO with its input buffer on can sit near 2.2 V, which the AT8833's >= 2 V VIH reads as high) needs no hardware fix on the motor inputs. It is fixed in silicon from A3 (A4 is the production stepping; order A4). On A2 it can't occur after a power-on or RUN reset (input enable starts clear), and the firmware turns the motor pins' input buffers off (they are never read), so they stay off across a watchdog reset whether or not it resets the pads. The AT8833's 100 k pull-downs are enough.
- Tier 3: the NO_ESTOP jumper ships bridged. It must be cut when an e-stop is fitted, or the e-stop does nothing. Mark this on the silkscreen.

## Single board (`single_sh`)

Controller, driver and the `io_sh` connectors on one PCB, from the `single_sh/` schematic (`single()` in `gen/build.py`). Fully routed: 0 unconnected, no DRC errors, schematic parity clean.

- Schematic: a wired, hierarchical drawing (`gen/single_sh_sch.py`), unlike the label-only stack schematics.
  - Sheets: overview, power and USB, MCU, motor drivers, connectors and UI. Each block is framed, with design notes on its sheet.
  - Inter-sheet signals (motor lines, LADDER, BOARD_ID, I2C, CC, USB) are global labels. Power rails are power symbols, so their PCB nets are `GND`, `+3V3`, `V5` and so on; nets local to a sheet are named `/<sheet>/<label>`, e.g. `/Power/V5_IN`.
  - The AT8833 uses a project symbol (`lib/stack.kicad_sym`, written by `build.py`) with the DRV8833RTY pin numbers, laid out for the drawing; VINT is typed as an output, so it needs no PWR_FLAG.

- **122 x 20 mm**, 4 layers, 1.6 mm thick. All parts on top; only the THT legs of the USB-C and the JST-PH/XH connectors come through.
- Layers: F.Cu parts and signals, In1 solid GND, In2 V5, B.Cu signals. In2 has a 3V3 island under the MCU, joined to the LDO by a thin finger along the -y edge.
- Along the board: USB-C (mouth on the short edge) and power path | drivers M4 M3 M2 M1 | MCU | drivers M10 M9 M8 M7 M6 M5 | LED strip connector.
  - Motor connectors run along one long edge in the order J12 J11 | J15 J14 J13, so each motor bus leaves the MCU in pin order and never crosses itself.
  - Qwiic x2, BOOTSEL, e-stop, panel buttons, 5 V in, status LED and LED power run along the other long edge, above the drivers.
- Driver cell: AT8833 rotated so its outputs face the connector. VINT/VM/VCP caps sit above it; the 0603 sense resistors sit below, ISEN pads inboard, each GND pad with its own via (R16 and R20 instead tie to the neighbouring cell's GND pad).
  - BOUT2 runs down between the two ISEN pads and BOUT1 under R_B's body. The resistors went 0805 -> 0603 in place on the routed board: pad 1 (ISEN) kept, GND pad 0.275 mm inboard (R5 and R11 keep the GND pad instead, clear of a neighbouring via and track).
  - The connector's pin order (B2 B1 A2 A1, left to right) runs against the driver's (A1 A2 B2 B1), so A1/A2 hop over B1/B2 on short In2 jumpers (4 vias per motor).
  - Swapping this board's connector pin assignment (pins 1-2 = coil B, pins 3-4 = coil A reversed) would remove the jumpers, if firmware maps coil order and polarity per board. Not done, because the cable pinout would then differ from `io_sh`.
- V5 enters the In2 plane through via clusters at the soft-start FET, the external 5 V diode and the LED-strip fuse.
- Layout source: `single_sh()` in `gen/pcbgen.py` (placement and zones).
  - The driver cells and the core regulator were pre-routed by script. The rest was routed with Freerouting 2.5.0 plus a small grid router.
  - Those routing scripts are not in the repo; the board is hand-edited from here.
- Silkscreen: reference designators are hidden (too dense).
  - Motor numbers sit above each connector; function labels are on top where they fit, otherwise on the underside.
  - The underside also carries the NO_ESTOP cut warning.
  - The 4 remaining silkscreen warnings are outlines touching each other or the board edge.

### Fabrication (JLCPCB)

`gen/jlcfab.py` writes the order files to `single_sh/jlcpcb/`: Gerbers + Excellon drill (zipped), the BOM and the placement (CPL) file. Run it with KiCad's Python; `--check` also verifies the placements.

```bash
/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3 gen/jlcfab.py single_sh --check
```

- JLCPCB places each part with its own library footprint, so `CPL_FIX` corrects rotation and origin per LCSC part (SOT-23, SOT-23-5, WS2812, the JST connectors and USB-C need it).
  - `--check` fetches those footprints from the JLCEDA library and compares their pads with the KiCad pads.
  - All 142 placements land within 0.003 mm of centre with every pin on its own pad.
- Pin numbering traps:
  - SS54 (C22452): JLCPCB numbers 1 = anode, 2 = cathode, the reverse of KiCad's diodes. `PAD_MAP` handles it; the diodes need no extra rotation.
  - AOTA inductor (L1): the dot goes at the +1V1 end, as on the Pico 2 (RP2350 datasheet, regulator layout figure). JLCPCB's silkscreen and Abracon's top view put the dot at their pad 2, so pad 2 lands on +1V1. JLCPCB's placement preview shows the dot there; the order remark repeats it.
- The BOM has one line per LCSC part. JLCPCB unticks a part that appears on several lines.
- First order (2026-10-07, 10 boards, $314 before shipping):
  - PCB: 4 layers, 1.6 mm, ENIG (0.4 mm pitch QFN-80), min via 0.15/0.3 mm, plugged vias.
  - Assembly: Standard PCBA, top side, confirm parts placement with no auto-confirm.
- Cost drivers:
  - The 0.15 mm via option ($37) forces a 4-wire Kelvin test ($18) and TG155.
  - The WS2812B-2020 (D2) is Standard-PCBA only, as are its alternatives. Standard adds $25 setup, feeder fees and 5 mm edge rails (a 122 x 70 mm panel, nearly double the shipping weight). Economic with D2 left off was $265.
  - THT parts (JST PH/XH, the USB-C shell legs) are hand soldered, about $6.
- Vias sit in the QFN exposed pads and in U11 pin 59 (+3V3). JLCPCB doesn't offer tented for this board; plugged (solder mask) is the free option. Epoxy filled & capped costs about $20 more if those joints give trouble.

## Schematics and BOM

Schematics are generated. Edit `gen/build.py`, not the `.kicad_sch` files. For `single_sh`, part placement and wiring live in `gen/single_sh_sch.py`.

```bash
python3 gen/footprints.py   # project footprints -> lib/stack.pretty
python3 gen/build.py        # schematics
```

The controller and `single_sh` PCBs were started by `gen/pcbgen.py <board>`, run once with KiCad's Python
(it needs `pcbnew`); it refuses to overwrite an existing board. It set up the
outline, stackup, JLCPCB rules and net classes, and a first placement of every
part. From there the layout is edited by hand in KiCad.

```bash
/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3 gen/pcbgen.py controller
```

Outputs:

- `controller/`, `driver/`, `io_xh/` (28BYJ-48), `io_sh/` (micro steppers): one KiCad schematic each.
- `single_xh/`, `single_sh/`: the whole circuit (controller + driver + that IO variant) on one board, with the BTB connectors removed and their nets joined directly. `single_sh` has a routed PCB (see above); `single_xh` is schematic only.
- Each part's pins are connected by net labels, except in `single_sh`, which is drawn with wires (`gen/single_sh_sch.py`, using `gen/schdraw.py`).
  - Before writing, `schdraw` traces the drawing the way KiCad does (wires, junctions, labels, power symbols) and fails unless every pin is on exactly the net `build.py` gave it.
  - Moving or adding a part in `build.py` means placing it in `single_sh_sch.py` too; an unplaced part, unwired pin or short stops the build.
  - Refs and symbol UUIDs are what the routed PCB links to. Changing a net's name or sheet changes its PCB net name: run Update PCB from Schematic in KiCad afterwards.
- `*_bom.csv` in each folder, exported with `kicad-cli sch export bom` (grouped by value, footprint, LCSC, MPN and note).

Part data:

- Every BOM line has an LCSC number, checked against the JLCPCB parts library on 2026-10-07 (part, package, rating, stock). Passives take theirs from `PASSIVES` in `gen/build.py`.
- JLCPCB basic parts where one exists. Extended lines: RP2354B, AT8833CQ, 3.3 uH inductor, crystal, ESD diode, LDO, AHCT buffer, status LED, polyfuse, USB-C, the JST connectors, and three passives:
  - 2.2 uF VM caps: the only basic 0402 2.2 uF is rated 6.3 V, so a 16 V part is used on the 5 V motor rail.
  - 0.82 ohm sense resistors (0603, 250 mW): no basic part exists.
  - 27 ohm USB series resistors and the 39 k VMOT_SENSE divider resistor: no basic 0402 part exists.
- Solder jumpers and test pads are excluded from the BOM and from the placement file (they are copper only).

Open items:

- Soft-start: measure the inrush on the first boards and adjust the 100 nF / 100 k if needed.
- Stock to watch (JLCPCB, 2026-10-07):
  - AOTA-B201610S3R3-101-T inductor: 677 in stock (662 when the first order was prepared).
  - RP2354B: about 1.7k in stock.
  - Polyfuse: 1.97k in stock. Ruilon C702823 isn't in JLCPCB's library, so PTTC SMD1812P150TF/8 (C209721) is used.
  - Status LED: the Worldsemi WS2812B-2020 (C965555) had 1 in stock, so the XINGLIGHT XL-2020RGBC-WS2812B (C5349955) is used. It has the same pinout and footprint.
- Sense resistors: 0603, 250 mW (Viking SR731JTTDR820F, C186570; 2.2k in stock 2026-10-10). Holding a coil at the AT8833 current-limit trip (up to 0.29 A at VTRIP 240 mV) dissipates 69 mW, 28 % of rating. The worst case is VTRIP^2 / R, so a lower R for more current needs R >= 0.29 ohm to keep 20 % margin. Most 0603 0.82 ohm parts are 100 mW (69 %); keep a 250 mW part if this one is substituted.
- LED PWR (J6) is a JST-PH rated 2 A per pin, which caps a separately powered strip at 2 A unless the connector changes.
- PCB layout: controller fully routed (0 unconnected; silkscreen labels still to tidy); `single_sh` fully routed (unreviewed); driver and tier-3 boards not started. Routing used Freerouting 2.5.0 plus a small grid router for the last nets; neither is part of the repo.
