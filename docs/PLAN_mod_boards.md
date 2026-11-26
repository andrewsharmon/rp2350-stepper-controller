# Modular boards (`mod_*`): separation plan

> **Status: plan, rev 6, 2026-11-26.** Branch `mod-boards`.
> - All three boards (`mod_mcu`, `mod_driver`, `mod_motor`): netlists (`gen/build.py`, label-only for now) and placements (`gen/pcbgen.py`) done, unrouted, all on the **56 x 47.18 mm** outline.
> - Next: wired schematics, then routing.

The fabricated `single_sh_proto` (150 x 30 mm, ordered 2026-10-11, board 1
brought up 2026-10-21) is split into a stack of three boards.

Goals:

1. **Stack first.** The boards plug together with no cables.
2. **`mod_mcu` is a general-purpose RP2354B layer** that other projects can reuse. All 48 GPIO are broken out, and nothing stepper-specific lives on it.
3. **An FFC port on `mod_mcu`** so the same board can also drive a layer that sits somewhere else.

It is the same circuit as `single_sh_proto`: the same GPIO map and the same
nested dual-motor cable pinout. New parts: the stack headers, the FFC socket,
the free-up jumpers on `mod_mcu`, and a new board-ID value.

### Cost and size rules (this rev)

Cost and compactness come first.

- **Passives are 0402 by default**, JLCPCB basic. A part goes 0603 only where the 0402 basic part is under-rated or isn't basic:
  - Sense resistors, 1 ohm 0603 (C22936). Holding a coil at the trip dissipates up to 58 mW, against an 0402's 62.5 mW rating.
  - V5 10 uF, 0603 10 V (C19702). The basic 0402 10 uF (C15525) is 6.3 V, too close on a 5.5 V rail once DC bias is counted.
  - 39 k VMOT_SENSE divider, 0603 (C23153). The 0402 one is extended.
- **Shared driver capacitors.** Per chip, VCP 100 nF and VINT 1 uF stay. They are the charge pump and the internal regulator, so they can't be shared.
  - The proto's per-chip 2.2 uF VM caps and separate 3x 10 uF bulk (13 parts, ~52 uF) become **three 10 uF 0603 per row of five drivers**, each in the gap between two chips: 6 parts, ~60 uF.
  - The 2.2 uF line leaves the BOM, and the V5 capacitance the soft-start was sized for stays about the same.
- **Extended parts are counted.** Each unique extended part costs a JLCPCB setup fee, so reuse one part number wherever a choice exists.
- **Not done, needs your call:** tying AISEN / BISEN straight to GND would remove all 20 sense resistors and a strip of board, but also the chopper current limit (0.16-0.24 A), the only hard protection for the small motors.

## Stack

```
        FFC (60P, 0.5 mm) ──> optional remote layer
          │
 ┌──────────────────────────┐  mod_mcu     4 layers  RP2354B, USB-C, power entry, Qwiic, LED, buttons
 └──▲─────────────────────▲─┘                         2x20 male headers underneath
    │ mod bus A    mod bus B│                         (~11 mm gap)
 ┌──┴─────────────────────┴─┐  mod_driver  4 layers  10x AT8833CQ, V5 sense
 └──▲──────────▲──────────▲─┘                         stacking headers carry the bus through
    │ bus A    │ motor out│ bus B                     + the motor-output header
 ┌──┴──────────┴──────────┴─┐  mod_motor   2 layers  GH motor plugs + stepper IO, all underneath
 └──────────────────────────┘
```

- **Mod bus:** two 2x20 2.54 mm headers on opposite long edges, 80 pins. Every layer gets every GPIO, V5, 3V3 and GND. Layers only use the pins they need, so any layer can be swapped for a custom one.
  - 2.54 mm keeps it hand-pluggable and lets a perfboard be a layer: the headers sit on one 2.54 mm grid, with the bus inner rows 15 pitches (38.1 mm) apart and J3 halfway between.
  - At 3 A per pin, 4 pins per rail carry the motor current with plenty of margin. That would take ~10 pins per rail on the DF40.
  - The ~11 mm gap between layers leaves room for parts on both facing sides. The DF40 stack allowed ~1 mm.
- **Motor-output header:** a third 2x20 down the middle of `mod_driver` / `mod_motor`. It carries the 40 driver outputs and nothing else. It is a separate header so the bus stays generic.
- **Outline 56 x 47.18 mm** for all three layers (set by the `mod_driver` placement, 2026-11-26), with the bus headers on the 56 mm edges. The header bodies end 2.6 mm from the short edges.
  - It has 10 cells in two rows of 5, either side of the motor-output header.
  - The cell pitch is 10.16 mm, four header pins. Each motor's four outputs then run straight across to four adjacent pins of the middle header, and each row is 50.8 mm long.

### Parts (JLCPCB, checked 2026-11-26; all extended, THT ones hand- or THT-soldered)

| Use | Part | LCSC | Stock |
|---|---|---|---|
| Bus, `mod_mcu` underside (male) | PZ254-2-20-Z-8.5, 2x20 2.54 mm | C2894981 (alt C3012260) | 801 (1129) |
| Bus pass-through on `mod_driver` (female body on top, 12.3 mm tails mate the layer below) | 2x20 2.54 mm stacking (PC104) | C35165 | 6278 |
| Bus + motor-output sockets, `mod_motor` top (female) | 2x20 2.54 mm, 8.5 mm | C5124634 | 6394 |
| FFC port on `mod_mcu` | 05A20L60P, 60P 0.5 mm flip-top, bottom contact, 400 mA/pin | C22435645 | 1970 |
| Motor plugs, `mod_motor` (vertical, on the underside) | JST GH BM08B-GHS-TBT, 1.25 mm | C133062 ($0.35) | 30k |

Prices (JLCPCB, qty 10): stacking C35165 $0.86, female C5124634 $0.27 at 50, male C3012260 $0.33, FFC 60P $0.15 at 50, AT8833CQ $0.25 at 50.

**Motor-output header on `mod_driver`: the C35165 stacking header too.**
- All three driver headers then share one part and one tail length, so they mate `mod_motor` at the same height by construction.
- A male header would save ~$0.53 per board, but its mated height would have to match the stacking tails.
- Its female body sits on top, under `mod_mcu`, so `mod_mcu` keeps its underside clear there. It doubles as a 40-pin probe point for the outputs.

## Mod bus pinout (draft budget)

The split follows the RP2354B pad order (checked on the proto's U11), and each
header feeds one row of drivers on `mod_driver`:

- Pads 77-80 and 1-20 run GPIO0-20 along one side of the chip. GPIO0-19 go to **header A** = M1-M5, the driver row next to A.
- Pads 21-28 and 36-58 run GPIO20-47. They go to **header B** = M6-M10 and the utilities, the row next to B.
- The crystal and SWD (pads 30-35) sit on the side between them, which faces a short edge. USB, QSPI and the core regulator (pads 61-76) face the other short edge, where the USB-C and the FFC go.

Each header puts signals on its inner row (toward the board centre) and power on its outer row (the board edge). Every signal pin then has a power or GND pin beside it, and the driver rows' inputs come in without crossing.

| Header | Inner row (20) | Outer row (20) |
|---|---|---|
| A | GPIO0-19 = M1-M5 | 11 GND, 4 V5, 2 3V3, RUN, SWCLK, SWDIO |
| B | GPIO20-39 = M6-M10 | GPIO40-47 (utilities), LED_DOUT, 7 GND, 3 V5, 1 3V3 |

LED_DOUT is on B because the status LED sits by GPIO45, on the MCU's bus B side.

- `LED_DOUT` is the status LED's data out. The LED strip on the bottom layer chains after it, as on the proto.
- RUN and SWD on the bus let a lower layer reset or debug the MCU.
- 3V3 on the bus comes from the ME6211 (500 mA). Budget roughly 300 mA for add-on layers.
- Pin numbers are fixed in `mod_bus()` in `gen/build.py`. Inner row = odd pins, outer row = even pins.
- **Direction.** Seen from the top, the pin numbers run the way the RP2354B's pads do (counter-clockwise), so `mod_mcu`'s fan-out doesn't cross.
  - A runs along the -y edge, pin 1 at the +x end (GPIO0 east).
  - B runs along the +y edge, pin 1 at the -x end (GPIO20 west).
- **Driver input order.** Because of that direction, each motor's four GPIOs reach its driver mirrored. `mod_driver` therefore wires GPIO 4k..4k+3 to BIN1, BIN2, AIN2, AIN1 (not AIN1, AIN2, BIN1, BIN2).
  - That drives (A, B) = (-sin, cos): the same field turned 90 degrees electrical.
  - The motor turns the same way, one full step on, so the firmware is unchanged.
  - In return, the inputs need no crossings at all.

## FFC port

60 pins, 0.5 mm, along `mod_mcu`'s west edge with pin 1 at the north end. Pinout: `mod_ffc()` in `gen/build.py`.

- All 48 GPIO, 9 GND (one after every 6 signals), 1 3V3, and 2 V5_FFC pins.
- **Pin order** is whatever routes from the bus header pins to the FFC on B.Cu without crossing:
  - GPIO19..0 (header A, west to east).
  - Then header B's GPIOs, east to west, with the outer-row utilities fitted between the inner pins they pass.
- **V5_FFC comes through an open solder jumper,** so by default there's no V5 on the FFC. Bridged it is unfused: 2 conductors at 400 mA each. That was decision 4, settled on cost; a remote motor layer brings its own 5 V.
- **Part:** HanElectricity 05A20L60P (C22435645, $0.15 at 50). The Amphenol F32Q 60P has a KiCad footprint but costs $1.21 and had 9 in stock.
  - Its footprint (`stack:FFC_0.5mm_60P_FlipTop`, written by `gen/footprints.py`) uses the Hirose FH12 land pattern. The 05A20L60P drawing couldn't be read (LCSC's viewer has no text; JLCEDA's footprint API now blocks scripts).
  - **Check the pads against its drawing before ordering.**
- It is meant for an optional **`mod_ffc` adapter**: a small 2-layer board with the FFC socket and the two bus headers, plus a 5 V input. It rebuilds the mod bus at the far end of the cable, so `mod_driver` + `mod_motor` (or any other layer) can sit away from the MCU without any change to them. The adapter is a later board, not part of the first three.
- Cable: use a contacts-on-opposite-sides FFC (type B) so pin 1 maps to pin 1. Decide this when the adapter is drawn.

## Boundaries

### `mod_mcu`: general-purpose MCU layer

`mod_mcu()` in `gen/build.py`: `controller()`'s circuit rebuilt with generic GPIO net names, plus the FFC. Proto refs:

- **MCU core:** U11, decoupling C35-C48, core regulator L1 / R24 / C49, crystal Y1 / C50 / C51 / R25.
- **USB and power path:** J1, U12 (ESD), R26 / R27, CC R28 / R29, F1, D1, C52 / C53, LDO U13 / C55, soft-start Q1 / C54 / R30.
- **Generic UI:**
  - Qwiic: **one** connector (proto J2 / J3: two) plus the supply jumper and the I2C pull-ups. Qwiic devices daisy-chain, and the second connector didn't fit.
  - BOOTSEL SW3 / R36.
  - Status LED U14 / D2 / C57 / C58.
  - Ladder pull-up R33 / C56 with on-board buttons SW1 / R34 and SW2 / R35.
  - Board-ID pull-up R37.
- **Test pads:** TP1-TP5 (SWD, RUN, GND, 3V3), VBUS, V5_IN, +1V1, SDA, SCL.

To keep it general, every on-board function on GPIO40-47 sits behind a
**bridged solder jumper**. Cut it to get a plain GPIO:

| GPIO | On-board function | Jumper |
|---|---|---|
| 40 / 41 | USB-C CC1 / CC2 sense (the 5.1 k pull-downs stay on CC) | JP_CC1, JP_CC2 |
| 42 | none (VMOT_SENSE comes from `mod_driver`) | - |
| 43 | board-ID 10 k pull-up | JP_ID |
| 44 | ladder pull-up, 100 nF, SW1 / SW2 | JP_LADDER |
| 45 | status LED buffer input | JP_LED |
| 46 / 47 | I2C 4.7 k pull-ups (JP2 on the proto already does this) | JP2 |

GPIO0-39 have nothing on `mod_mcu`. GPIO40-47 are the RP2354B's ADC pins,
which is why the on-board analog functions sit there.

All eight solder jumpers (the five above, I2C_PU, QWIIC_SUPPLY, FFC_V5) are
on the **underside**. They're copper only, so they cost nothing there. Set
them before the board goes into the stack.

Part changes from the proto, for cost and size:
- **0402 passives.** USB series resistors are 22 ohm 0402 (C25092, basic); the 27 ohm 0402 is extended.
- **V5_IN bulk:** 10 uF 0603.
- **F1:** a 1206 1.5 A polyfuse (SMD1206B150TFT, C269120, 8 V) instead of the 1812. Both are extended.
- **D1 stays SS54 (SMA):** every basic Schottky rated 2 A or more is SMA.
- **L1 is the Abracon AOTA-B201610S3R3-101-T again** (C42411119), back in stock (5130, 2026-11-26). That's the part the RP2350 design guide validated.

### `mod_driver`: H-bridge layer

All of `driver()`. Proto refs:

- U1-U10, each with its sense resistors (0603) and VINT / VCP caps (0402). The cell follows the proto's: caps above the chip, ISEN pads straight below.
- Three 10 uF 0603 per row in the gaps between chips (6 in all), in place of the proto's ten 2.2 uF and the 3x 10 uF bulk.
- NSLEEP pull-up R21, to bus 3V3.
- VMOT_SENSE divider R22 / R23 / C34. It stays here so it measures V5 at the drivers, after the connector drop. With no driver layer, GPIO42 reads 0 V, so firmware can tell.
- nFAULT stays wire-ORed onto LADDER (GPIO44) on the bus.
- Test pads: V5, VMOT_SENSE, NSLEEP, LADDER, GND.
- Uses bus GPIO0-39, 42 and 44, and passes all 80 pins through.

### `mod_motor`: motor connectors + stepper IO

This is the old tier-3 `io_board()`. The stepper-specific IO moves here from
`mod_mcu` so the MCU layer stays generic. Proto refs:

- **Motor plugs:** J11-J15, 5x **JST GH 8P (1.25 mm)**, one pitch up from the proto's JST-SH (1.0 mm); your decision, 2026-11-26.
  - Same nested pinout (`gh_motor_pair_nested()` = `sh_motor_pair_nested()` with the GH part), so the per-axis firmware flags stay as on the proto.
  - Vertical BM08B-GHS-TBT on the underside, plugs facing down.
- **Board-ID resistor**, new value: 2.2 k (595 mV) = "GH 8P, nested pinout". The proto's 4.7 k is shared with `io_sh`, which has the old pinout, so firmware can't tell the two apart.
- **External 5 V in:** J4 / D3 / C59, onto bus V5. Motor current from an external supply then never crosses into `mod_mcu`.
- **LED strip out:** U15 / R38 / C60 / F2 / JP3 / J5 / J6 / C61, fed from LED_DOUT.
- **E-stop** J7 / JP4 / R39 and **panel buttons** J8 / R40 / R41, on LADDER.
- Everything but the three sockets is on the **underside**. The bottom of the stack is free there, and the layer above sits only ~8.5 mm over the top.
  - The PH / XH connectors are the proto's THT parts, bodies underneath.
  - All SMD parts are on B, so the PCBA stays single-sided.
- The 1812 strip polyfuse becomes the same 1206 part as `mod_mcu` F1. The 10 uF is 0603; passives are 0402.

## Power

- V5 is one rail across the whole stack. Its sources:
  - USB on `mod_mcu`: PTC, Schottky, soft-start.
  - The external 5 V on `mod_motor`: Schottky. It back-feeds `mod_mcu`'s LDO through the soft-start FET's body diode, as on the proto.
- The soft-start was sized for ~70 uF on V5. The split keeps the same capacitors, so the timing stands.
- No hot-plugging layers.

## Open decisions

1. ~~Outline~~: 56 x 47.18 mm from the `mod_driver` placement. `mod_mcu` (MCU, USB-C, power, 60P FFC between the bus headers) and `mod_motor` (5 SH plugs) still have to fit it. If the plugs don't fit `mod_motor`'s two free short edges, they go on its underside (top-entry BM08B), or `mod_motor` overhangs.
2. ~~Motor plug edge~~: there's no room for side entry, so the plugs face down from the underside. If they have to come out sideways instead, `mod_motor` needs an overhanging edge, which would take the cheaper side-entry GH clone (C42376901, $0.08) but make the board bigger.
3. ~~Stacking header for the motor outputs~~: settled, stacking header (see Parts).
4. ~~FFC V5 limit~~: an open solder jumper (no V5 on the FFC by default), for cost.
5. **`mod_driver` on 2 layers?** Cheaper, but the proto's cells lean on In1 GND / In2 V5. Default 4.
6. ~~Inductor L1~~: the Abracon AOTA is back in stock, so `mod_mcu` uses it.
7. **FFC footprint:** check the FH12-derived land pattern against the 05A20L60P drawing.

## `mod_driver` placement (2026-11-26)

Generated by `gen/pcbgen.py mod_driver`; parts only, no copper yet.

- Bus A (J1) and B (J2) on the long edges, motor-output header J3 down the middle.
- Rows of five cells: M5..M1 under A (chips rot 90, caps toward A), M6..M10 over B (rot 270). Pitch 10.16 mm, so each motor's GPIOs sit straight above its chip and its outputs straight below.
- The proto cell is unchanged except for 0402 VINT / VCP caps, which sit 4.3 mm from the chip centre instead of 4.75.
- The shared 10 uF caps are in the chip gaps. The free gap in each row holds the NSLEEP pull-up and the VMOT_SENSE divider.
- Test pads sit at the board's short ends.
- **Ratsnest:**
  - Inputs, outputs and the motor buses: no crossings.
  - Each motor's second input passes its VINT or VCP cap.
  - NSLEEP and LADDER (nFAULT) are multi-drop nets. Route them as spines, as on the proto.
- DRC: no courtyard or clearance errors; all 259 connections still unrouted.
- **BOM:** 63 placements, 9 lines. Basic: the 0402 / 0603 passives. Extended: AT8833CQ (C5120769) and the stacking header (C35165, 3 per board).

## `mod_mcu` placement (2026-11-26)

Generated by `gen/pcbgen.py mod_mcu`; all 73 footprints placed, no courtyard or clearance errors, unrouted.

**Layout:**
- **Bus headers:** the male headers sit on the underside, pin for pin under `mod_driver`'s J1 / J2.
- **MCU:** centred, rot 270, so its pads run counter-clockwise like the bus. Checked: GPIO0 at the north-east, GPIO20 at the north-west, GPIO47 at the south-east.
- **West edge:** the FFC, with its cable going out west. It's the top layer, so a flat cable can run off over the board.
- **East edge:** USB-C, with the power path, BOOTSEL and the ESD / CC parts between it and the MCU.
- **Corners:** the Qwiic connector and the status LED (with its buffer) south-east, by GPIO45-47. The crystal sits in the pocket by XIN / XOUT. One ladder button is south-west, the other north-west.
- **Decoupling and core regulator:** as on the proto, turned with the chip.

**Routing room is kept free of parts:**
- A band along each bus header.
- The MCU's north and south columns.
- The FFC's via field, which takes the B.Cu branches up to its pads.
- A channel for GPIO20-27, from the MCU's north-west corner down to bus B's west end, outside the crystal.

**Routing plan.** Straight-line checks:
- **B.Cu** (bus pins to FFC): 15 crossings over 48 lines, so the FFC order mostly works.
- **F.Cu** (MCU to bus): the crossings concentrate on:
  - GPIO40-47, which run to bus B's outer row and have to pass the inner-row lines.
  - RUN / SWD, from the west face to bus A's outer row.
  - The GPIO20-27 wrap.

  These need short layer hops. **In2 is therefore 3V3 (+1V1) under the MCU only, as on the proto, and a signal layer under the bus bands.** In1 stays solid GND.

**Tidy by hand:** one I2C pull-up (R9) landed at the east end of bus B's inner row.

**BOM:** 1 RP2354B, 2 bus headers (C3012260), the 60P FFC, USB-C, 1 Qwiic, 3 buttons. Passives are 0402 basic, except 10 uF 0603 and the 1206 polyfuse. The 5050 status LED is hand-soldered on an Economic order, as on the proto.

## `mod_motor` placement (2026-11-26)

Generated by `gen/pcbgen.py mod_motor`, 2 layers; all 31 footprints placed, no courtyard or clearance errors, unrouted.

**Top:** the female bus sockets J1 / J2 and the motor-output socket J3, at the same pin positions as `mod_driver`.

**Underside:**
- The five GH plugs stand across each half of the board, each beside the J3 pins it comes from:
  - North half (J3's odd row): M5+M6 west, M3+M4, M1+M2 east.
  - South half (even row): M7+M8, M9+M10 east.
- Each plug has pin 1 at the end away from J3. The nested pinout then reaches its J3 row with no crossings when the outer pins (1, 2) are routed round one side of the plug's pad column and the rest down the other. A straight-line ratsnest still draws crossings because it can't show that wrap.
- M5+M6 takes both J3 rows, so a few of its lines change layer.
- IO, each with its parts: E-STOP and PANEL BTN in the gaps of the north half; 5 V IN, LED STRIP and LED PWR in the south-west.

**BOM:** 23 placements on one side (5 GH plugs extended, the rest are the proto's parts), plus the hand-soldered THT sockets and IO connectors.

## Work plan

1. Write the bus, motor-output and FFC pin tables as `mod_bus_a()` / `mod_bus_b()` / `mod_motor_out()` / `mod_ffc()` in `gen/build.py`, like `btb_a()` / `btb_b()`. Write the footprints in `gen/footprints.py` if the KiCad library ones don't fit.
2. `gen/build.py`: add `mod_mcu()`, `mod_driver()` and `mod_motor()`, reusing `controller()` (+ the GPIO40-47 jumpers and the FFC), `driver()` and `io_board()` (whole, with `sh_motor_pair_nested` and the 2.2 k ID) with the proto substitutions and the stack headers added.
3. Wired multi-sheet schematics like `single_sh` / the proto: one `gen/mod_*_sch.py` per board on `schdraw.py`, ERC 0.
4. PCBs:
   - Do a `mod_driver` placement first to fix the outline, then `mod_mcu`, then `mod_motor`.
   - Start each with `gen/pcbgen.py`. Copy in the proto's placement and routing where it carries over (driver cells, MCU decoupling and core regulator, USB / power path).
   - Proto rules: 0.52 / 0.3 vias, 45 degree routing, no vias in pads.
   - Check the stack in the 3D viewer: header heights, facing-side parts.
5. Firmware:
   - Board `-DPICO_BOARD=mod`, same GPIO map as the proto.
   - Decode the board ID: no motor layer / nested SH / old SH / XH.
   - Check for "no driver layer" on VMOT_SENSE.
6. Fab outputs via `gen/jlcfab.py <board> --check` for each board. Add a `mod_*` section to `hardware/README.md`.
7. Later: the `mod_ffc` adapter.

Not in scope for the first pass: splitting the drivers over several layers (2 x 5 motors), and encoders, which are still planned as a Qwiic / SPI add-on (the bus and FFC expose I2C on GPIO46/47).
