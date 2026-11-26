# Modular boards (`mod_*`): separation plan

> **Status: plan, rev 3, 2026-11-26.** Branch `mod-boards`. Next: `mod_driver` netlist and a rough placement to fix the outline.

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
  - The proto's per-chip 2.2 uF VM caps and separate 3x 10 uF bulk (13 parts, ~52 uF) become **one 10 uF 0603 per driver pair**, between the two chips' VM pins: 5 parts, ~50 uF.
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
 ┌──┴──────────┴──────────┴─┐  mod_motor   2 layers  motor connectors + stepper IO (e-stop, LED strip, 5 V in)
 └──────────────────────────┘
```

- **Mod bus:** two 2x20 2.54 mm headers on opposite long edges, 80 pins. Every layer gets every GPIO, V5, 3V3 and GND. Layers only use the pins they need, so any layer can be swapped for a custom one.
  - 2.54 mm keeps it hand-pluggable and lets a perfboard be a layer: the two header rows sit on the 2.54 mm grid, 45.72 mm (18 pitches) apart.
  - At 3 A per pin, 4 pins per rail carry the motor current with plenty of margin. That would take ~10 pins per rail on the DF40.
  - The ~11 mm gap between layers leaves room for parts on both facing sides. The DF40 stack allowed ~1 mm.
- **Motor-output header:** a third 2x20 down the middle of `mod_driver` / `mod_motor`. It carries the 40 driver outputs and nothing else. It is a separate header so the bus stays generic.
- **Target outline ~65 x 56 mm** for all three layers, with the bus headers on the 65 mm edges. `mod_driver` is the densest layer, so its placement decides the final size.
  - It has 10 cells in two rows of 5, either side of the motor-output header.
  - The cell pitch is 10.16 mm, four header pins. Each motor's four outputs then run straight across to four adjacent pins of the middle header, and each row is 50.8 mm long.

### Parts (JLCPCB, checked 2026-11-26; all extended, THT ones hand- or THT-soldered)

| Use | Part | LCSC | Stock |
|---|---|---|---|
| Bus, `mod_mcu` underside (male) | PZ254-2-20-Z-8.5, 2x20 2.54 mm | C2894981 (alt C3012260) | 801 (1129) |
| Bus pass-through on `mod_driver` (female body on top, 12.3 mm tails mate the layer below) | 2x20 2.54 mm stacking (PC104) | C35165 | 6278 |
| Bus + motor-output sockets, `mod_motor` top (female) | 2x20 2.54 mm, 8.5 mm | C5124634 | 6394 |
| FFC port on `mod_mcu` | 05A20L60P, 60P 0.5 mm flip-top, bottom contact, 400 mA/pin | C22435645 | 1970 |
| Motor plugs, `mod_motor` (side entry at the board edge) | SM08B-SRSS-TB | C160407 | 151k |

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
| A | GPIO0-19 = M1-M5 | 10 GND, 4 V5, 2 3V3, RUN, SWCLK, SWDIO, LED_DOUT |
| B | GPIO20-39 = M6-M10 | GPIO40-47 (utilities), 7 GND, 3 V5, 1 3V3 |

- `LED_DOUT` is the status LED's data out. The LED strip on the bottom layer chains after it, as on the proto.
- RUN and SWD on the bus let a lower layer reset or debug the MCU.
- 3V3 on the bus comes from the ME6211 (500 mA). Budget roughly 300 mA for add-on layers.
- Exact pin numbers get assigned during the `mod_mcu` placement. Rule: GPIOs in pad order, a GND at least every 4th pin, power pins at the header ends.

## FFC port (draft)

60 pins, 0.5 mm: GPIO0-47 in pad order, 9 GND interleaved, 1 3V3, 2 V5.

- V5 goes out through a small polyfuse (~0.5 A hold) on `mod_mcu`, because each conductor is rated 400 mA. A remote motor layer has to bring its own 5 V.
- It is meant for an optional **`mod_ffc` adapter**: a small 2-layer board with the FFC socket and the two bus headers, plus a 5 V input. It rebuilds the mod bus at the far end of the cable, so `mod_driver` + `mod_motor` (or any other layer) can sit away from the MCU without any change to them. The adapter is a later board, not part of the first three.
- Cable: use a contacts-on-opposite-sides FFC (type B) so pin 1 maps to pin 1. Decide this when the adapter is drawn.

## Boundaries

### `mod_mcu`: general-purpose MCU layer

`controller()` from `gen/build.py`, plus the FFC. Proto refs:

- **MCU core:** U11, decoupling C35-C48, core regulator L1 / R24 / C49, crystal Y1 / C50 / C51 / R25.
- **USB and power path:** J1, U12 (ESD), R26 / R27, CC R28 / R29, F1, D1, C52 / C53, LDO U13 / C55, soft-start Q1 / C54 / R30.
- **Generic UI:**
  - Qwiic: J2 / J3 / JP1 / JP2 / R31 / R32.
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

### `mod_driver`: H-bridge layer

All of `driver()`. Proto refs:

- U1-U10, each with its sense resistors (0603) and VINT / VCP caps (0402). The cell follows the proto's: caps above the chip, ISEN pads straight below.
- One 10 uF 0603 per pair on the VM pins, in place of the proto's ten 2.2 uF and the 3x 10 uF bulk.
- NSLEEP pull-up R21, to bus 3V3.
- VMOT_SENSE divider R22 / R23 / C34. It stays here so it measures V5 at the drivers, after the connector drop. With no driver layer, GPIO42 reads 0 V, so firmware can tell.
- nFAULT stays wire-ORed onto LADDER (GPIO44) on the bus.
- Test pads: V5, VMOT_SENSE, NSLEEP, LADDER, GND.
- Uses bus GPIO0-39, 42 and 44, and passes all 80 pins through.

### `mod_motor`: motor connectors + stepper IO

This is the old tier-3 `io_board()`. The stepper-specific IO moves here from
`mod_mcu` so the MCU layer stays generic. Proto refs:

- **Motor plugs:** J11-J15, 5x JST-SH 8P with the nested pinout (`sh_motor_pair_nested()`), side-entry SM08B along one long edge. That edge must be clear of the bus header, so the plugs go on a short edge or the headers move inboard; settle this in placement.
- **Board-ID resistor**, new value: 2.2 k (595 mV) = "SH 8P, nested pinout". The proto's 4.7 k is shared with `io_sh`, which has the old pinout, so firmware can't tell the two apart.
- **External 5 V in:** J4 / D3 / C59, onto bus V5. Motor current from an external supply then never crosses into `mod_mcu`.
- **LED strip out:** U15 / R38 / C60 / F2 / JP3 / J5 / J6 / C61, fed from LED_DOUT.
- **E-stop** J7 / JP4 / R39 and **panel buttons** J8 / R40 / R41, on LADDER.
- Side-entry or edge-mounted versions of the PH/XH connectors, since the layer above sits ~11 mm over the board.

## Power

- V5 is one rail across the whole stack. Its sources:
  - USB on `mod_mcu`: PTC, Schottky, soft-start.
  - The external 5 V on `mod_motor`: Schottky. It back-feeds `mod_mcu`'s LDO through the soft-start FET's body diode, as on the proto.
- The soft-start was sized for ~70 uF on V5. The split keeps the same capacitors, so the timing stands.
- No hot-plugging layers.

## Open decisions

1. **Outline.** ~65 x 56 mm is a target. Confirm it with a `mod_driver` placement first, because that layer is the tightest.
2. **Motor plug edge** on `mod_motor`: a short edge, or move the bus headers inboard.
3. ~~Stacking header for the motor outputs~~: settled, stacking header (see Parts).
4. **FFC V5 limit:** a 0.5 A polyfuse, or a solder jumper (no V5 on the FFC by default).
5. **`mod_driver` on 2 layers?** Cheaper, but the proto's cells lean on In1 GND / In2 V5. Default 4.
6. **Inductor L1:** check whether the Abracon AOTA is back in stock before keeping the CMLW substitute. Take the proto's 1V1 ripple result into account.

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
