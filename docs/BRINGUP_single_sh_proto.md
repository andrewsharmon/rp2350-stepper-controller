# single_sh_proto bring-up

Bring-up checklist for the first `single_sh_proto` boards (ordered 2026-10-11, Economic PCBA; D2 is the 5050 status LED, hand-soldered). Work through it in order. Don't connect motors until step 6.

Firmware: branch `single-sh-proto-firmware`, board target `single_sh_proto`.

```bash
cmake -S firmware -B firmware/build-proto -G Ninja -DPICO_BOARD=single_sh_proto
cmake --build firmware/build-proto
```

Equipment: a multimeter, a current-limited bench supply (5 V, 2 A), a USB-C cable to the host (a USB meter in line helps), a scope or logic analyser if you have one, an SWD probe (optional; Pico Probe / Debug Probe), one or two of the 8 mm bench steppers with a nested-pinout cable, and the e-stop and panel-button harnesses.

## 0. Inspection (unpowered)

- [ ] Check under magnification: U11 (RP2354B QFN-80) and the ten AT8833s for bridges, especially the QFN corners; L1 (CMLW201610S3R3MST, no polarity); crystal; USB-C J1 pins.
- [ ] Hand-solder D2 (XL-5050RGBC-2812B). Check its pin 1 against the silkscreen before soldering.
- [ ] Solder jumper defaults as shipped: JP2 I2C_PU **bridged**, JP3 STRIP_SHARED **bridged**, JP4 NO_ESTOP **bridged**, JP1 QWIIC_SUPPLY **open**.
- [ ] Resistance to GND (meter red on the rail; low readings that rise are capacitors charging):

| Test pad | Net | Expect | Fail if |
|---|---|---|---|
| TP6 | VBUS | > 10 k | < 100 ohm |
| TP7 | V5_IN | > 1 k | < 100 ohm |
| TP8 | V5 | > 1 k (10 drivers) | < 100 ohm |
| TP5 | +3V3 | > 1 k | < 100 ohm |
| TP9 | +1V1 | > 100 ohm (core load) | < 10 ohm |

- [ ] V5 to +3V3 and +3V3 to +1V1 are not shorted.

## 1. First power (bench supply, no USB data)

Feed **J4 (5V IN)** from the bench supply: 5.0 V, **current limit 200 mA**. No motors, no LED strip.

- [ ] Idle current < 60 mA. (The MCU runs from flash if the factory left a program; a blank RP2354B sits in the boot ROM.)
- [ ] Voltages:

| Test pad | Net | Expect |
|---|---|---|
| TP7 | V5_IN | ~5.0 V (J4 feeds V5_IN directly, after the Schottky from VBUS) |
| TP8 | V5 | ~5.0 V, rises over ~10 ms (soft-start P-FET) |
| TP5 | +3V3 | 3.30 V +-3 % |
| TP9 | +1V1 | 1.10 V. **Scope the ripple**: L1 isn't the Raspberry Pi validated part. Expect < ~30 mV p-p |
| TP12 | VMOT_SENSE | V5 / 4.9, ~1.02 V |
| TP11 | LADDER | ~2.2 V (JP4 bypass, idle). 0 V means a driver nFAULT is low or there's a short |
| TP13 | NSLEEP | ~V5 (47 k pull-up) |
| TP14/TP15 | SDA/SCL | 3.3 V (JP2 pull-ups) |

- [ ] Nothing gets hot (fingertip on U11, the LDO, L1 and each driver).

## 2. USB and first flash

Remove the bench supply. Plug USB-C into the host (USB meter in line).

- [ ] TP6 VBUS ~5.0 V; V5 ~4.6-4.7 V (Schottky drop).
- [ ] Hold BOOTSEL, tap RUN (or plug in with BOOTSEL held): an `RP2350` drive appears. If it doesn't, check USB D+/D- (22 ohm series resistors), the crystal (scope XIN for 12 MHz) and +1V1; SWD (TP1 SWCLK, TP2 SWDIO, TP3 RUN, TP4 GND) is the fallback.
- [ ] `picotool info -a` reports an RP2350 with 2 MB of flash.
- [ ] Flash: `picotool load -x firmware/build-proto/stepper.uf2` (or copy the .uf2 to the drive).
- [ ] The board enumerates as a USB serial port. Open it (`screen /dev/tty.usbmodem* 115200`) and type `?`.

## 3. Status check (`?` output)

| Field | Expect |
|---|---|
| ladder | ~2200 mV (`idle`) |
| vmot | ~4600-5000 mV (measured V5) |
| board id | ~1050 mV (10 k pull-up, 4.7 k to GND) |
| cc | the 5.1 k Rd pull-downs: ~0.4 V on default USB, ~0.9 V at 1.5 A, ~1.7 V at 3 A, on the cable's CC side; ~0 V on the other |
| motors | `10 motors (10 with pins)` |
| state | `running`, or `STOPPED (c to clear)` |

- [ ] vmot within 2 % of TP8 measured with the meter. If it's off, adjust `BOARD_VMOT_RATIO` in `firmware/src/board_pins.h`.
- [ ] `cfg`: axes 1, 3, 5, 7 and 9 show `rev` and `ba` (nested-pinout defaults). If a saved config from another board is in flash, run `defaults` then `save`.
- [ ] `X` reboots cleanly and the port comes back.

## 4. Status LED and LED strip

- [ ] D2 (pixel 0) shows status: blinking red = stopped (red/magenta blink = driver fault), blue = all axes holding, solid red = watchdog tripped.
- [ ] Scope TP16 LED_DATA: 5 V WS2812 bursts after the 74AHCT1G125 buffer.
- [ ] Connect a short strip to J5 (power through JP3 from V5). Pixels 1-10 follow the motors' speed and direction once they move.
- [ ] Optional: cut JP3 and feed the strip from J6 (LED PWR) instead.

## 5. Ladder: e-stop and buttons

Watch the ladder in `?` (or `ts ladder` with `t 10`).

- [ ] Idle (JP4 bridged): ~2.2 V, `idle`.
- [ ] SW1 Btn1: ~1.3 V, `btn1`. SW2 Btn2: ~0.7 V, `btn2`. Same through J8 PANEL BTN.
- [ ] Programs (actuators mid-stroke at power-up): Btn2 steps the selection (the LED bar grows by one), Btn1 starts program 1; Btn2 while it runs stops the axes and starts the next. Step through all 12 and back to program 1; Btn1 stops.
- [ ] Hold a button over 3 s: stop latches; `c` clears it.
- [ ] E-stop test: cut JP4, fit the NC e-stop on J7. Closed: idle. Pressed or unplugged: 3.3 V, stop latches, outputs off. `c` clears it only after the e-stop is released.
- [ ] Short TP11 LADDER to GND briefly through 1 k (simulates nFAULT): the stop latches.

## 6. Motors, one connector at a time

Power from USB, or from the bench supply at 5 V with a **1 A limit**. Start on J11 (M2 + M1). Use a nested-pinout cable (README, "Prototype single board").

- [ ] `amp 30` (fixed 30 %), then `v 2 100`: M2 (outer pins) turns smoothly. `s`.
- [ ] `v 1 100`: M1 (inner pins) turns smoothly in the **same direction** as M2. If it buzzes without turning, the coil swap is wrong: try `coils 1 ab`. If it turns backwards: `dir 1 fwd`. Note any change here, so the defaults in `config.c` can be fixed.
- [ ] `amp auto`, then `m 1,2 2000` and `m 1,2 0`: both return to the same place without missing steps.
- [ ] Supply current with two motors at hold (~25 %), then running at 48 % (the default high amplitude). Driver and sense resistors only warm.
- [ ] Repeat for J12-J15 (M3-M10).
- [ ] With all ten connected: `stress on` for 10 minutes on the bench supply. Watch V5 sag in `?` (vmot), driver temperatures, the core busy figures, and that no underrun or fault events appear (`te on`). `stress off`.
- [ ] Re-measure the speed and amplitude limits on this board's V5 (bench notes: 60 % with a 2000 steps/s^2 ramp held 1919 full steps/s on the Pico 2). Update the `config_defaults` vmax if it differs.

## 7. Qwiic I2C target

- [ ] With a Pico or another I2C host on J2/J3: the target answers at 0x42 (`i2cdetect` or a bus scan). stepperctl has no I2C path yet, so this needs a small host script.
- [ ] JP1 (QWIIC_SUPPLY) stays open unless a downstream device needs 3V3 from this board.

## 8. Host tools

- [ ] From `tools/`: `python3 -m stepperctl status` and `ping` answer over USB; `show` uploads and runs a demo from `tools/shows`.
- [ ] Upload a show, set `boot <n>`, `save`, power-cycle: the show starts by itself.
- [ ] Watchdog: `W` trips the watchdog and the outputs go off; `X` recovers.

## Record

For each board: serial/date, idle currents (step 1 and 2), 1V1 ripple, vmot error, ladder levels, any coil/direction changes, max reliable speed at 48 %, issues. Put fixes into the next revision's notes in `hardware/README.md`.

### Board 1 (USB serial 9E6A370B4D94ADC7), 2026-10-21

- Step 2: came up in BOOTSEL as `RP2350` (UF2 Bootloader v1.1). Flashed by copying `stepper.uf2` to the drive (the SDK-built picotool has no USB support, so `picotool info -a` and the 2 MB flash check weren't run). Enumerates as `/dev/tty.usbmodem12401`.
- Step 3 (`?` on USB):

  | Field | Read |
  |---|---|
  | ladder | 2196 mV, `idle` |
  | vmot | 4792 mV; TP8 on the meter 4.813 V (-0.4 %, `BOARD_VMOT_RATIO` 4.9 kept) |
  | board id | 1055 mV |
  | cc | 508 / 44 mV (default USB) |
  | motors | `10 motors (10 with pins)`, core busy 0.0 %, ring low-water 59/63 |
  | state | `running`, config defaults |

  `cfg` shows `rev ba` on axes 1, 3, 5, 7, 9 and `fwd ab` on the even axes. `X` reboots and the port is back in 2.6 s.
- Step 4: D2 shows blue. LED strip and TP16 not checked yet.
- Step 5: SW1/SW2 presses report in the CLI (levels not recorded). Hold-to-stop, e-stop and nFAULT checks not done yet.
- Step 6 (2026-10-22, USB power): all ten axes drive linear steppers under `stress on`, two motors (one connector) at a time, on the config defaults with no coil or direction changes. With the stress test on all ten axes, core 1 is ~40 % busy and the ring low-water is 58/63.
- Step 6, ten motors at once (USB power): all ten linear steppers run together under `stress on`. vmot 4792 mV idle, ~4650 mV holding, 4430-4520 mV running.
- **Core 0 hang**, ~80 min into a ten-motor stress run: the console stopped answering (USB still enumerated), D2 froze green while the motors sat still and energised. The 1200-baud BOOTSEL reset still worked, so core 0's interrupts were alive and its main loop was stuck. Suspect a core 1 stall that leaves core 0 spinning in `control_snapshot`, which also blocks the core 1 watchdog. Not reproduced or confirmed yet (no SWD probe at the time).
- Speed (linear steppers, unloaded, USB power, `amp auto`, scurve 30 ms): end-to-end moves over +-1370 full steps, reaching cruise in the first quarter of the stroke, run clean at **6000 full steps/s** (26,000 full steps/s^2). Taken as the practical max for these actuators on these settings; the rotary 8 mm bench limit was 1919. `config_defaults` vmax (1500) left as is.
- Still to do in step 6: same-direction checks, `m` round trips, supply current, the bench-supply ten-motor run.
- Not yet recorded: steps 0-1 (resistances, bench-supply idle current, rail voltages, 1V1 ripple), the rest of step 6, steps 7-8.
