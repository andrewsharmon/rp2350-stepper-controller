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

Three boards, ~25 x 25 mm core, joined by fine-pitch board-to-board (BTB)
connectors at opposite edges. No screws: the enclosure captures the stack.

| Tier | Board | Layers | Size |
|---|---|---|---|
| 1 | Controller | 4 | ~25 x 25 mm |
| 2 | Universal driver | 2 | ~25 x 25 mm |
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
  - BOOTSEL (also readable at runtime).
  - Btn1 and Btn2 on the analog ladder (see below).
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

- **Part:** XUNPU BTB0.408 series, 0.4 mm pitch, 30 pins. Mating pair confirmed from the LCSC datasheets:
  - **Header** BTB0.408-30PLBDR-**G41** (C42420579). Goes on the **underside of the upper board**.
  - **Socket** BTB0.408-30PLBDR-**M41** (C42420584). Goes on the **top of the lower board**.
  - The two drawings show the same series and pitch, and the same 30-pin span (C = 5.60 mm). The socket body is 0.80 mm tall (series "0.408" = 0.4 mm pitch, 0.8 mm stack).
  - Both had about 18,000 in stock. Rated 0.3 A per pin.
- **Footprints** are in `lib/stack.pretty`, generated by `gen/footprints.py` from the datasheet layouts:
  - Socket: 0.20 x 0.40 signal pads, rows at +/-1.20; hold-downs 0.90 x 0.92 at y = +/-0.99, x = pin 1 - 1.00.
  - Header: 0.23 x 0.65 signal pads, rows at +/-0.875; hold-downs 0.40 x 0.53 at y = +/-0.565, x = pin 1 - 0.80.
  - Odd pins are on the -Y row, with pin 1 at -X.
- **Layout check:** the header is flipped onto B.Cu. Confirm that pin 1 lands on the mating socket's pin 1 (overlay both boards, or check in the 3D viewer) before ordering.
- **Mated height** is about 0.8 mm. Keep the facing sides of the boards clear under and around the connectors.
- **Two identical 30-pin pairs per interface.** Their positions are offset (asymmetric) so the boards can only mate one way.
- BTB pins carry roughly 0.3-0.5 A each.
- The authoritative pin lists are `btb_a()` and `btb_b()` in `gen/build.py`.

| Connector | Controller <-> driver | Driver <-> tier 3 |
|---|---|---|
| A | motors 1-5 inputs (20), 6x GND, 3x V5, 3V3 | motors 1-5 outputs (20), 7x GND, 3x V5 |
| B | motors 6-10 inputs (20), 3x V5, 3x GND, VMOT_SENSE, BOARD_ID, LADDER, LED_DATA | motors 6-10 outputs (20), 3x V5, 4x GND, BOARD_ID, LADDER, LED_DATA |

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

## Schematics and BOM

Schematics are generated. Edit `gen/build.py`, not the `.kicad_sch` files.

```bash
python3 gen/footprints.py   # project footprints -> lib/stack.pretty
python3 gen/build.py        # schematics
```

Outputs:

- `controller/`, `driver/`, `io_xh/` (28BYJ-48), `io_sh/` (micro steppers): one KiCad schematic each.
- Each part's pins are connected by net labels.
- `*_bom.csv` in each folder, exported with `kicad-cli sch export bom`.

Part data:

- Passives are generic (value + footprint).
- The LCSC field is only set on parts whose number was checked on lcsc.com.

Open items:

- Soft-start: measure the inrush on the first boards and adjust the 100 nF / 100 k if needed.
- ABM8-272-T3 (C20625731): check whether JLCPCB stocks it as basic or extended.
- PCB layout.
