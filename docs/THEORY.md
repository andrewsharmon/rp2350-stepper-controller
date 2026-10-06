# Theory of Operation

How the firmware turns a command such as "move axis 3 to 500 steps" into
current in a stepper coil: through trajectory planning, a 1 kHz motion tick,
20 kHz sine microstepping, PWM encoding, and PIO pin toggling fed by DMA.
The end of the document covers every motion mode and the extra features
(groups, PVT, cams, shows, safety, configuration).

Source references are in `firmware/src/`. The diagrams use Mermaid, which
GitHub renders.

---

## 1. Big picture

There are no stepper driver ICs. The RP2350 works out the coil voltages
itself (voltage-mode sine microstepping) and drives cheap AT8833/DRV8833 dual
H-bridges directly, four GPIOs per motor, for up to 10 motors.

The two cores divide the work strictly:

| Core | Job | Timing |
|---|---|---|
| **Core 0** | USB console, binary protocol, I²C target, show player, LEDs, telemetry, flash, buttons, core 1 watchdog | best effort, ~0.5 % idle load |
| **Core 1** | every axis's trajectory, interpolation, amplitude, microstepping, PWM encoding, DMA rings, e-stop ladder | hard real time, ~33–35 % load |

```mermaid
flowchart LR
    subgraph Host
        USB[USB serial<br/>text console + COBS frames]
        I2C[Qwiic I²C target]
        BTN[Ladder buttons]
    end

    subgraph Core0[Core 0: non-real-time]
        CON[console.c / protocol.c]
        I2CT[i2c_target.c]
        PLY[player.c<br/>show → PVT feed]
        SA[standalone.c]
        WD[core 1 watchdog]
        LED[led.c WS2812]
        CFG[config / show / cam store<br/>flash]
    end

    Q[(Command queue<br/>64 × control_cmd_t)]
    SNAP[(Snapshot<br/>seqlock)]

    subgraph Core1[Core 1: control_core1_main]
        APPLY[apply commands]
        TICK[1 kHz motion tick<br/>vleaders → groups → axes → cams]
        INT[per-period interpolation]
        MS[microstep: sine LUT +<br/>error diffusion]
        ENC[hbridge_encode<br/>4 segments/period]
        LAD[ladder poll<br/>every 10 µs]
    end

    subgraph HW[Hardware]
        RING[DMA ring per motor<br/>64 periods = 3.2 ms]
        PIO[PIO SM per motor]
        HB[AT8833 H-bridge]
        M((Stepper))
        ADC[ADC round robin DMA]
    end

    USB --> CON
    I2C --> I2CT --> CON
    BTN --> ADC
    CON --> Q
    PLY --> Q
    SA --> PLY
    Q --> APPLY --> TICK --> INT --> MS --> ENC --> RING --> PIO --> HB --> M
    TICK --> SNAP --> CON
    SNAP --> PLY
    SNAP --> LED
    ADC --> LAD
    LAD -- stop --> PIO
    WD -. heartbeat .- Core1
```

Core 0 never touches motion state directly. It **posts** commands into a
lock-free 64-entry queue (`control_post`, each stamped with a sequence
number) and **reads** state from a snapshot that core 1 publishes every tick
under a sequence lock (`control_snapshot`). Commands report back through
`last_seq` (the last command applied) and `rejected` (commands core 1
refused, such as a move on a grouped axis).

---

## 2. Units and number formats

Everything hinges on one choice of fixed-point position:

```
1 full step          = 2^30 units      (MOTION_UNITS_PER_STEP)
1 electrical cycle   = 4 full steps = 2^32 units
```

Position is an `int64_t`. Its **low 32 bits are exactly the electrical phase**
of the motor, so microstepping needs no conversion: `phase = (uint32_t)pos`.
Wrapping is free, the resolution is about 10⁻⁹ step, and the range is about
8.6 × 10⁹ full steps, so long moves never lose a unit.

Speeds are `float` full steps/s and accelerations full steps/s².

---

## 3. The rate pyramid

```mermaid
flowchart TB
    A["Commands (any rate)<br/>host, show player, buttons"] --> B
    B["Trajectory tick: 1 kHz (MOTION_TICK_HZ)<br/>generators, filters, groups, cams, amplitude target"] --> C
    C["PWM period: 20 kHz (HBRIDGE_PWM_HZ)<br/>20 periods per tick: phase interpolation, amplitude slew,<br/>sine lookup, error diffusion, encoding"] --> D
    D["PIO clock: 75 MHz (150 MHz / 2)<br/>3750 clocks per period, 4 segments, pin patterns"]
```

Core 1 doesn't run on a timer interrupt. It runs a tight loop that keeps
every motor's DMA ring full. Each pass:

1. Find `n`, the smallest number of free PWM periods across all rings.
2. For each of those `n` periods:
   * if this is period 0 of a tick (`sub == 0`), run the 1 kHz motion tick;
   * for each motor, compute one PWM period and append it to its ring.

The DMA drains the rings at exactly 20 kHz, so the PWM clock paces the
motion tick. The ring holds 3.2 ms of output, so core 1 can be late by up
to about 3 ms without any glitch at the motor.

---

## 4. Trajectory planning (one axis): `motion.c`

Each axis is a `motion_axis_t`, split into a **raw generator** and an
**output filter**:

```mermaid
flowchart LR
    CMD[command<br/>move / vel / jog / pvt] --> GEN
    subgraph motion_tick
        GEN["generate()<br/>gen_pos, gen_vel"] -->|"Δ = gen_pos − gen_last<br/>(units per tick)"| F1
        F1["moving average 1<br/>(scurve, smooth)"] --> F2["moving average 2<br/>(smooth)"]
        F2 --> OUT["pos += Δ<br/>vel = Δ·1000/2^30"]
    end
    GRP[group_tick] -. writes gen_pos .-> GEN
    CAMT[cam_tick] -. writes gen_pos .-> GEN
```

### 4.1 Generators

**Trapezoid, the exact discrete stopping law.** In position mode, every tick
the generator picks the fastest next speed `u` that can still stop exactly
on the target. It solves

```
(v + u)/2 · dt  +  u² / (2a)  ≤  distance remaining
```

for `u`, clamps the result to `vmax`, and moves toward it by at most
`a·dt`. Because this is the discrete law (not the continuous `v = √(2ad)`),
the ramp never overshoots. When a tick would reach the target at a speed it
could stop from in one tick, the generator snaps exactly onto the target.
Retargeting mid-move is free: the next tick simply uses the new distance.

**Velocity and jog** move `gen_vel` toward `cmd_vel` at `amax`. Jog also
counts down a watchdog: if the host doesn't refresh it within the timeout,
`cmd_vel` becomes 0 and the axis ramps to a stop.

**Cosine and quintic (fixed time).** At the start of the move,
`start_segment` picks the shortest duration `T` that respects the limits.
From rest:

| shape | peak v | peak a |
|---|---|---|
| cosine | π/2 · d/T | π²/2 · d/T² |
| quintic (minimum jerk) | 15/8 · d/T | 10/√3 · d/T² |

A quintic retargeted mid-move is re-solved with the current `v₀` and `a₀` as
boundary conditions. `T` is stretched by 15 % per iteration until a sampled
check passes. Each tick advances by Simpson's rule on the velocity (not by
evaluating `x(t)` in float), and every few ticks the generator re-syncs to
the exact double-precision curve, spreading the correction over the next
interval.

### 4.2 S-curve filters

`scurve` and `smooth` don't use a separate jerk-limited planner. They run the
trapezoid's **per-tick position deltas** through one or two moving averages
of length `Tj` (`jerk_ms`, up to 100 ms):

```mermaid
xychart-beta
    title "Velocity: trapezoid vs moving-averaged (conceptual)"
    x-axis "time" [0,1,2,3,4,5,6,7,8,9,10,11,12]
    y-axis "v" 0 --> 10
    line [0,4,8,10,10,10,10,10,10,8,4,0,0]
    line [0,2,5,8,9.5,10,10,10,10,9.5,7,3,0.5]
```

* One stage turns each step in acceleration into a ramp: jerk = `amax / Tj`.
* Two stages make the jerk itself continuous.
* Averaging never raises peak speed or acceleration.
* The integer remainder of each division is **carried**, so the filtered
  total equals the input total exactly: the output lands on the target to
  the unit.

The cost is a lag of `Tj` per stage. An axis counts as `settled` only once
the generator is idle **and** the filters have flushed. Profile changes are
accepted only while settled.

PVT and cam followers set `unfiltered` and bypass the filters, because their
input is already smooth and a lag would break their timing.

---

## 5. Core 1 motion tick ordering

Order matters, because some axes read others' positions in the same tick:

```mermaid
sequenceDiagram
    participant Q as Command queue
    participant VL as Virtual leaders (2)
    participant G as Groups (4)
    participant AX as Normal axes
    participant CAM as cam_tick
    participant FW as Follower axes
    participant AMP as Amplitude / snapshot

    Q->>Q: apply() every pending command
    VL->>VL: motion_tick (trapezoid)
    G->>AX: group_tick writes members' gen_pos
    AX->>AX: motion_tick (non-followers)
    CAM->>FW: gen_pos = f(leader.pos) + offset (+ blend)
    FW->>FW: motion_tick (followers)
    AMP->>AMP: Δpos per axis → per-period step, want_amp, publish()
```

---

## 6. From a 1 ms tick to 20 PWM periods: interpolation

After the tick, each axis has moved `Δ = pos_now − pos_before` units. That
movement is spread evenly over the next 20 PWM periods:

```
q     = Δ / 20            (64-bit: Δ can exceed 2^31 above ~2000 steps/s)
rem   = Δ − 20q
step  = q, plus ±1 for the first |rem| periods
```

so after 20 periods the electrical phase has advanced by **exactly** Δ. The
microstep phase therefore never drifts from the trajectory position, and on
an e-stop clear it is resynced with `phase = (uint32_t)pos`.

The result is piecewise-linear phase at 20 kHz, which follows the 1 kHz
trajectory with sub-microstep resolution.

---

## 7. Drive amplitude (voltage-mode current control)

With no current sensing, the coil current is set by the PWM **amplitude**, a
0..1 fraction of the full duty span. Each tick computes a target:

```mermaid
flowchart TD
    E{energized?} -- no --> Z[0]
    E -- yes --> MAN{manual amp ≥ 0?}
    MAN -- yes --> MV[manual value]
    MAN -- no --> H{settled for<br/>hold_delay_ms?}
    H -- yes --> HOLD[amp_hold]
    H -- no --> CURVE["auto_amplitude(|v|)"]
```

```
amp
 ^          amp_high ________
 |                  /
 |                 /   (linear: makes up for back-EMF)
 | amp_low _______/
 +----------------+------+-------> |speed|
              low_speed  high_speed
```

Standstill heats the coils most, so after the hold delay the axis drops to
`amp_hold`. At each PWM period the actual amplitude slews toward the target
at 2.0 per second (`AMP_SLEW_PER_SEC`), so run/hold changes don't jerk the
rotor.

---

## 8. Sine microstepping: `microstep.c`

```mermaid
flowchart LR
    P["phase (u32)"] -->|">> 22"| IDX[idx 0..1023]
    IDX --> SIN["LUT[idx] → sin (Q15)"]
    IDX -->|"+256"| COS["LUT[idx+256] → cos"]
    SIN --> SB["× amp × max_duty / 32767"]
    COS --> SA["× amp × max_duty / 32767"]
    SA --> DA[error diffusion A]
    SB --> DB[error diffusion B]
    DA --> SW[swap coils / reverse flags]
    DB --> SW
    SW --> OUTD[duty_a, duty_b<br/>signed counts]
```

* **Lookup table:** 1024 entries per electrical cycle, which is 256 entries
  per full step (1/256 microstepping). Coil A gets cosine and coil B gets
  sine, a quarter cycle apart. The table is generated offline
  (`tools/gen_sine_lut.py`) because the SDK's `sinf` misbehaved near
  multiples of π/2 on the RP2350.
* **Error diffusion:** the exact duty is fractional, and the driver swallows
  pulses shorter than ~450 ns. Each coil keeps a `carry`: rounding residue,
  and any duty below `min_duty` (500 ns ≈ 38 counts), are added to the next
  period instead. The **average** duty therefore matches the request even
  near the zero crossings, where a naive driver would produce a dead zone.
* **Wiring fixes:** `SWAP_COILS` exchanges A and B, and `REVERSE` negates B
  (which reverses rotation).

---

## 9. PWM encoding: `hbridge_encode.c`

At 150 MHz / 2 the PIO runs at 75 MHz, so a 20 kHz period is **3750 SM
clocks**. Each period is four segments, and each segment is a 16-bit
half-word:

```
bits [3:0]   pin pattern: AIN1 AIN2 BIN1 BIN2
bits [15:4]  length L; the segment lasts L + 3 clocks
```

The usable span is `3750 − 4×3 = 3738` counts per coil. For signed duties
`a` and `b`:

```
|<-- min(|a|,|b|) -->|<-- |hi|-|lo| -->|<----- rest/2 ----->|<----- rest/2 ----->|
 both coils driven     longer coil only     both braked           both braked
 (seg 1)               (seg 2)              (seg 3)               (seg 4)
```

| Pattern | Meaning |
|---|---|
| `xIN1=1, xIN2=0` | forward drive |
| `xIN1=0, xIN2=1` | reverse drive |
| `xIN1=1, xIN2=1` | **brake** (slow decay: both low-side FETs on, coil current recirculates) |

The off-time is brake (slow decay) rather than coast, which gives smoother
current at low duty. The brake time is split in two segments only so each
length fits in 12 bits.

The two half-words of segments 1–2 form word 0, and segments 3–4 form
word 1: **two 32-bit words per period per motor**.

---

## 10. DMA rings and the PIO program: IO toggling

```mermaid
flowchart LR
    C1[core 1<br/>hbridge_write_period] -->|"w0,w1 at wr"| R
    subgraph R[Ring: 128 words, aligned, wraps on 512 B]
        direction LR
        r0[...] --- r1[w0] --- r2[w1] --- r3[...]
    end
    R -->|"DMA, endless, DREQ = TX FIFO"| F[TX FIFO<br/>joined, 8 deep]
    F -->|autopull 32| SM[PIO SM]
    SM -->|out pins,4| G[GPIO 4n..4n+3]
```

The PIO program is three instructions:

```
.wrap_target
    out pins, 4      ; set AIN1 AIN2 BIN1 BIN2 for this segment
    out x, 12        ; segment length
seg_loop:
    jmp x-- seg_loop ; wait L+1 clocks (plus 2 for the outs = L+3)
.wrap
```

* **One state machine per motor:** motors 0–3 on PIO0, 4–7 on PIO1, 8–9 on
  PIO2. All SMs in a block start with `pio_enable_sm_mask_in_sync`, so the
  periods stay phase-aligned.
* **The DMA** runs in endless mode with ring wrap (`channel_config_set_ring`,
  9 address bits = 512 B), paced by the SM's TX FIFO DREQ. It replays the
  ring forever, and core 1 only has to stay ahead of the read pointer
  (`hbridge_free_periods`, with one period of slack).
* **Fail safe:** if the FIFO ever runs dry, the SM stalls holding its last
  pattern. The encoder always ends a period with brake, so a stall leaves
  both coils shorted, with no drive.
* `hbridge_safe_off` aborts the DMA, stops the SM, and hands the pins to SIO
  driven low (coast). It is used by the e-stop and the core 1 watchdog,
  because a stalled producer would otherwise leave the ring replaying stale
  periods.

### End-to-end timing of one period

```mermaid
gantt
    dateFormat X
    axisFormat %s
    title One 50 µs PWM period (|a| = 2000, |b| = 800 of 3738)
    section Coil A
    drive        :a1, 0, 2003
    brake        :a2, after a1, 3750
    section Coil B
    drive        :b1, 0, 803
    brake        :b2, after b1, 3750
```

---

## 11. Motion modes

```mermaid
stateDiagram-v2
    [*] --> IDLE
    IDLE --> POSITION: MOVE / MOVE_REL
    IDLE --> VELOCITY: VELOCITY
    IDLE --> JOG: JOG
    IDLE --> PVT: PVT_START (points queued, at rest)
    IDLE --> GROUP: GROUP_CREATE (settled)
    IDLE --> CAM: CAM_ENGAGE (settled)
    POSITION --> POSITION: retarget
    POSITION --> VELOCITY: VELOCITY
    VELOCITY --> POSITION: MOVE
    POSITION --> IDLE: arrived
    VELOCITY --> IDLE: STOP → v=0
    JOG --> IDLE: timeout / STOP → v=0
    PVT --> IDLE: last point, v=0
    PVT --> VELOCITY: underrun while moving (brake at amax)
    GROUP --> IDLE: GROUP_RELEASE (at rest)
    CAM --> VELOCITY: STOP (disengage, ramp down)
    POSITION --> IDLE: e-stop (halt)
```

| Mode | What drives `gen_pos` | Notes |
|---|---|---|
| `IDLE` | nothing | holds; drops to hold amplitude after `hold_delay_ms` |
| `POSITION` | trapezoid stop law, or a cosine/quintic segment | retargetable mid-move, lands exactly |
| `VELOCITY` | ramp to `cmd_vel` at `amax` | `STOP` ramps to 0 |
| `JOG` | as velocity, plus a timeout | stops unless refreshed (default 300 ms from the console) |
| `PVT` | cubic Hermite between streamed points | queue of 32 per axis; underrun → safe brake |
| `GROUP` | `group_tick` | individual motion commands refused; `STOP` stops the group |
| `CAM` | `cam_tick`: `f(leader)` | motion commands refused; `STOP` disengages |

### Profiles (per axis)

| Profile | Generator | Filter | Use |
|---|---|---|---|
| `trap` | stop law | none | fastest, step changes in acceleration |
| `scurve` (default, Tj 30 ms) | stop law | 1 moving average | limited jerk |
| `smooth` | stop law | 2 moving averages | continuous jerk |
| `cosine` | fixed-time cosine | none | smooth, symmetric moves from rest |
| `quintic` | minimum-jerk quintic | none | smoothest point to point; retargets with v₀ and a₀ |

---

## 12. Coordinated groups: `group.c`

A group (up to 4) owns a set of axes and moves them along a queue of up to
32 straight **joint-space segments**, so every member starts and finishes
each segment together.

```mermaid
flowchart LR
    L[GROUP_LINE / GROUP_ARC] --> SEG["segment: unit vector u,<br/>length, path vmax/amax"]
    SEG --> JN["junction speed:<br/>per-axis Δv ≤ amax·corner_s"]
    JN --> PLAN["backward pass:<br/>v_entry = min(v_junction, √(v_next² + 2aL))"]
    PLAN --> EXEC["group_tick:<br/>stop law along the path<br/>to v_exit of the next segment"]
    EXEC --> AX["member gen_pos = start + u·s"]
    AX --> FILT[each axis's own s-curve filter]
```

* **Path limits:** the segment's path `vmax`/`amax` is the largest value for
  which no member exceeds its own limits (scaled by its share of `u`).
* **Corners:** the junction speed is capped so that no axis's speed changes
  by more than `amax × corner_s` across the corner.
* **Lookahead:** after every append the queue is re-planned backward, so the
  group can always stop by the end of what it has been given.
* **Execution** uses the same discrete stop law as a single axis, but toward
  the next segment's entry speed. Segments end exactly on their integer
  targets, and leftover travel carries into the next segment.
* **Arcs** (members 0 and 1) are fed in as chords, sized so no chord strays
  further than `tol` from the true circle. Chords are added lazily as the
  queue drains, so an arc of any length fits, and the last point is exact.
* **Hold/resume** decelerates along the path and waits. **Stop** holds, then
  drops the queue once at rest.

---

## 13. Streamed PVT

The host sends `(pos, vel, Δt)` points. Each axis follows a cubic Hermite
between them, landing exactly on each knot position and speed.
`PVT_START` starts only from rest, so several axes can start on the same
tick. If the queue runs dry while moving, the axis brakes at `amax` and
counts an **underrun** (reported as an event). PVT bypasses the s-curve
filters.

---

## 14. Electronic cams and virtual leaders

```mermaid
flowchart LR
    VL1["Virtual leader v1/v2<br/>(own trapezoid ramps)"] --> E
    AXL["or a real axis 1–10"] --> E
    E["cam_eval(table, leader.pos)"] --> PLUS(("+")) 
    OFF[offset] --> PLUS
    BL["engage blend<br/>(smoothstep fade)"] --> PLUS
    PLUS --> F[follower gen_pos<br/>unfiltered]
```

* A **table** holds up to 128 `(x, y)` points in full steps, joined by cubic
  Hermite segments with Catmull-Rom slopes, so it is C¹ and the follower's
  speed is continuous.
* **Cyclic** tables repeat with period `X`, and their net rise `R`
  accumulates each cycle (`R ≠ 0` advances the follower every revolution).
  Cycles are counted in exact integer units, so millions of cycles don't
  drift. **One-shot** tables hold their end values.
* Following is by **position**, so a follower retraces exactly when the
  leader reverses, at any speed.
* **Engage:** core 0 first checks (`cam_check`) that the follower can keep up
  with `v_f = y′·v_l` and `a_f = y″·v_l² + y′·a_l` at the leader's limits.
  This check is too slow for the 1 ms tick, so it doesn't run on core 1.
  Core 1 then fades out the initial position difference over `blend_ms`
  with a smoothstep.
* Two **virtual leaders** are pure trajectory generators with no motor:
  velocity, move, stop, limits and zero.
* There are 4 table slots. Tables are saved in flash by `save` and restored
  at boot.

---

## 15. Shows and the player

A **show** is a binary blob (compiled from JSON by `tools/stepperctl`) with
keyframe tracks for axes and LED pixels on one timeline. It is stored in one
of 4 flash slots, validated with a CRC and checked against the axes' limits.

```mermaid
sequenceDiagram
    participant FL as Flash slot
    participant P as player.c (core 0)
    participant Q as Command queue
    participant C1 as Core 1 axes
    participant L as WS2812
    FL->>P: show_parse
    P->>C1: MOVE to first keys (PREP), wait settled
    P->>Q: PVT points, keeps ~12 queued per axis, ≤16 per poll
    P->>Q: PVT_START (all show axes, same tick)
    loop RUN
        C1-->>P: snapshot pvt_depth
        P->>Q: top up points (round robin, wraps at loop seam)
        P->>L: LED colour = linear fade between keys
    end
```

Axis keys become PVT points, so motion passes exactly through every key.
Automatic speeds come from the neighbouring keys, with zero at the ends of a
non-looping show.

---

## 16. Safety

```mermaid
flowchart TD
    subgraph Ladder["Analog ladder on one ADC pin"]
        L1["3.3 V: e-stop open / cable cut → STOP"]
        L2["2.2 V: idle"]
        L3["1.3 V: Btn1"]
        L4["0.7 V: Btn2"]
        L5["0 V: driver nFAULT → STOP"]
    end
    ADC[ADC round robin, DMA] --> POLL[core 1 poll_ladder<br/>every 10 µs, also between PWM periods]
    POLL -->|3 consecutive stop samples| OFF["hbridge_safe_off all motors (coast)<br/>halt_all: axes, groups, cams<br/>~30 µs"]
    OFF --> LATCH[stop latched; motion commands dropped]
    LATCH -->|"CLEAR (c / Btn1), refused while line still reads stop"| RE[hbridge_restart, phase = pos, amp from 0]
    WD[core 0 watchdog:<br/>no core 1 heartbeat] --> OFF2[safe_off all motors]
```

* **Stop latency** is about 30 µs. The ladder is also polled inside the
  per-period loop, not just once per tick.
* **A held button can hide an open e-stop:** e-stop open + Btn1 reads
  1.65 V and e-stop open + Btn2 reads 0.82 V, both inside the button bands.
  The 1.49–1.76 V band, held for 5 ms, latches an e-stop. Btn2's case is too
  close to Btn2 alone to split, so any button held longer than 3 s latches
  an e-stop (this also catches a stuck button). Neither counts as a press,
  and the stop can't be cleared until the line is back to idle.
* **Core 1 watchdog:** core 0 watches `control_heartbeat`. If it stops (and
  no flash write is in progress), all outputs go to coast.
* **PIO starvation** fails to brake, as described in section 10.
* **Power-up:** the outputs run at zero amplitude until V5 has settled, then
  `control_energized` lets amplitude ramp up at the slew limit.
* **Flash writes** (config, shows, cams) pause core 1 safely through
  `flash_safe_execute`, and only with all axes at rest.

---

## 17. Configuration and persistence

| Item | Where |
|---|---|
| per-axis vmax, amax, profile, jerk_ms | `config_t`, two alternating flash sectors, generation counter + CRC-32 |
| drive curve (amp_low/high/hold, low/high speed), reverse, swap coils | `config_t` |
| hold delay, boot show | `config_t` |
| shows | 4 slots (`show_store.c`) |
| cam tables | `cam_store.c`, saved by `save` |

An interrupted save leaves the previous record valid. At load, the newest
valid record wins.

---

## 18. Interfaces

* **USB text console:** human commands (`m`, `j`, `g`, `gl`, `ga`, `p`, `cam`,
  `ce`, `vl`, `t`, `drive`, `save`, ...). Type `help`.
* **USB binary protocol:** COBS frames delimited by `0x00`, CRC-16. Frames
  share the port with the console, and every request is ACKed with the core 1
  sequence number (spec in `protocol.h`, client in `tools/stepperctl`).
* **I²C target** (Qwiic, address 0x42, RP2354B board): the same requests
  without framing. Untested.
* **Telemetry:** up to 1 kHz, numbered lines, with selectable axes and fields,
  plus events (done, underrun, e-stop, fault, clear).
* **Standalone mode:** a boot show starts at power-up. Btn1 clears a stop or
  starts/stops the show, and Btn2 selects the next show.
* **Status LEDs:** pixel 0 shows overall status (fault, watchdog, bypass),
  and pixels 1–10 show each axis's speed and direction.

---

## 19. Summary: life of one microstep

```mermaid
flowchart TD
    A["host: MOVE axis 3 → 500 steps"] --> B["core 0: control_post (seq N)"]
    B --> C["core 1, tick start: apply → motion_move_to"]
    C --> D["generate: stop law → gen_pos += (v+u)/2·dt"]
    D --> E["scurve moving average → pos += Δ"]
    E --> F["Δ/20 per PWM period → phase += step"]
    F --> G["LUT[phase>>22] → cos/sin × amp × 3738"]
    G --> H["error diffusion → integer duties ≥ 38 counts or 0"]
    H --> I["encode: 4 segments → 2 words"]
    I --> J["DMA ring → TX FIFO → PIO out pins"]
    J --> K["AT8833: drive / brake → coil current ∝ sin, cos"]
    K --> L["rotor follows the rotating field"]
```
