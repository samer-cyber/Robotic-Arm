# Wiring — ESP32-S3 + 3× TMC2209 (UART mode)

Your driver module:

```
        ┌─────────────────────────┐
   VS ──┤                         ├── EN
  GND ──┤                         ├── MS1
   A2 ──┤       TMC2209           ├── MS2
   A1 ──┤                         ├── RX
   B1 ──┤                         ├── TX
   B2 ──┤                         ├── CLK
  VIO ──┤                         ├── STEP
  GND ──┤                         ├── DIR
        └─────────────────────────┘
```

## 1. Power

| From | To | Notes |
|---|---|---|
| PSU 12–24 V | `VS` on every driver | one 100 µF electrolytic across VS/GND **per driver**, as close to the driver as you can get it |
| PSU GND | `GND` on every driver | |
| PSU GND | ESP32 `GND` | **mandatory.** Without a common ground the UART and STEP signals have no reference and you get garbage |
| ESP32 `3V3` | `VIO` on every driver | logic supply, not motor supply |
| USB | ESP32 | the ESP32's only power during bench testing |

Motor coils go to `A1`/`A2` (one coil) and `B1`/`B2` (the other). Get the pairing
right — use a multimeter, the two wires of a coil read a few ohms to each other
and open circuit to the other coil.

Two rules that will save you a driver:

- **Never unplug a motor while VS is live.** The back-EMF spike kills TMC2209s.
- Don't drive logic into an unpowered driver. Bring VIO up with or before VS.

## 2. Step / Dir / Enable

### ESP32-WROOM-32D (default target, `-e wroom`)

| Signal | J1 (base) | J2 (shoulder) | J3 (elbow) |
|---|---|---|---|
| `STEP` | **GPIO25** | **GPIO26** | **GPIO27** |
| `DIR`  | **GPIO32** | **GPIO33** | **GPIO14** |
| `EN`   | **GPIO13** | GPIO13 | GPIO13 |
| UART   | TX **GPIO17**, RX **GPIO16** | | |

Limit switches when you fit them: J1 = 34, J2 = 35, J3 = 36. Those four pins
(34/35/36/39) are **input only with no internal pullups**, so each switch needs
an external 10 kΩ pullup to 3V3.

### ESP32-S3-DevKitC-1 (`-e s3`)

| Signal | J1 (base) | J2 (shoulder) | J3 (elbow) |
|---|---|---|---|
| `STEP` | **GPIO4** | **GPIO5** | **GPIO6** |
| `DIR`  | **GPIO8** | **GPIO9** | **GPIO10** |
| `EN`   | **GPIO14** | GPIO14 | GPIO14 |
| UART   | TX **GPIO17**, RX **GPIO18** | | |

Limit switches: J1 = 38, J2 = 39, J3 = 40, internal pullups are fine.

### Both boards

`EN` is **active low** and all three drivers share one line.

**Put a 10 kΩ pullup from the EN line to 3V3.** The MCU pin floats during
reset, during reflash, and for the first moment after power-on — a floating
enable is an undefined enable. The pullup guarantees the drivers stay off until
firmware deliberately pulls the line low, which is what `en 1` does. You do not
want an arm that goes stiff and twitches every time you upload.

`CLK` — leave it floating, no wire. Every stepstick-style board either pulls it
down on-board or works fine without it. If you want to confirm, measure `CLK` to
`GND` on the unpowered module; a resistor reading means the board handles it.

`CLK` — tie to **GND**. That selects the chip's internal oscillator, which is
what you want. Many modules work with it left floating, but GND is what the
datasheet actually specifies, and it costs you nothing.

## 3. The UART bus

> **Before anything else: `VS` must be connected and powered, or the driver
> will not answer.** `VIO` powers only the I/O pad ring. The chip's digital
> core — including the UART — runs from an internal 5 V regulator fed by `VS`.
> With `VIO` alone the driver is electrically present and logically dead, which
> looks exactly like a wiring fault and is the single most common reason a
> TMC2209 never replies.
>
> Power `VS` with the **motors unplugged** and you get a fully configurable
> driver that physically cannot move anything.

Your board breaks out `RX` and `TX` as separate pads, which is convenient but
slightly misleading: **internally they are the same pin.** The TMC2209 has one
UART pin (`PDN_UART`) and it is half-duplex — one wire carrying both directions.
The two pads exist so the on-board series resistor sits on the right side of the
link. So:

- It is still one shared bus, not three point-to-point links.
- It is still addressed by `MS1`/`MS2` (§4).
- There is still a hard ceiling of **four drivers per bus**.

### First, check the UART pin is connected to anything at all

Many TMC2209 modules ship with `PDN_UART` **not routed to any header pin.** The
board is built to serve two different mainboard conventions and commits to
neither until you bridge a solder jumper.

On the modules used here it is **R7**, a 3-pad selector next to the header:

| Bridge | Routes `PDN_UART` to | |
|---|---|---|
| pad 2 ↔ **pad 3** | **Header Pin 4** | direct, no series resistor — **use this one** |
| pad 2 ↔ pad 1 | Header Pin 5 | through an on-board 20 kΩ |

Pad 2 is the middle one and is the UART net itself. Unbridged, both header pins
are dangling traces that go nowhere.

**Bridge pad 2 to pad 3 on every driver.** Then verify with a meter before you
wire anything: **Pin 4 → GND should read about 20 kΩ**, which is the chip's
internal pull-down on `PDN_UART`. Open circuit means the solder did not take.

That one measurement is worth more than an hour of continuity-checking your
harness, because it proves the path all the way into the chip.

> Symptoms when R7 is unbridged, so you recognise it next time: Pin 4 to Pin 5
> reads open; Pin 4 to GND reads open; `uart loop` passes with a jumper between
> the MCU pins but fails the moment the drivers are wired in — because TX and RX
> land on two isolated floating nets instead of one shared node.

### Then, find out whether your board has the resistor

Put a multimeter across the `RX` and `TX` pads of one driver, board unpowered:

| Reading | What it means | What to do |
|---|---|---|
| **~1 kΩ** | resistor is on board — this is the common case for boards with split RX/TX | wire it straight through, no external parts |
| **~0 Ω** | the pads are just shorted together, no resistor | add **one external 1 kΩ** in series with the ESP32's TX line |

This matters. The resistor is what lets the driver talk back over a wire the
ESP32 is also driving. Getting it wrong — most often by adding a second 1 kΩ
when the board already has one — is the number one reason a TMC2209 never
answers.

### Then wire the bus

With R7 bridged pad 2–3, **Header Pin 4 is the whole bus.** Both MCU lines land
on it — it is one wire electrically, not a TX/RX pair:

```
  MCU TX ──[ 1 kΩ ]──┬──── Pin 4 on J1
                     ├──── Pin 4 on J2
  MCU RX ────────────┴──── Pin 4 on J3
```

One 1 kΩ total, in the TX line, before it fans out. Header Pin 5 goes unused.

TX/RX pins by board: **WROOM** TX = GPIO17, RX = GPIO16. **S3** TX = GPIO17,
RX = GPIO18.

**Which pad is which only matters if the board has the resistor.** If you
measured ~1 kΩ and `probe` finds nothing, swap GPIO17 and GPIO18 — some vendors
label these pads from the MCU's point of view rather than the driver's, which
puts the resistor on the pad you would not expect.

**If you measured ~0 Ω, do not swap them.** The two pads are literally the same
node on your board, so swapping achieves nothing except moving your external
resistor onto the receive line, which breaks it. The 1 kΩ must sit in the
**GPIO17** line. Connect GPIO18 to either pad; it makes no difference.

## 4. Addressing — how three drivers share one wire

Each TMC2209 latches its bus address from `MS1` and `MS2` at power-up. `MS1` is
bit 0, `MS2` is bit 1:

| Axis | Address | `MS1` | `MS2` |
|---|---|---|---|
| J1 | 0 | GND | GND |
| J2 | 1 | **3V3** | GND |
| J3 | 2 | GND | **3V3** |
| (spare) | 3 | 3V3 | 3V3 |

These pins have internal pull-downs so floating reads as 0, but tie the GND ones
to GND anyway — a floating pin next to a stepper motor is an antenna.

**You cannot leave MS1/MS2 unconnected to "skip microstepping".** In UART mode
they are not microstepping pins any more, they are address pins, and without
them all three drivers answer to address 0 and the bus is unusable. See §5 for
where microstepping actually lives now.

Four addresses is the hard ceiling per bus. Axes 5 and 6 need a second UART bus
on GPIO1/GPIO2. See §8.

## 5. Microstepping — set to 1 (full step), as you asked

This is the payoff for wiring UART: **microstepping is a software setting now.**
The firmware sets `mstep_reg_select(true)`, so the value comes from the chip's
`MRES` register rather than from pins. The default in `src/config.cpp` is now:

```c
a.microsteps = 1;      // full step
```

so you get **one step pulse per full step, 200 per motor revolution**, and the
arithmetic stays round. Nothing is wired for it and nothing needs to be. When
you want to change it:

```
ms 0 16          # J1 to 1/16
ms all 32        # everything to 1/32
cfg              # confirm, and see the recomputed steps/deg
save
```

No jumpers, no reflash, no rewiring. Change it mid-session and the firmware
rescales the position counter so the joint angle stays put.

Two things to know about running at full step:

- **Leave `intpol` on** (it is on by default). Interpolation tells the driver to
  smooth each incoming full step into 256 microsteps internally. You still send
  one pulse per full step, but the motor sees a smooth current waveform instead
  of four hard quadrant jumps. Without it, full step on a stepper is loud,
  rough, and prone to resonance. With it, full step is genuinely usable.
- **Your positioning resolution is 1.8° per step** at 1:1 gearing. That is
  coarse for an arm, and it will show up as visible quantisation — the `line`
  command reports endpoint error, and that error will be step quantisation
  almost entirely. `t ik` won't show it (that test is pure maths), so don't be
  surprised when a perfect IK result still lands a few millimetres off. Raising
  microstepping later fixes it with one command.

## 6. Limit switches and DIAG

| Signal | J1 | J2 | J3 | Notes |
|---|---|---|---|---|
| limit switch | **GPIO38** | **GPIO39** | **GPIO40** | switch between the pin and **GND**, internal pull-up is on. LOW = triggered |
| `DIAG` | **GPIO1** | **GPIO2** | **GPIO21** | optional. StallGuard output, for sensorless homing |

Your board's header list doesn't show a `DIAG` pad. Plenty of TMC2209 modules
put it on the underside or leave it off entirely — check yours. If it isn't
there, nothing breaks: set `diag` to `-1` in the `PINS` table in `src/config.h`
and the firmware polls `SG_RESULT` over UART instead. Slower and less crisp, but
sensorless homing still works.

Wire the switches **normally-closed** if you can: a broken wire then reads the
same as a triggered switch, which fails safe.

One real gotcha: **StallGuard4 only works in StealthChop on the TMC2209.**
Switch an axis to SpreadCycle and sensorless homing on it stops working. The
firmware refuses `home <ax> sg` in that state rather than silently ramming the
joint into its endstop.

## 7. Pins deliberately left alone

On an ESP32-S3-DevKitC-1:

| Pin(s) | Why |
|---|---|
| 0, 3, 45, 46 | strapping pins — the level at reset changes how the chip boots |
| 19, 20 | native USB D−/D+ |
| 26–32 | SPI flash |
| 33–37 | octal PSRAM on the `-N16R8` modules |
| 43, 44 | UART0, the USB-serial console |
| 48 | onboard RGB LED on DevKitC-1 v1.1 |

That leaves 1, 2, 4–18, 21, 38–42, 47 — 25 usable pins, which is exactly enough
for six axes and not one more.

## 8. When this becomes a 6-DOF arm

The plan is already in `src/config.h`. Two things change:

**Pins.** DIAG gets dropped and GPIO1/GPIO2 become the second UART bus (axes 5
and 6, addresses 0 and 1). Full six-axis map:

| | J1 | J2 | J3 | J4 | J5 | J6 |
|---|---|---|---|---|---|---|
| `STEP` | 4 | 5 | 6 | 7 | 15 | 16 |
| `DIR` | 8 | 9 | 10 | 11 | 12 | 13 |
| limit | 38 | 39 | 40 | 41 | 42 | 47 |
| UART bus | A (17/18) | A | A | A | B (1/2) | B |
| address | 0 | 1 | 2 | 3 | 0 | 1 |

`EN` stays on GPIO14 for all six. Spare: GPIO21, GPIO48.

**Step generation.** This is the reason the WROOM is the better target of the
two. FastAccelStepper generates step pulses from hardware peripherals, and which
peripheral it gets depends on the chip:

- **ESP32-WROOM-32D** — MCPWM + PCNT, which reaches **six** steppers. Enough for
  the whole arm.
- **ESP32-S3** — RMT channels, of which the S3 has few. The ceiling is lower and
  plausibly below six.

Either way the firmware fails loudly rather than weirdly: if
`stepperConnectToPin()` comes back empty at boot you get a `!!` message naming
the axis.

**Pin budget is the WROOM's weak spot instead.** Six axes needs 6 STEP + 6 DIR +
1 EN + 2 UART = 15 pins, which is exactly every safe output pin the WROOM has.
That leaves only the four input-only pins (34/35/36/39) for six limit switches,
and nothing for a second UART bus. Two ways out when you get there:

1. Put the limit switches on a PCF8574 I2C expander. They are slow signals; you
   can poll the expander at 1 kHz over 400 kHz I2C without noticing, which is
   plenty prompt for homing.
2. Use GPIO 2/5/15 for switches, accepting that a switch closed at power-on will
   stop the board booting, because those are strapping pins.

If you outgrow both boards, the real answer is a single timer-ISR Bresenham step
generator covering all six axes — what GRBL and Marlin do. More code, but it
gives you *properly* coordinated motion instead of the near-enough time-scaling
trick used now, which is what you want for a real arm anyway.

---

# Servos — J4 / J5 / J6 + gripper

| Joint | Servo | Sweep | Signal (WROOM) | Signal (S3) |
|---|---|---|---|---|
| **J4** pitch | MG996R | 270° | **GPIO4** | **GPIO15** |
| **J5** roll | — | 270° | **GPIO21** | **GPIO16** |
| **J6** pitch | MG995 | 270° | **GPIO23** | **GPIO21** |
| **GRIP** | — | 180° | **GPIO19** | **GPIO47** |

Change any of them live: `svset J4 pin 25`, then `save`. The firmware refuses
a pin that `pinWarning()` knows is unusable, so you cannot accidentally put a
servo on the SPI flash bus.

These pins deliberately reuse the slots that the unused J4–J6 **stepper**
entries occupy in `DEFAULT_PINS`. `N_AXES` is 3, so nothing ever drives those
— but if you ever did go back to six steppers, this is the collision to
remember.

## Power — read this one

**Do not take servo power from the ESP32 board's 5V pin.**

An MG996R stalls at roughly **2.5 A**, an MG995 at about **1.4 A**. Four of
them moving together is well past anything a dev board's regulator or USB can
supply. What you get instead is a brownout reset mid-move — and the firmware
*will* tell you, because it prints the reset reason at boot:

```
# reset reason: BROWNOUT - supply dipped...
```

If you see that line, this is why.

| From | To |
|---|---|
| Separate 5–6 V supply, ≥3 A | servo **red** |
| That supply's GND | servo **brown/black** |
| That supply's GND | **ESP32 GND** — mandatory, shared reference or the pulses mean nothing |
| ESP32 GPIO (table above) | servo **orange/yellow** |

Put a few hundred µF across the servo rail near the servos. The inrush when
four horns start at once is what causes the dips.

## Angles

Commanded in **joint degrees with 0 = the middle of travel**. A 270° servo
therefore runs −135…+135 and the 180° gripper −90…+90.

Soft limits start at ±90° for the wrist, deliberately tight. Open them up with
`svset J4 max 120` once you have swept the joint and know where it actually
fouls a bracket. `sweep j4` in `Host/servo_test.py` is the quick way to find
that.

`trim` is the servo's own 0…range position that your joint zero maps onto
(default: half the range). Use it to square a horn up without re-cutting
anything. `inv` flips which way positive goes.

## Why J1–J3 keep ±180°

They are steppers on cycloidal boxes and can turn all the way round, so their
joint limits stay at the full ±180. **The desk is what actually stops them**,
not an angle — and a fixed angle limit is the wrong tool for that, because
whether the arm hits the table depends on where every other joint is.

So instead: `desk` runs the real geometry.

```
desk              show the floor and the current clearance
desk 0            set the work surface to Z = 0
desk 0 15         ...and keep 15 mm of margin
```

It walks the whole chain — J2, J3, J4, J5, J6 and the tool tip — and reports
the lowest point of any of them against `deskZ + deskMargin`. Raise `deskZ` if
the arm sits on a plinth.

## Link lengths

Measured centre-of-rotation to centre-of-rotation:

| | mm | `geo` key |
|---|---|---|
| J2 → J3 (upper arm) | *measure yours* | `l1` |
| J3 → J4 (forearm) | **139.982** | `l2` |
| J4 → J5 | **24.5** | `l3` |
| J5 → J6 | **73.464** | `l4` |
| J6 → tool tip | *guess, 60* | `l5` |

J4 → J6 is 97.964 mm; J5 sits 24.5 mm along from J4, so `l4` is the remainder.

Because `l2` now means J3 → **J4**, `fkPose()` and `ikSolve()` resolve the
**wrist root**, not the tool tip — which is the correct decoupling for a 6-DOF
arm. `fkChain()` walks the servos too and is what `desk` uses.
