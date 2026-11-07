# Reference

Everything this thing does, what each number means, and how the code is put
together. Written to be read out of order — jump to what you need.

---

# 1. How the pieces fit

```
   your laptop                          the arm
   ───────────                          ───────
   serial monitor  ─┐
                    ├─ USB ─► ESP32 ─ STEP/DIR ─► 3× TMC2209 ─► motors
   tools/arm.py    ─┘                 └─ UART ──►  (config + status)
```

Two completely separate conversations happen over different wires:

- **STEP/DIR** — one pulse per step, one pin for direction. This is *motion*.
  Fast, dumb, one-way. The driver has no idea where the arm is.
- **UART** — a data bus to the driver chips. This is *configuration and
  telemetry*: current, microstepping, chopper mode, temperature, stall
  detection. Nothing about position travels here.

The ESP32 is the only thing that knows where the arm is, and it knows only by
counting the pulses it sent. Nothing measures the actual arm. That is what
"open loop" means, and it is why a skipped step is invisible and permanent
until you re-home.

---

# 2. The protocol

One command per line. Every command replies with zero or more `#` comment
lines, then exactly one final line: **`ok`** or **`err <reason>`**.

```
gt 150 0 200 0.3
# xyz 150.00 0.00 200.00 (elbow up) -> q 0.000 58.493 -89.121
ok
```

That shape is deliberate. A human reads the `#` lines; a script reads until it
sees `ok` or `err` and never has to guess when a reply is finished.

- Case-insensitive. `STAT`, `stat`, `Stat` are the same.
- Arguments are separated by spaces.
- **`#` starts a comment** — the rest of the line is ignored, so you can paste
  annotated blocks of commands safely.
- Axes are `0`/`1`/`2` or `J1`/`J2`/`J3`, and most commands also accept `all`.
- CR, LF or CRLF all work as Enter.

---

# 3. Commands

## 3.1 Looking around

### `?`
Prints the command list. Same information as this section, compressed.

### `ver`
Firmware version, build timestamp, axis count, baud rates, sense resistor.
Useful for confirming a reflash actually took.

### `stat`
Where the arm is right now.

```
# state    ENABLED, idle
# J1        16.800 deg      140 steps  vmax 30.0 deg/s
# J2        80.100 deg      890 steps  vmax 30.0 deg/s
# J3       -10.080 deg      -84 steps  vmax 30.0 deg/s
# tool     x 50.89  y 15.36  z 318.06  mm
```

- **state** — `ENABLED`/`disabled` is the driver enable line; `MOVING`/`idle`
  is whether steps are being generated.
- **deg** — where the firmware *believes* each joint is. Belief, not
  measurement.
- **steps** — the raw pulse counter behind that belief.
- **tool** — forward kinematics applied to those angles. Only meaningful once
  `geo` matches your real arm.
- `[LIMIT]` appears if a limit switch is currently pressed.
- `[DRIVER OFFLINE]` means that chip is not answering on the UART bus, and is
  therefore running on power-up defaults rather than your settings. Always
  worth fixing before anything else.

### `cfg`
Dumps every per-axis setting as a table, plus geometry and reach. The left
column is the key name you pass to `set`.

The most useful row is the last one, **steps/deg** — the conversion between the
angles you type and the pulses that get sent. If that number is wrong, every
move is wrong by the same factor.

### `geo`
With no arguments, prints the arm's dimensions. See §4.2.

---

## 3.2 Moving

Everything below refuses to run unless `en 1` has been issued.

### `en <0|1>`
Driver enable. `en 1` energises the motors — they go stiff and hold position.
`en 0` releases them.

The enable line is shared by all three drivers and is active-low, with a
pull-up so the motors stay off during reset and reflash.

### `mv <axis> <degrees> [feed]`
**Relative** move on one joint. `mv 0 10 0.3` turns J1 ten degrees further at
30% of its configured max speed.

`feed` is `0..1`, a fraction of `vel`. Leave it out for full speed. Use small
values while you still trust nothing.

### `mva <axis> <degrees> [feed]`
**Absolute** move on one joint. `mva 0 0 0.3` returns J1 to zero regardless of
where it is now.

### `j <q1> <q2> <q3> [feed]`
**Coordinated** move — all three joints start together and arrive together.

This is not the same as three separate `mva` commands. Each axis is slowed in
proportion to how far it has to travel, so the joints sweep smoothly together
instead of the fast one finishing early and the arm lurching through a corner.

### `hz <axis> <steps/s>`
Free-run at a raw step rate, ignoring joint limits. Negative reverses. Runs
until `stop`. This is a low-level tool for finding out what the motor can
physically do — not a way to move the arm.

### `stop` / `estop`
`stop` decelerates normally. `estop` halts instantly and drops the enable line,
so the arm goes limp. Use `estop` when something is wrong; use `stop` when you
just want it to finish.

### `wait`
Blocks until all motion finishes, then replies `ok`. Meaningless by hand;
essential in a script, where it is how you say "don't send the next move yet".

### `zero [axis|all]` and `pos <axis|all> <deg>`
**Neither of these moves anything.** They change what the firmware believes.

`zero` declares the current physical position to be 0°. `pos` declares it to be
whatever angle you give.

Without limit switches this is your only way to establish a reference: park the
arm somewhere you can measure, then tell the firmware what it is looking at.
Get this wrong and every IK result is offset by the same error.

### `home <axis|all> [sg]`
Drives toward the limit switch, backs off, creeps in again slowly, and sets the
position to `hdeg`. The slow second approach is what makes the trip point
repeatable.

`home 0 sg` uses StallGuard instead of a switch — it drives the joint until the
motor load spikes. No extra hardware, but it needs threshold tuning and it does
run the joint into its own end stop, so tune it gently.

---

## 3.3 Kinematics

Forward kinematics = *joints in, position out*. Always solvable, always
unique.

Inverse kinematics = *position in, joints out*. Sometimes impossible (out of
reach), sometimes ambiguous (two elbow solutions), and occasionally degenerate.

### `fk [q1 q2 q3]`
With no arguments, where the tool is now. With three angles, where the tool
*would* be at those angles. Costs nothing, moves nothing.

### `ik <x> <y> <z> [up|dn]`
Solve for joint angles without moving. Always run this before `gt` when you are
unsure — it shows you what the arm is about to be asked to do.

If it fails it tells you which kind of failure:
- `unreachable` — geometrically impossible, the point is outside the annulus.
- `violates joint limits` — the maths works but a joint would exceed `min`/`max`.
  It prints the unclamped solution so you can see how far off you were.

`up`/`dn` picks the elbow branch (§6.3).

### `gt <x> <y> <z> [feed]`
IK, then go there. The joints move in a coordinated joint-space move, so the
**tool does not travel in a straight line** — it takes whatever curved path the
joints produce. That is fine for getting somewhere; use `line` when the path
itself matters.

### `line <x> <y> <z> [segments] [feed]`
Straight line in Cartesian space. Chops the path into `segments` knots, runs IK
on each, and drives through them in order.

It dry-runs the whole path first and refuses to start if any knot fails — much
better than discovering the problem halfway. On completion it reports endpoint
error, which at full step is almost entirely step quantisation.

Each segment is a discrete move with a stop between knots, so it is accurate
but not smooth. It is a geometry test, not a motion profile.

---

## 3.4 Driver control

Every one of these takes effect immediately, on a running motor, with no
reflash and no rewiring. This is the entire reason for wiring UART.

### `tmc [axis]`
Full decoded status. See §5 for what every line means.

### `probe`
Re-tests the UART link to every driver and reconfigures any that reappear.
First thing to try whenever something behaves strangely.

### `uart loop | scan | pins`
Low-level bus diagnostics that bypass the driver library entirely.

- `uart loop` — sends four bytes and checks they come back. Because TX and RX
  share one node, the ESP32 *must* hear itself. Passing proves the pins,
  resistor and wiring; failing proves the fault is on the MCU side and the
  drivers are innocent.
- `uart scan` — hand-built read frame to addresses 0–3, dumping raw hex.
- `uart pins` — which GPIOs and which UART peripheral are in use.

### `cur <axis|all> <mA> [holdmult]`
RMS current per motor, in milliamps. `holdmult` (0–1) is how much of that to
keep at standstill; 0.5 halves it.

Start below your motor's rating and raise it only if the motor skips. More
current means more torque and more heat, in that order.

### `ms <axis|all> <1|2|4|8|16|32|64|128|256>`
Microsteps per full step. `1` is full step.

Higher values mean smoother, quieter motion and finer resolution, at the cost
of more pulses per degree — which caps your top speed. The firmware rescales
the position counter when you change this, so the joint angle stays put.

### `sc <axis|all> <0|1>`
Chopper mode. **`1` = StealthChop** (near silent, less torque, especially at
speed). **`0` = SpreadCycle** (audible, more torque). For a geared arm fighting
friction, SpreadCycle is usually the right answer.

### `tpwm <axis|all> <tstep>`
Automatic changeover: StealthChop below this speed, SpreadCycle above it. Lets
you have quiet at rest and torque when moving. `0` disables the switch.

The units are `TSTEP`, which is *inversely* proportional to speed — bigger
number means slower threshold.

### `sg <axis|all> <0..255>`
StallGuard threshold. The driver reports a load value; when it drops below
roughly `2 × sg`, the driver calls it a stall.

**StallGuard only works in StealthChop on the TMC2209.** Switch to SpreadCycle
and sensorless homing on that axis stops working. The firmware refuses rather
than ramming the joint into a stop.

### `cool <axis|all> <semin> <semax>`
CoolStep — automatic current reduction when the motor is lightly loaded, saving
heat. `semin 0` disables it.

### `sgr [axis]`
Streams live load values. Press any key to stop. This is how you pick an `sg`
threshold: watch the number while the joint moves freely, then while you gently
load it, and choose something between.

### `mon [ms]`
Streams joint angles, tool position and per-driver telemetry. Any key stops it.

The best way to *feel* what a setting does: start `mon 200`, then flip
`sc 0 0` / `sc 0 1` and watch and listen.

---

## 3.5 Configuration and persistence

### `set <axis|all> <key> <value>`
Changes one setting. Keys are listed in §4.1 and shown by `cfg`.

Settings that live inside the driver chip (current, microstepping, chopper
mode) are written immediately. If a driver is offline you get a warning saying
the value was stored locally but never reached the chip.

### `geo <key> <value>`
Sets an arm dimension. Keys in §4.2.

### `save` / `load` / `defaults`
`save` writes everything to flash so it survives power cycles. `load` restores
it. `defaults` resets to the compiled-in values — which are placeholders, not
your arm.

**`save` after anything you want to keep.** Reflashing does not erase saved
config, but it also does not create it.

---

## 3.6 Tests

All of them abort on any keystroke.

### `t ik [n]`
Generates `n` random valid joint poses, runs each through FK to get a position,
runs IK back, and compares. **Nothing moves.**

This checks the maths against itself. Cartesian error should be effectively
zero. If it is not, your geometry or angle conventions are inconsistent and
every IK move would inherit that error. Run it after any change to `geo`.

"Branch flips" are not errors — they count how often IK picked the other elbow
solution, which is legitimate.

### `t reach [n]`
Samples random points in a box around the base and reports how many are
reachable, how many are blocked by joint limits, and how many are simply
outside the arm's geometry.

A low percentage is normal, especially when `l1` and `l2` differ a lot — the
workspace is an annulus with a dead zone in the middle. What this really tells
you is whether your **soft limits** are throwing away workspace unnecessarily.

### `t speed <axis> [maxHz] [stepHz] [dwellms]`
Walks the step rate up in increments and, at each rung, reports both what was
commanded and what was actually *delivered* — measured from the pulse counter.

That second number is the point. It catches "I asked for 20 kHz and got 6", and
it stops as soon as the delivered rate falls behind, reporting the highest clean
rate in Hz, deg/s and motor rev/s.

**The axis free-runs.** Make sure it can spin without hitting anything.

### `t sweep <axis> <cycles> <degA> <degB> [feed]`
Bounces between two angles, reporting driver flags every cycle. This is a
thermal and endurance soak — it is how you find out 1.4 A was optimistic before
you smell it. Aborts on thermal shutdown.

### `t rep <axis> <cycles>`
Homing repeatability. Homes once, then repeatedly backs off and creeps back in,
recording where the switch tripped **without** rezeroing.

The spread between trips is your real mechanical accuracy floor. No amount of
microstepping beats a sloppy switch or a loose pulley, and this number tells you
which problem you actually have. Needs limit switches.

---

# 4. Every setting

## 4.1 Per-axis (`set <axis> <key> <value>`)

| Key | Meaning | Notes |
|---|---|---|
| `steps` | motor full steps per revolution | 200 for a 1.8° motor |
| `us` | microsteps | same as `ms` |
| `gear` | reduction **after** the motor | 15 means 15:1. Gets this wrong and every move is off by that factor |
| `inv` | flip direction | `1` reverses. Applied inside the driver chip |
| `min` / `max` | soft limits, degrees | the only thing protecting your mechanics from a bad IK result |
| `hdeg` | joint angle when the home switch trips | |
| `hdir` | which way to seek home | `-1` or `1` |
| `vel` | max speed, deg/s **at the joint** | after gearing |
| `acc` | max acceleration, deg/s² | |
| `hvel` | homing speed, deg/s | |
| `ma` | run current, mA RMS | same as `cur` |
| `hold` | standstill current as a fraction of `ma` | 0.5 is a good default |
| `sc` | `1` StealthChop, `0` SpreadCycle | same as the `sc` command |
| `intpol` | interpolate to 256 microsteps | leave on — it is what makes full step smooth |
| `sg` | StallGuard threshold | |
| `tcool` | speed threshold for CoolStep and StallGuard | |
| `tpwm` | StealthChop→SpreadCycle changeover | `0` = never |
| `semin`/`semax` | CoolStep window | `semin 0` = off |

**steps/deg** = `steps × us × gear ÷ 360`. This single number converts
everything you type into pulses.

## 4.2 Geometry (`geo <key> <value>`, all mm)

| Key | Meaning |
|---|---|
| `bh` | height of the shoulder axis above the work surface |
| `bo` | horizontal offset from the base rotation axis to the shoulder axis |
| `l1` | shoulder pivot to elbow pivot |
| `l2` | elbow pivot to the tool point |
| `elbow` | default IK branch, `up` or `dn` |

`l2` defines *which point on the tool* IK controls. Change it and you are
controlling a different point.

---

# 5. Reading a `tmc` report

```
--- J1  (addr 0, bus 0) ---
  version    0x21   IFCNT 22   GSTAT 0x01
  current    run 600 mA (set) / 305 mA (CS=9 now)  hold x0.50
  microstep  1   intpol 1
  mode       StealthChop   (DRV_STATUS.stealth=1)   TPWMTHRS 0
  coolstep   semin 0 semax 2   TCOOLTHRS 1048575
  stallguard SGTHRS 80   SG_RESULT 0   TSTEP 1048575
  DRV_STATUS 0xC0090000
  flags      standstill=1  otpw=0 ot=0  t120=0 t143=0 t150=0 t157=0
             short2gnd A=0 B=0  short2vs A=0 B=0  openload A=0 B=0
```

| Field | What it means |
|---|---|
| `version 0x21` | the chip identifying itself as a TMC2209. Anything else means you are not talking to what you think |
| `IFCNT` | counts successful UART writes. Watch it increase to prove writes are landing |
| `GSTAT 0x01` | bit 0 = "I have been reset". Normal after power-up, clears when read |
| `run X / Y (CS=n now)` | X is what you asked for; Y is what the chip is delivering **right now**. At standstill Y is the hold current, so seeing half your setting is correct, not broken |
| `microstep` | what the chip is actually doing. If this disagrees with `cfg`, the driver never got configured |
| `stealth=1` | the chip confirming its own chopper mode |
| `SG_RESULT` | load measure. High = unloaded, falling = loading up, near zero = stalled. Meaningless at standstill |
| `TSTEP` | time between steps. Huge (1048575) means not moving |
| `standstill` | no steps recently, hold current is active |
| `otpw` | **over-temperature pre-warning.** Your cue to back off current or add cooling |
| `ot` | over-temperature shutdown. The driver has switched itself off |
| `t120/t143/t150/t157` | temperature thresholds in °C, progressively worse |
| `short2gnd`, `short2vs` | short-circuit detection on the motor outputs. Any `1` here means stop and check wiring |
| `openload A/B` | no motor coil detected. Expected with motors unplugged; suspicious otherwise |

---

# 6. Concepts

## 6.1 Microstepping and interpolation

A stepper has 200 physical detents per revolution. Microstepping subdivides
each by shaping the coil currents, so 16 microsteps means 3200 pulses per
revolution and much smoother motion.

The cost is pulse rate. At 256 microsteps you need 256× the pulses for the same
speed, and eventually the step generator runs out of headroom.

**Interpolation (`intpol`)** is the way out. The driver takes your coarse pulses
and internally smooths each one into 256 microsteps. You send one pulse per full
step — nice round numbers, low rate — and the motor still sees a smooth current
waveform. This is why full step is perfectly usable here.

## 6.2 StealthChop vs SpreadCycle

Two ways of driving current through the coils.

**StealthChop** is voltage-mode and near-silent, but it gives up torque,
increasingly so as speed rises. **SpreadCycle** is current-mode: audible, but
stronger and much better behaved at speed.

Quiet demos want StealthChop. A geared arm fighting gearbox friction wants
SpreadCycle. `tpwm` lets you have both, switching automatically at a speed you
choose.

## 6.3 The two elbow solutions

Any reachable point (except right at the edge) can be reached two ways: elbow
bent up, or elbow bent down. Both put the tool in exactly the same place.

`ELBOW_UP` picks the default; `ik ... up` / `ik ... dn` override it per call.
Which one is usable depends on your mechanics and joint limits — one of them
often folds the arm into itself.

## 6.4 Singularities

At certain configurations, IK stops having a unique answer.

The obvious one here: with the tool directly above the base rotation axis,
`q1 = atan2(0, 0)`. Spin the base a full turn and the tool does not move, so
*every* base angle is a correct answer.

Forward kinematics is perfectly happy there. It is the inverse that breaks.

The practical consequence is not the exact point but the region *near* it: a
tiny tool movement demands a huge base rotation, so a `line` passing close to
the base axis makes J1 whip around at full speed. Plan paths that stay away
from it.

## 6.5 Coordinated motion

When several joints move at once, the naive approach — each at its own top
speed — means the short move finishes first and the arm lurches.

`j` and `gt` instead compute how long the *longest* move needs, then scale every
other axis so they all take exactly that long. Start together, arrive together.

The acceleration phases still differ slightly, so the joint-space path is very
nearly straight rather than exactly straight. Fine for testing; a proper motion
planner is the real fix.

## 6.6 Open loop

Nothing measures the arm. Position is purely a count of pulses sent.

If a motor skips — too little current, too much acceleration, a collision — the
count and reality diverge silently and permanently. Everything afterwards is
wrong by that amount, and the only cure is re-homing.

This is why `t rep` matters, why current and acceleration are worth tuning, and
why encoders exist.

---

# 7. The code

| File | Job |
|---|---|
| `src/config.h` | pin maps for both boards, the per-axis config struct, the 6-axis expansion plan |
| `src/config.cpp` | default values, and saving/loading them to flash |
| `src/kinematics.h` | FK and IK. Header-only, and the block to replace when you add a wrist |
| `src/drivers.cpp` | TMC2209 setup, the decoded status report, and the raw UART diagnostics |
| `src/motion.cpp` | step generation, coordinated moves, homing, the limit-switch guard |
| `src/tests.cpp` | the `t ...` routines and `line` |
| `src/console.cpp` | line parsing and command dispatch |
| `src/main.cpp` | start everything, then loop over console and motion |

## 7.1 Where the important logic lives

**`Motion::stepsPerDeg()`** — `steps × microsteps × gear ÷ 360`. The single
conversion everything else depends on.

**`Motion::moveJointsDeg()`** — the coordinated move. Computes each axis's
travel, finds the slowest, then scales every axis's speed and acceleration by
the same ratio so they finish together.

**`Motion::poll()`** — the limit-switch guard, called from the main loop. Stops
an axis only if it is moving *into* a pressed switch, so backing off home does
not trip it.

**`Drivers::ensure()`** — called before any write to a chip. If a driver has
dropped off the bus it re-probes and fully reconfigures it. This exists because
an earlier version silently discarded settings for offline drivers, which cost
an evening.

**`ikSolve()`** — closed-form two-link solution. Computes the elbow angle from
the law of cosines, then the shoulder angle from the difference of two
`atan2` terms.

**`Console::poll()`** — accumulates characters until a line ending, splits on
whitespace, stops at `#`, dispatches.

---

# 8. Things that have already gone wrong

Recorded so they are recognisable next time.

| Symptom | Cause |
|---|---|
| Drivers never answer on UART | `PDN_UART` not routed to any header pin. The **R7 solder jumper** must be bridged (pad 2–3) |
| Drivers never answer, part two | **`VS` was not connected.** The chip's logic runs off an internal regulator fed by the motor supply. `VIO` alone is not enough |
| `uart loop` passes with a jumper, fails with drivers wired | TX and RX landed on two isolated nets, because R7 was unbridged |
| Joint moves ~250× less than commanded | Driver marked offline at boot, so it never got configured and stayed at its 256-microstep default while the firmware assumed full step |
| `microstep 256` after setting 1 | TMCStepper spells full step `0`, not `1`. `microsteps(1)` silently did nothing |
| Typing did nothing in the monitor | Terminal sent CR, parser only accepted LF |
| Pasted commands fused into nonsense | Serial RX buffer overflowed while the firmware was printing, dropping newlines |
| `fk` reported 0 0 0 | A pasted trailing comment was parsed as three joint angles |
