# Calibration

Five checks, in order. The first two fix most of the "it reaches the object but
misses by a bit" problem. Everything is typed into the control panel's console
(`python host/control.py`) or any serial monitor at 115200 baud.

## 1. Gear ratios (do this first)

The joints use the inverted cycloidal layout: the roller drum turns, so the
reduction is the **roller count**, not the lobe count.

| Joint | Box | Rollers | Correct `gear` | Firmware <= 1.2.0 used |
|---|---|---|---|---|
| J1 base | "15:1" | 16 | **16** | 15 (6.3 % short) |
| J2 shoulder | "20:1" | 21 | **21** | 20 (4.8 % short) |
| J3 elbow | "15:1" | 16 | **16** | 15 (6.3 % short) |

Firmware 1.3.0 has the right defaults, but your **saved** settings live in the
ESP32's flash and survive a reflash. On the arm as of 2026-11-07 they still say
15/20/15. Fix them:

```
set J1 gear 16
set J2 gear 21
set J3 gear 16
save
```

`set` keeps the current angle and rescales the step count behind it, so nothing
moves. Then prove it: put a tape flag on the base, `enable 1`, `jog J1 90`, and
check it turned a real quarter turn. Before the fix it stopped about 5.6° short.

## 2. Direction and zero, with the twin

Open `http://localhost:8080/twin` (the panel links to it). The grey arm is
the real one, drawn from the CAD and driven by the joint angles the firmware
reports.

1. **Zero.** Put the arm straight up, like the CAD, and `teach all 0`. That is
   the pose where every model angle is 0.
2. **Signs.** `jog J2 10` and watch: if the twin tips the other way, set
   `"sign": -1` for J2 in [`description/arm.json`](../description/arm.json) and
   reload the twin page (the IK picks the change up on its own). Repeat for
   every joint. Signs only describe how the motor was wired and bolted;
   nothing else changes.
3. If you would rather keep a different firmware zero (say, folded), put the
   difference in `offset` instead of re-teaching.

## 3. The gripper (continuous-rotation servo)

The hand's servo has no position: the pulse width sets **speed**, and one
particular width means stop. The firmware treats it as a spool: `grip close`
winds until the fingers stall and then holds with reduced torque, and `grip open`
unwinds for as long as it wound, then **stops**. Unwinding further winds the
tendons the other way and the hand closes again.

```
enable 1
grip                       # shows mode, wind estimate and every setting
grip run 0 2000            # neutral for 2 s: should be silent and still
grip set neutral_us 1490   # if it creeps, nudge this until it does not
grip run 0.5 300           # short pulse: which way did it go?
grip set close_dir -1      # only if +speed OPENED the hand
grip zero                  # with the hand fully open: wind = 0
grip run 1 3000            # count how long a full close takes (empty hand)
grip set close_ms 1100     # ...that time
grip close                 # closes, then holds
grip open                  # should end fully open and silent
grip set open_ms 950       # shorten if it re-closes at the end, lengthen if it stays curled
grip set hold 0.25         # holding torque; lower = cooler servo, weaker grip
save
```

`stop` and `goto` never release what the hand is holding. `grip open` and
E-STOP do.

The wind estimate is dead reckoning. A large object stalls the fingers early,
so the estimate over-counts, and `open` can leave the fingers slightly curled
afterwards. A positional or feedback servo (STS3215) removes the guessing; see the
[roadmap](roadmap.md).

## 4. Tool point

`tcp.position` in `arm.json` is a guess (the hand is not in the CAD). Jog to
touch a known point, such as the desk at a measured spot, compare it with the
twin's "arm tool point" readout, and correct it. IK aims this point, so every
millimetre here shows up at the fingertips.

## 5. Camera to arm (only for vision)

See [`ros2/README.md`](../ros2/README.md) for the overhead OAK-D SR setup and
`calibrate_extrinsics`. That pipeline predates the wrist and still uses the old
3-DOF geometry, so redo steps 1-4 first. A camera calibrated against an arm
with wrong gear ratios bakes the ratio error into the camera transform.
