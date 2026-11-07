# 3-DOF arm test rig — ESP32-S3 + TMC2209

A serial console for poking at a stepper arm: drive joints, read and rewrite
every TMC2209 feature at runtime, run inverse kinematics, and run tests that
tell you something you did not already know.

- [docs/WIRING.md](docs/WIRING.md) — how to connect it. Read first.
- [docs/REFERENCE.md](docs/REFERENCE.md) — **every command, every setting, every
  field of every report, and how the code works.** The manual.
- [tools/arm.py](tools/arm.py) — interactive crane-style jog control from your
  laptop. `python tools/arm.py`

## Toolchain

**PlatformIO in VS Code.** You already have VS Code — install the "PlatformIO
IDE" extension, open this folder, hit Build. It handles the ESP32-S3 toolchain
and pulls `TMCStepper` and `FastAccelStepper` on its own.

Arduino IDE works too and there is nothing exotic in this code, but you would
have to flatten `src/` into a sketch folder, install both libraries by hand, and
set board options through menus instead of `platformio.ini`. Since this is
multi-file already and will grow, PlatformIO is less friction, not more.

Which USB socket you plug into matters:

- **UART socket** (the CH343/CP2102 bridge) — default, `ARDUINO_USB_CDC_ON_BOOT=0`.
  Use this. The port stays put across resets, which matters a lot once a host
  script is holding it open.
- **USB socket** (native) — faster, but it re-enumerates on every reset and your
  script loses the handle. Set `ARDUINO_USB_CDC_ON_BOOT=1` if you want it.

## Two different terminals — do not mix them up

**The PlatformIO terminal** runs `pio` commands on your PC. Build, upload,
monitor. Shell commands go here.

**The serial monitor** is a chat session with the firmware running on the MCU.
`probe`, `stat`, `t ik` and everything else in this README are typed *there*.
They are not shell commands and will do nothing in the PlatformIO terminal.

Open the monitor with `Ctrl+Alt+S`, or:

```bash
pio device monitor -e wroom
```

Then **click inside the monitor panel** so it has keyboard focus, type `probe`,
press Enter. `Ctrl+C` closes it. The monitor holds the COM port open, so close
it before uploading — PlatformIO usually handles that itself, but if an upload
fails with a port-busy error, that is why.

If typing still does nothing, PuTTY works reliably: Connection type Serial,
your COM port, speed 500000.

## Bring-up order

Do not skip ahead. Each step is a checkpoint.

1. **Wire VIO, GND, the UART bus, and VS. Leave the motors unplugged.**

   **A TMC2209 cannot talk over UART without VS connected.** `VIO` only powers
   the I/O pad ring; the chip's digital core runs off an internal 5 V regulator
   fed from `VS`. No motor supply, no logic, no UART reply — the driver looks
   completely dead and you will hunt the wiring for an hour.

   Powering `VS` with no motors plugged in is the safe middle ground: the
   drivers are fully alive for configuration, and there is physically nothing
   for them to turn. Open-load flags (`ola`/`olb`) will be set in `tmc`, which
   is correct and expected with no coils attached.

   Plug the motors in only when you reach step 7, **with the supply switched
   off**. Never connect or disconnect a motor while `VS` is live.

2. Flash, open the monitor at **500000 baud**, type `probe`.
   All three must say `ONLINE`. If not, see §3/§4 of WIRING.md — almost always
   `VS` missing, the 1 kΩ resistor on the wrong line, or a missing MS1/MS2 strap.
3. `tmc` — full decoded dump. Version should read `0x21`.
4. **Set what motion needs.** Current and mechanical ratios only — geometry can
   wait, because joint moves do not use it:
   ```
   cur all 500             # start low. raise it only if the motor skips.
   set all gear 1.0        # your actual reduction per joint
   save
   ```
   Microstepping already defaults to **1 (full step)**, 200 pulses per motor
   revolution. It is a software setting — `ms all 16` whenever you want more
   resolution, no rewiring. See WIRING.md §5 for what full step costs you.
5. **Switch the motor supply off, plug the motors in, switch it back on.**
   Then position the arm by hand while it is still limp, and:
   ```
   en 1                    # motors go stiff
   zero                    # call this pose 0,0,0
   mv 0 90 0.3             # quarter turn on the base
   ```
   Check each joint moves the direction you expect. If not, `set N inv 1`.
   `estop` kills everything instantly.
6. **Play with the driver.** This is the part UART bought you — every one of
   these takes effect immediately, no reflash:
   ```
   tmc 0                   # decoded status: temp, open load, actual current
   mon 200                 # live telemetry, any key stops
   sc 0 0                  # SpreadCycle: louder, more torque at speed
   sc 0 1                  # StealthChop: near silent
   ms 0 16                 # microstepping, live
   cur 0 800               # current, live
   ```
7. **Now measure the arm** and set the geometry — this is what IK needs:
   ```
   geo l1 150              # shoulder to elbow, mm
   geo l2 150              # elbow to tool tip, mm
   geo bh 100              # height of the shoulder axis
   geo bo 0                # base axis to shoulder axis, horizontal
   set 1 min -20           # real travel limits per joint
   set 1 max 135
   save
   ```
8. `t ik 2000` — pure maths, nothing moves. Confirms the solver agrees with the
   FK model. If cartesian error is not ~0, your geometry or angle conventions
   are wrong and every move after this would be wrong too. Then `t reach 2000`
   for how much workspace your limits leave you.
9. `gt 200 0 150` then `line 150 0 150 40 0.4` — IK-driven motion.
10. `t speed 0` with the joint free to spin — finds the real step-rate ceiling.
11. `home 0`, then `t rep 0 20` — homing repeatability, once switches are
    fitted. This is your actual mechanical accuracy floor, and it is usually
    worse than people expect.

`estop` at any time kills motion and drops the enable line. Any keystroke aborts
a running test.

## Protocol

One command per line. Every command answers with zero or more `# ...` lines and
then exactly one `ok` or `err <reason>`. That is what makes it equally fine to
type by hand and to drive from a script — a host just reads until `ok`/`err`.

Type `?` for the full command list.

## Host software: Python now, ROS 2 later

**Use a Python script over pyserial for now.** At three joints, on a bench, with
a serial console this direct, ROS 2 gets you nothing you cannot already do — and
it costs you a build system, a workspace layout, URDF authoring, launch files,
and a debugging surface where every problem could be in any of six places. You
would spend the first week fighting `colcon` instead of tuning your arm.

**But keep ROS 2 on the table, because the day it earns its keep is real.** It
becomes worth it when you want MoveIt 2 — collision-aware planning, trajectory
generation, an actual 6-DOF pose goal instead of a wrist position. Writing that
yourself is a much bigger job than learning ROS 2.

The protocol above is designed so that switch is cheap. When you get there you
write one node that owns the serial port, publishes `JointState` from `stat`,
and accepts `FollowJointTrajectory` by feeding `j` commands. The firmware does
not change at all. So: learn ROS 2 on your own schedule, not on the critical
path of getting the arm moving.

## Layout

| File | |
|---|---|
| `src/config.h` | pin map, per-axis config struct, 6-axis expansion plan |
| `src/config.cpp` | **defaults you need to edit**, NVS save/load |
| `src/kinematics.h` | FK/IK, header-only. Swap this block for a wrist later |
| `src/drivers.cpp` | TMC2209 setup and the decoded status dump |
| `src/motion.cpp` | step generation, coordinated moves, homing, limit guard |
| `src/tests.cpp` | the test routines |
| `src/console.cpp` | line protocol and command dispatch |

## Known rough edges

- **Not yet compiled.** Written against the TMCStepper and FastAccelStepper
  APIs but never built — expect to fix a signature or two on first build.
- Coordinated moves time-scale each axis so they arrive together. Accel phases
  still differ slightly, so the joint-space path is near-straight, not exactly
  straight. Fine for testing; a real DDA is the fix.
- `line` runs each segment as a discrete move with a full stop between knots.
  Correct, and good for checking geometry, but it will not be smooth.
- Homing and the test routines block. They poll for a keystroke and abort, so
  you cannot get stuck, but `stop` from a host script will not reach them
  mid-run.
- Everything is open loop. `t rep` measures repeatability against a switch;
  nothing measures whether a move actually arrived.
