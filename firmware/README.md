# arm-fw — joint controller firmware (ESP32)

Joint-space only: it owns the joints, turns degrees into steps and pulses,
enforces limits, and never claims to have done something it did not.
Kinematics run on the host.

- ESP32-WROOM-32 (`esp32dev`), Arduino framework, PlatformIO
- 3 steppers through TMC2209s (STEP + UART, direction over UART), FastAccelStepper
- 3 hobby servos + a continuous-rotation gripper on LEDC PWM
- line protocol over USB at 115200 baud

## Build and flash

```bash
pio run -t upload           # from this folder
pio device monitor          # then type: help
```

Flash **before** handing the USB port to WSL. A device attached to WSL is
invisible to Windows.

## Protocol

One command per line. Every command ends with exactly one `ok` or
`err <code> <name> <detail>`. Data lines start with `< ` (key=value), notes with
`# `, and unsolicited events with `! ` (e.g. `! idle`). Nothing blocks: moves
return at once, so a host can always send `stop`.

| Command | |
|---|---|
| `info`, `state`, `config [j]`, `pins` | inventory, positions, settings, pin audit |
| `enable 0\|1`, `stop`, `estop`, `estop clear` | power and safety |
| `move <j> <deg> [feed]`, `jog ...`, `goto <d1..d7> [feed]` | motion; `-` skips a joint in `goto` |
| `teach <j\|all> <deg>` | redefine where a joint is, no motion |
| `set <j\|all> <key> <value>`, `save`, `load`, `defaults` | tuning, kept in flash |
| `grip`, `grip close [power]`, `grip open`, `grip stop`, `grip run <speed> [ms]`, `grip zero`, `grip set <key> <value>` | the continuous-rotation gripper (1.3.0) |
| `probe`, `driver [j]`, `bus echo\|scan`, `diag joint\|units\|skip ...` | TMC2209 and step-path diagnostics |

`help` on the device lists everything with arguments.

## 1.3.0

- **Gear ratios 16 / 21 / 16** (were 15 / 20 / 15). The boxes are inverted
  cycloids, so the ratio is the roller count. Saved settings override the
  defaults: see [docs/calibration.md](../docs/calibration.md) step 1.
- **Gripper** as a continuous-rotation servo: timed close-then-hold, timed
  open-then-stop, every drive capped at `max_run_ms`. Settings live in their
  own flash key, so adding them did not discard anyone's joint config.
  `stop` never releases a held object; `grip open` and E-STOP do.
- GRIP's position is now the wind estimate in percent (0 open, 100 closed).

Earlier history: [docs/notes/direction-over-uart.md](../docs/notes/direction-over-uart.md) (1.2.0).
