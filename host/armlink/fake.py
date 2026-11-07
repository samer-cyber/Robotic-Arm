"""A simulated controller that speaks the same protocol as the firmware.

Two uses:
  * develop and test the host stack with no hardware plugged in
  * prove that a bug is in the host, not the arm, by running the same code
    against a controller that is known-correct by construction

It models joint motion with a linear ramp, so `moving` and `! idle` behave the
way the real thing does. It deliberately does NOT model lost steps: the
simulator is the reference, and any divergence from it on real hardware is
mechanical.
"""

from __future__ import annotations

import time

FW_VERSION = "1.3.0"

JOINTS = [
    # name, kind, gear, min, max
    # Inverted cycloidal boxes: ratio = roller count (16 / 21 / 16).
    ("J1", "stepper", 16.0, -180.0, 180.0),
    ("J2", "stepper", 21.0, -120.0, 120.0),
    ("J3", "stepper", 16.0, -120.0, 120.0),
    ("J4", "servo",    1.0, -120.0, 120.0),
    ("J5", "servo",    1.0, -120.0, 120.0),
    ("J6", "servo",    1.0, -120.0, 120.0),
    # Continuous-rotation servo on a tendon spool. "Position" is the firmware's
    # wind estimate in percent: 0 = open, 100 = closed.
    ("GRIP", "servo",  1.0,    0.0, 100.0),
]

GRIP_DEFAULTS = dict(continuous=1, neutral_us=1500, span_us=500, close_dir=1,
                     close_power=1.0, close_ms=1200, hold=0.30, open_ms=1000,
                     max_run_ms=4000)


class _Joint:
    def __init__(self, name, kind, gear, lo, hi):
        self.name, self.kind, self.gear = name, kind, gear
        self.min, self.max = lo, hi
        self.pos = 0.0
        self.start_pos = 0.0
        self.target = 0.0
        self.t0 = 0.0
        self.dur = 0.0
        self.present = True
        self.current = 1200
        self.usteps = 1
        self.vel = 60.0
        self.acc = 300.0

    def update(self):
        if self.dur <= 0:
            return
        frac = (time.time() - self.t0) / self.dur
        if frac >= 1.0:
            self.pos, self.dur = self.target, 0.0
        else:
            self.pos = self.start_pos + (self.target - self.start_pos) * frac

    @property
    def moving(self):
        self.update()
        return self.dur > 0

    def move_to(self, deg, feed, seconds=None):
        self.update()
        self.start_pos = self.pos
        self.target = deg
        dist = abs(deg - self.pos)
        self.t0 = time.time()
        self.dur = seconds if seconds is not None else dist / max(1e-3, self.vel * feed)


class FakeSerial:
    """Just enough of the pyserial surface for ArmLink."""

    def __init__(self, n_present: int | None = None):
        # Every joint is reported; `present` says which are actually built.
        # Default is ALL of them - the wrist servos and gripper are fitted
        # now. Pass a smaller number to simulate an arm mid-build.
        if n_present is None:
            n_present = len(JOINTS)
        self.j = [_Joint(*spec) for spec in JOINTS]
        for i, x in enumerate(self.j):
            x.present = i < n_present
        self.enabled = False
        self.estop = False
        self.grip = dict(GRIP_DEFAULTS)
        self.grip_hold = False      # holding torque on after a full close
        self._out: list[bytes] = []
        self._was_moving = False
        self._emit(f"# arm-fw {FW_VERSION} proto=1 SIMULATED")
        self._emit("# reset: power-on")
        self._emit("ok")

    # -- pyserial surface ---------------------------------------------------
    @property
    def in_waiting(self):
        self._tick()
        return sum(len(b) for b in self._out)

    def reset_input_buffer(self):
        pass

    def flush(self):
        pass

    def close(self):
        pass

    def readline(self) -> bytes:
        self._tick()
        return self._out.pop(0) if self._out else b""

    def write(self, data: bytes) -> int:
        for line in data.decode("utf-8", "replace").splitlines():
            line = line.strip()
            if line:
                self._handle(line)
        return len(data)

    # -- internals ----------------------------------------------------------
    def _emit(self, text: str):
        self._out.append((text + "\n").encode())

    def _tick(self):
        moving = any(x.moving for x in self.j)
        if self._was_moving and not moving:
            self._emit("! idle")
        self._was_moving = moving

    def _pick(self, token):
        t = token.lower()
        if t == "all":
            return list(range(len(self.j)))
        for i, x in enumerate(self.j):
            if x.name.lower() == t:
                return [i]
        if t.isdigit() and int(t) < len(self.j):
            return [int(t)]
        return []

    # -- gripper --------------------------------------------------------------
    @property
    def _g(self) -> _Joint:
        return self.j[-1]

    def _grip_mode(self) -> str:
        g = self._g
        if g.moving:
            return "closing" if g.target > g.start_pos else "opening"
        if self.grip_hold and g.pos >= 99.9:
            return "holding"
        return "open" if g.pos <= 0.1 else "stopped"

    def _grip_drive_to(self, pct: float, power: float | None = None):
        """Same timing model as the firmware: close_ms per full wind at
        close_power, open_ms per full unwind."""
        g = self._g
        g.update()
        pct = min(100.0, max(0.0, pct))
        power = power or self.grip["close_power"]
        if pct >= 99.9 or pct > g.pos:
            secs = (pct - g.pos) / 100.0 * self.grip["close_ms"] / 1000.0 / max(power, 0.05)
            self.grip_hold = pct >= 99.9 and self.grip["hold"] > 0
        else:
            secs = (g.pos - pct) / 100.0 * self.grip["open_ms"] / 1000.0
            self.grip_hold = False
        g.move_to(pct, 1.0, seconds=max(secs, 0.0))

    def _grip_report(self):
        g = self.grip
        self._emit(f"< grip mode={self._grip_mode()} wind={self._g.pos:.1f} "
                   f"speed=0.00 continuous={g['continuous']}")
        self._emit(f"< grip neutral_us={g['neutral_us']} span_us={g['span_us']} "
                   f"close_dir={g['close_dir']} close_power={g['close_power']:.2f} "
                   f"close_ms={g['close_ms']} hold={g['hold']:.2f} "
                   f"open_ms={g['open_ms']} max_run_ms={g['max_run_ms']}")

    def _grip_cmd(self, p: list[str]):
        sub = p[1].lower() if len(p) > 1 else ""
        if not sub:
            self._grip_report()
            return self._emit("ok")
        if sub == "set":
            if len(p) < 4:
                return self._emit("err 1 syntax grip set <key> <value>")
            key = p[2].lower()
            if key not in self.grip:
                return self._emit(f"err 9 bad-value {key}")
            self.grip[key] = type(self.grip[key])(float(p[3]))
            self._grip_report()
            return self._emit("ok")
        if sub == "zero":
            self._g.pos = self._g.target = 0.0
            self._g.dur = 0.0
            self.grip_hold = False
            self._grip_report()
            return self._emit("ok")
        if sub == "stop":
            self._g.update()
            self._g.dur = 0.0
            self.grip_hold = False
            self._grip_report()
            return self._emit("ok")
        if self.estop:
            return self._emit("err 10 estop estop is latched - run: estop clear")
        if not self.enabled:
            return self._emit("err 6 disabled motors are off - run: enable 1")
        if sub == "close":
            self._grip_drive_to(100.0, float(p[2]) if len(p) > 2 else None)
        elif sub == "open":
            self._grip_drive_to(0.0)
        elif sub == "run":
            speed = float(p[2])
            ms = min(float(p[3]) if len(p) > 3 else 300.0, self.grip["max_run_ms"])
            per = self.grip["close_ms"] if speed > 0 else self.grip["open_ms"]
            self._g.update()
            target = self._g.pos + 100.0 * speed * ms / per
            self.grip_hold = False
            self._g.move_to(min(100.0, max(0.0, target)), 1.0, seconds=ms / 1000.0)
        else:
            return self._emit("err 1 syntax grip [close|open|stop|run|zero|set]")
        self._grip_report()
        return self._emit("ok")

    def _handle(self, line: str):
        p = line.split()
        c = p[0].lower()

        if c == "grip":
            return self._grip_cmd(p)

        if c == "info":
            self._emit(f"< fw=arm-fw version={FW_VERSION} proto=1 built=simulated")
            self._emit(f"< joints={len(self.j)} steppers=3 servos={len(self.j)-3}")
            for x in self.j:
                self._emit(f"< joint={x.name} kind={x.kind} "
                           f"present={int(x.present)} attached={int(x.present)} "
                           f"min={x.min:.2f} max={x.max:.2f}")
            return self._emit("ok")

        if c == "state":
            moving = any(x.moving for x in self.j)
            self._emit(f"< enabled={int(self.enabled)} moving={int(moving)} "
                       f"estop={int(self.estop)}")
            for x in self.j:
                extra = " driver=online" if (x.kind == "stepper" and x.present) else ""
                if x is self._g:
                    extra = f" grip={self._grip_mode()}"
                self._emit(f"< {x.name} pos={x.pos:.3f} moving={int(x.moving)} "
                           f"present={int(x.present)} attached={int(x.present)}{extra}")
            return self._emit("ok")

        if c == "config":
            for x in self.j:
                self._emit(f"< {x.name} kind={x.kind} gear={x.gear:.3f} "
                           f"usteps={x.usteps} steps_per_rev=200 invert=0")
                self._emit(f"< {x.name} min={x.min:.2f} max={x.max:.2f} "
                           f"vel={x.vel:.1f} acc={x.acc:.1f}")
                if x.kind == "stepper":
                    spd = 200 * x.usteps * x.gear / 360.0
                    self._emit(f"< {x.name} current={x.current} hold=0.60 "
                               f"chopper=spread intpol=1 steps_per_deg={spd:.4f}")
            return self._emit("ok")

        if c == "enable":
            if self.estop and p[1] != "0":
                return self._emit("err 10 estop clear it first: estop clear")
            self.enabled = p[1] != "0"
            if not self.enabled:
                self.grip_hold = False      # no pulses, no holding torque
            self._emit(f"< enabled={int(self.enabled)}")
            return self._emit("ok")

        if c == "estop":
            if len(p) > 1 and p[1] == "clear":
                self.estop = False
                self._emit("< estop=0")
            else:
                self.estop = True
                self.enabled = False
                self.grip_hold = False
                for x in self.j:
                    x.dur = 0.0
                self._emit("< estop=1")
            return self._emit("ok")

        if c == "stop":
            for x in self.j:
                x.update()
                x.dur = 0.0
            return self._emit("ok")

        if c in ("move", "jog"):
            idx = self._pick(p[1])
            if not idx:
                return self._emit(f"err 3 bad-joint {p[1]}")
            x = self.j[idx[0]]
            if self.estop:
                return self._emit("err 10 estop")
            if not self.enabled:
                return self._emit("err 6 disabled")
            if not x.present:
                return self._emit(f"err 5 not-attached {x.name} is not present")
            deg = float(p[2]) + (x.pos if c == "jog" else 0.0)
            if x is self._g and self.grip["continuous"]:
                self._grip_drive_to(deg)
                self._emit(f"< target {x.name}={min(100.0, max(0.0, deg)):.3f}")
                return self._emit("ok")
            if deg < x.min or deg > x.max:
                return self._emit(f"err 4 out-of-range {x.name} target={deg:.2f} "
                                  f"limits={x.min:.2f}..{x.max:.2f}")
            x.move_to(deg, float(p[3]) if len(p) > 3 else 1.0)
            self._emit(f"< target {x.name}={deg:.3f}")
            return self._emit("ok")

        if c == "goto":
            if self.estop:
                return self._emit("err 10 estop")
            if not self.enabled:
                return self._emit("err 6 disabled")
            vals, feed = [], 1.0
            for tok in p[1:]:
                if tok == "-":
                    vals.append(None)
                else:
                    vals.append(float(tok))
            if len(vals) > len(self.j):
                feed = vals[len(self.j)] or 1.0
                vals = vals[:len(self.j)]
            longest = 0.0
            for i, v in enumerate(vals):
                if v is None:
                    continue
                x = self.j[i]
                if not x.present:
                    return self._emit(f"err 5 not-attached {x.name} is not present")
                if v < x.min or v > x.max:
                    return self._emit(f"err 4 out-of-range {x.name} target={v:.2f} "
                                      f"limits={x.min:.2f}..{x.max:.2f}")
                longest = max(longest, abs(v - x.pos) / max(1e-3, x.vel * feed))
            for i, v in enumerate(vals):
                if v is None:
                    continue
                if self.j[i] is self._g and self.grip["continuous"]:
                    self._grip_drive_to(v)
                else:
                    self.j[i].move_to(v, feed, seconds=longest)
            return self._emit("ok")

        if c == "teach":
            for i in self._pick(p[1]):
                self.j[i].pos = self.j[i].target = float(p[2])
                self.j[i].dur = 0.0
            return self._emit("ok")

        if c == "set":
            idx = self._pick(p[1])
            if not idx:
                return self._emit(f"err 3 bad-joint {p[1]}")
            key, val = p[2].lower(), p[3]
            for i in idx:
                x = self.j[i]
                try:
                    if   key == "current": x.current = int(float(val))
                    elif key == "usteps":  x.usteps = int(float(val))
                    elif key == "vel":     x.vel = float(val)
                    elif key == "acc":     x.acc = float(val)
                    elif key == "gear":    x.gear = float(val)
                    elif key == "min":     x.min = float(val)
                    elif key == "max":     x.max = float(val)
                except ValueError:
                    return self._emit(f"err 9 bad-value {val}")
            return self._emit("ok")

        if c in ("save", "load", "probe", "defaults"):
            if c == "probe":
                for x in self.j[:3]:
                    self._emit(f"< {x.name} driver=online")
            return self._emit("ok")

        if c == "pins":
            for i, x in enumerate(self.j):
                self._emit(f"< {x.name} " + ("step=?? dir=?? en=13 addr=%d" % i
                                             if x.kind == "stepper" else "pwm=??"))
            return self._emit("ok")

        return self._emit(f"err 2 unknown-command {c}")
