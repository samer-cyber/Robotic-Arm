"""High-level arm API.

Everything above this layer works in floats and joint names. This is the class
a ROS 2 node should wrap: publish `state()` as JointState, feed
`goto()` from a trajectory controller, and the firmware never changes.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Iterable, Optional

from .link import ArmLink, Reply


def _f(d: dict, key: str, default: float = 0.0) -> float:
    try:
        return float(d[key])
    except (KeyError, TypeError, ValueError):
        return default


def _b(d: dict, key: str) -> bool:
    return str(d.get(key, "0")).lower() in ("1", "true", "online", "yes")


@dataclass
class JointState:
    name: str
    index: int
    position: float = 0.0
    moving: bool = False
    present: bool = True          # is the hardware built at all?
    attached: bool = False        # did the firmware get hold of it?
    driver_online: Optional[bool] = None     # None for servos
    kind: str = "stepper"
    min_deg: float = -180.0
    max_deg: float = 180.0
    extra: dict = field(default_factory=dict)   # any other key=value, e.g. grip=holding


@dataclass
class ArmState:
    enabled: bool = False
    moving: bool = False
    estopped: bool = False
    joints: list[JointState] = field(default_factory=list)

    @property
    def positions(self) -> list[float]:
        return [j.position for j in self.joints]

    @property
    def usable(self) -> list[JointState]:
        return [j for j in self.joints if j.present and j.attached]

    def by_name(self) -> dict:
        return {j.name: j.position for j in self.usable}


class ArmError(RuntimeError):
    def __init__(self, reply: Reply):
        super().__init__(str(reply))
        self.reply = reply
        self.code = reply.err_code
        self.name = reply.err_name


class Arm:
    def __init__(self, link: ArmLink):
        self.link = link
        self.meta: list[JointState] = []
        self.refresh_info()

    # ------------------------------------------------------------- plumbing --
    def cmd(self, line: str, timeout: float = 8.0, strict: bool = False) -> Reply:
        r = self.link.command(line, timeout)
        if strict and not r.ok:
            raise ArmError(r)
        return r

    def close(self) -> None:
        self.link.close()

    # ------------------------------------------------------------ discovery --
    def refresh_info(self, attempts: int = 3, delay: float = 0.6) -> list[JointState]:
        """Ask what joints exist, retrying an empty answer.

        One empty reply used to leave `meta` empty for the whole session and
        nothing ever asked again - the host simply believed the arm had no
        joints. That is a bad thing to conclude from one timing accident, and
        connecting to a board that is still booting is exactly such an
        accident.
        """
        for attempt in range(max(1, attempts)):
            got = self._read_info_once()
            if got:
                self.meta = got
                return self.meta
            if attempt + 1 < attempts:
                time.sleep(delay)
        return self.meta

    def _read_info_once(self) -> list[JointState]:
        r = self.cmd("info")
        out: list[JointState] = []
        idx = 0
        for row in r.rows:
            if "joint" not in row:
                continue
            out.append(JointState(
                name=row["joint"],
                index=idx,
                kind=row.get("kind", "stepper"),
                present=_b(row, "present") if "present" in row else True,
                attached=_b(row, "attached"),
                min_deg=_f(row, "min", -180.0),
                max_deg=_f(row, "max", 180.0),
            ))
            idx += 1
        return out

    def joint_index(self, who) -> int:
        if isinstance(who, int):
            return who
        for j in self.meta:
            if j.name.lower() == str(who).lower():
                return j.index
        raise ValueError(f"no such joint: {who}")

    # ---------------------------------------------------------------- state --
    def state(self, attempts: int = 3) -> ArmState:
        """Read joint state, retrying a reply that came back short.

        The link drops bursts, and a short reply is indistinguishable from
        a real answer about fewer joints. `info` tells us how many to
        expect, so anything less is a lost row, not news.
        """
        expected = len(self.meta)
        st = self._read_state_once()
        for _ in range(max(0, attempts - 1)):
            if not expected or len(st.joints) >= expected:
                break
            time.sleep(0.05)
            st = self._read_state_once()
        return st

    def _read_state_once(self) -> ArmState:
        r = self.cmd("state")
        st = ArmState()
        idx = 0
        for row in r.rows:
            names = row.get("_", [])
            if "enabled" in row:
                st.enabled = _b(row, "enabled")
                st.moving = _b(row, "moving")
                st.estopped = _b(row, "estop")
                continue
            if not names:
                continue
            meta = self.meta[idx] if idx < len(self.meta) else None
            st.joints.append(JointState(
                name=names[0],
                index=idx,
                position=_f(row, "pos"),
                moving=_b(row, "moving"),
                present=_b(row, "present") if "present" in row else True,
                attached=_b(row, "attached"),
                driver_online=(_b(row, "driver") if "driver" in row else None),
                kind=meta.kind if meta else "stepper",
                min_deg=meta.min_deg if meta else -180.0,
                max_deg=meta.max_deg if meta else 180.0,
                extra={k: v for k, v in row.items()
                       if k not in ("_", "pos", "moving", "present", "attached", "driver")},
            ))
            idx += 1
        return st

    def config(self, joint=None) -> list[dict]:
        target = "" if joint is None else f" {self.joint_index(joint)}"
        return self.cmd(f"config{target}").rows

    # --------------------------------------------------------------- motion --
    def enable(self, on: bool = True) -> Reply:
        return self.cmd(f"enable {1 if on else 0}")

    def move(self, joint, deg: float, feed: Optional[float] = None) -> Reply:
        f = "" if feed is None else f" {feed:.3f}"
        return self.cmd(f"move {self.joint_index(joint)} {deg:.4f}{f}")

    def jog(self, joint, deg: float, feed: Optional[float] = None) -> Reply:
        f = "" if feed is None else f" {feed:.3f}"
        return self.cmd(f"jog {self.joint_index(joint)} {deg:.4f}{f}")

    def goto(self, targets: Iterable[Optional[float]], feed: Optional[float] = None) -> Reply:
        """`None` for a joint leaves it where it is."""
        parts = ["-" if t is None else f"{float(t):.4f}" for t in targets]
        f = "" if feed is None else f" {feed:.3f}"
        return self.cmd("goto " + " ".join(parts) + f)

    def goto_named(self, mapping: dict, feed: Optional[float] = None) -> Reply:
        """Move using a {joint_name: degrees} map.

        Positional lists break as soon as the joint set changes - a pose saved
        when six joints were reported would later be replayed against three,
        or worse, sent to a joint that is not built. Names do not have that
        problem, and unknown or unbuilt joints are simply skipped.
        """
        # Which joints exist comes from `info` (cached in meta), NOT from a
        # fresh `state` read. A dropped newline on the serial link merges two
        # state rows into one, and since later keys overwrite earlier ones a
        # row named J4 can inherit `present=1 attached=1` from J3's half. That
        # is how an unbuilt servo ended up being commanded and the whole move
        # was refused with not-attached. One corrupt read must not be able to
        # invent a joint.
        usable = {j.name for j in self.meta if j.present and j.attached}
        if not usable:                      # meta never loaded - fall back
            usable = {j.name for j in self.state().usable}
        targets = []
        for j in self.meta:
            v = mapping.get(j.name)
            targets.append(v if (v is not None and j.name in usable) else None)
        if all(t is None for t in targets):
            return self.cmd("state")     # nothing to do, but stay well-formed
        return self.goto(targets, feed)

    def teach(self, joint, deg: float) -> Reply:
        who = "all" if joint == "all" else str(self.joint_index(joint))
        return self.cmd(f"teach {who} {deg:.4f}")

    def stop(self) -> Reply:
        return self.cmd("stop")

    def estop(self) -> Reply:
        return self.cmd("estop")

    def clear_estop(self) -> Reply:
        return self.cmd("estop clear")

    def wait_idle(self, timeout: float = 60.0, poll: float = 0.1) -> bool:
        """Poll until motion stops. Returns False on timeout.

        The firmware also emits `! idle`, which is the better signal for an
        event-driven host; this exists for scripts that just want to block.
        """
        deadline = time.time() + timeout
        while time.time() < deadline:
            if not self.state().moving:
                return True
            time.sleep(poll)
        return False

    # --------------------------------------------------------------- config --
    def set(self, joint, key: str, value) -> Reply:
        who = "all" if joint == "all" else str(self.joint_index(joint))
        return self.cmd(f"set {who} {key} {value}")

    def save(self) -> Reply:
        return self.cmd("save")

    def probe(self) -> Reply:
        return self.cmd("probe")

    # -------------------------------------------------------------- gripper --
    # The hand is a continuous-rotation servo on a tendon spool, so it takes
    # verbs, not angles. Closing winds until the fingers stall and then holds;
    # opening unwinds for as long as it was wound and stops (firmware >= 1.3.0).
    def grip_close(self, power: Optional[float] = None) -> Reply:
        return self.cmd("grip close" + ("" if power is None else f" {power:.2f}"))

    def grip_open(self) -> Reply:
        return self.cmd("grip open")

    def grip_stop(self) -> Reply:
        return self.cmd("grip stop")

    def grip_run(self, speed: float, ms: int = 300) -> Reply:
        """Raw drive for calibration. +speed closes. Capped by max_run_ms."""
        return self.cmd(f"grip run {speed:.2f} {int(ms)}")

    def grip_set(self, key: str, value) -> Reply:
        return self.cmd(f"grip set {key} {value}")

    def grip_state(self) -> dict:
        """{'mode': 'holding', 'wind': 100.0, 'neutral_us': 1500, ...}"""
        out: dict = {}
        for row in self.cmd("grip").rows:
            out.update({k: v for k, v in row.items() if k != "_"})
        for k, v in list(out.items()):
            try:
                out[k] = float(v)
            except (TypeError, ValueError):
                pass
        return out
