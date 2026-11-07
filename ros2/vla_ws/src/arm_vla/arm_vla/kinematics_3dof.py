"""Closed-form inverse kinematics for a 3-DOF articulated arm.

Pure Python, no ROS and no numpy. Import it from a node, or run it directly to
self-test the solver:

    python3 kinematics_3dof.py                 # round-trip test
    python3 kinematics_3dof.py 200 50 120      # solve one point

Frame and angle conventions
---------------------------
Base frame origin sits on the table, on the J1 rotation axis. +Z is up.

    theta_base      0 deg  = +X axis, positive counter-clockwise seen from above
    theta_shoulder  0 deg  = upper arm horizontal, positive lifts the arm up
    theta_elbow     0 deg  = forearm in line with the upper arm (straight),
                             measured RELATIVE to the upper arm,
                             negative folds the elbow upward

These match the conventions already used in the ESP32 test firmware, so a pose
means the same thing on both sides of the wire.

Errors are returned as data, never raised -- the same choice `armlink` makes.
An IKSolution is falsy when it failed, so `if not sol:` is the idiom.
"""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass
from typing import Optional, Tuple

__all__ = [
    "ArmGeometry", "JointLimits", "SafetyEnvelope", "IKSolution",
    "forward_kinematics", "solve_ik", "solve_both", "self_test",
]


# ---------------------------------------------------------------------------
#  Configuration
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class ArmGeometry:
    """All lengths in millimetres. Measured hardware, 2026-09-02.

    Runtime values come from config/vla_params.yaml; these defaults exist so
    the standalone self-test reports the real arm.

    Note l2 << l1. Reach is a SHELL, 115..245 mm from the shoulder axis: there
    is a dead zone around the shoulder as well as a limit at the outside.
    """
    l0: float = 107.65          # desk to the J2 (shoulder) rotation axis
    l1: float = 180.0           # shoulder axis to elbow axis
    l2: float = 65.0            # elbow axis to the flat tip
    base_offset: float = 0.0    # radial offset from the J1 axis to the J2 axis

    @property
    def reach_max(self) -> float:
        return self.l1 + self.l2

    @property
    def reach_min(self) -> float:
        return abs(self.l1 - self.l2)


@dataclass(frozen=True)
class JointLimits:
    """Soft limits in degrees, (min, max) per joint."""
    base: Tuple[float, float] = (-180.0, 180.0)   # full rotation
    shoulder: Tuple[float, float] = (-20.0, 135.0)
    elbow: Tuple[float, float] = (-150.0, 0.0)


@dataclass(frozen=True)
class SafetyEnvelope:
    """Limits on the *target point*, checked before the arm ever moves.

    min_radius is not a nicety. As the target approaches the J1 axis,
    atan2(y, x) becomes ill-conditioned: a target that wobbles by a couple of
    millimetres -- which is exactly what stereo depth does -- swings the base
    joint through tens of degrees, or flips it 180 degrees between frames. The
    arm slams around for no visible reason. Refuse the region instead.
    """
    z_floor: float = 10.0        # never command a tip below this, mm
    z_ceiling: float = 600.0
    min_radius: float = 60.0     # keep clear of the base singularity, mm
    max_radius: Optional[float] = None   # None = use geometry reach


@dataclass
class IKSolution:
    ok: bool
    status: str
    detail: str = ""
    base: float = 0.0
    shoulder: float = 0.0
    elbow: float = 0.0
    elbow_up: bool = True

    def __bool__(self) -> bool:
        return self.ok

    def as_tuple(self) -> Tuple[float, float, float]:
        return (self.base, self.shoulder, self.elbow)

    def as_named(self) -> dict:
        """Ready to hand to armlink's Arm.goto_named()."""
        return {"J1": self.base, "J2": self.shoulder, "J3": self.elbow}

    def __str__(self) -> str:
        if not self.ok:
            return f"IK FAILED [{self.status}] {self.detail}"
        return (f"IK ok  base={self.base:+8.3f}  shoulder={self.shoulder:+8.3f}  "
                f"elbow={self.elbow:+8.3f}  ({'up' if self.elbow_up else 'down'})")


# ---------------------------------------------------------------------------
#  Forward kinematics
# ---------------------------------------------------------------------------

def forward_kinematics(base_deg: float, shoulder_deg: float, elbow_deg: float,
                       geom: ArmGeometry) -> Tuple[float, float, float]:
    """Joint angles (deg) -> tip position (mm) in the arm base frame."""
    t1 = math.radians(base_deg)
    t2 = math.radians(shoulder_deg)
    t3 = math.radians(elbow_deg)

    r = geom.base_offset + geom.l1 * math.cos(t2) + geom.l2 * math.cos(t2 + t3)
    z = geom.l0 + geom.l1 * math.sin(t2) + geom.l2 * math.sin(t2 + t3)
    return (r * math.cos(t1), r * math.sin(t1), z)


# ---------------------------------------------------------------------------
#  Inverse kinematics
# ---------------------------------------------------------------------------

def _check_limits(sol: IKSolution, limits: JointLimits) -> IKSolution:
    for value, (lo, hi), name in (
        (sol.base,     limits.base,     "base"),
        (sol.shoulder, limits.shoulder, "shoulder"),
        (sol.elbow,    limits.elbow,    "elbow"),
    ):
        if value < lo - 1e-6 or value > hi + 1e-6:
            return IKSolution(
                ok=False, status=f"limit_{name}",
                detail=f"{name}={value:.2f} deg outside [{lo:.1f}, {hi:.1f}]",
                base=sol.base, shoulder=sol.shoulder, elbow=sol.elbow,
                elbow_up=sol.elbow_up)
    return sol


def solve_ik(x: float, y: float, z: float,
             geom: ArmGeometry,
             limits: Optional[JointLimits] = None,
             envelope: Optional[SafetyEnvelope] = None,
             elbow_up: bool = True,
             check_limits: bool = True) -> IKSolution:
    """Target point (mm, arm base frame) -> joint angles (deg).

    Set check_limits=False to see what the solver *would* have produced. That
    is worth printing when a move is rejected: "shoulder wanted 141 deg, limit
    is 135" tells you something. "unreachable" does not.
    """
    if limits is None:
        limits = JointLimits()

    # --- envelope checks on the raw target, before any trigonometry ---------
    if envelope is not None:
        if z < envelope.z_floor:
            return IKSolution(False, "below_floor",
                              f"z={z:.1f} mm is under the {envelope.z_floor:.1f} mm floor")
        if z > envelope.z_ceiling:
            return IKSolution(False, "above_ceiling",
                              f"z={z:.1f} mm is over the {envelope.z_ceiling:.1f} mm ceiling")
        r_xy = math.hypot(x, y)
        if r_xy < envelope.min_radius:
            return IKSolution(False, "too_close_to_base",
                              f"target is {r_xy:.1f} mm from the base axis, "
                              f"inside the {envelope.min_radius:.1f} mm singularity guard")
        r_cap = envelope.max_radius
        if r_cap is not None and r_xy > r_cap:
            return IKSolution(False, "outside_workspace",
                              f"target is {r_xy:.1f} mm out, cap is {r_cap:.1f} mm")

    # --- reduce to the planar 2-link problem -------------------------------
    t1 = math.atan2(y, x)
    r = math.hypot(x, y) - geom.base_offset
    dz = z - geom.l0
    dist = math.hypot(r, dz)

    if dist > geom.reach_max:
        return IKSolution(False, "unreachable_far",
                          f"{dist:.1f} mm from the shoulder, arm reaches {geom.reach_max:.1f} mm "
                          f"(short by {dist - geom.reach_max:.1f} mm)")
    if dist < geom.reach_min:
        return IKSolution(False, "unreachable_near",
                          f"{dist:.1f} mm from the shoulder, inner limit is {geom.reach_min:.1f} mm")

    cos_t3 = (r * r + dz * dz - geom.l1 ** 2 - geom.l2 ** 2) / (2.0 * geom.l1 * geom.l2)
    cos_t3 = max(-1.0, min(1.0, cos_t3))        # kill float dust at the edges

    t3 = math.acos(cos_t3)
    if elbow_up:
        t3 = -t3
    t2 = math.atan2(dz, r) - math.atan2(geom.l2 * math.sin(t3),
                                        geom.l1 + geom.l2 * math.cos(t3))

    sol = IKSolution(True, "ok",
                     base=math.degrees(t1),
                     shoulder=math.degrees(t2),
                     elbow=math.degrees(t3),
                     elbow_up=elbow_up)
    return _check_limits(sol, limits) if check_limits else sol


def solve_both(x: float, y: float, z: float,
               geom: ArmGeometry,
               limits: Optional[JointLimits] = None,
               envelope: Optional[SafetyEnvelope] = None,
               prefer_elbow_up: bool = True) -> IKSolution:
    """Try both elbow branches, return the first that respects the limits.

    A point can easily be reachable elbow-down and blocked elbow-up. Picking
    one branch and giving up is how a perfectly reachable target gets reported
    as unreachable.
    """
    order = (prefer_elbow_up, not prefer_elbow_up)
    first: Optional[IKSolution] = None
    for up in order:
        sol = solve_ik(x, y, z, geom, limits, envelope, elbow_up=up, check_limits=True)
        if sol.ok:
            return sol
        if first is None:
            first = sol
        # An envelope or reach rejection applies to both branches; no point
        # solving the second one.
        if sol.status not in ("limit_base", "limit_shoulder", "limit_elbow"):
            return sol
    return first if first is not None else IKSolution(False, "no_solution")


# ---------------------------------------------------------------------------
#  Self-test
# ---------------------------------------------------------------------------

def self_test(n: int = 5000, geom: Optional[ArmGeometry] = None,
              limits: Optional[JointLimits] = None, verbose: bool = True) -> bool:
    """Round-trip random in-limit poses: FK -> IK -> FK, report the error.

    This validates the solver against the FK model only. It cannot tell you
    that your link lengths match the physical arm -- nothing in software can.
    """
    import random
    geom = geom or ArmGeometry()
    limits = limits or JointLimits()
    rng = random.Random(0xA3D0F)

    worst_cart = 0.0
    sum_cart = 0.0
    worst_joint = 0.0
    fails = 0
    tested = 0
    mirrored = 0

    for _ in range(n):
        b = rng.uniform(*limits.base)
        s = rng.uniform(*limits.shoulder)
        e = rng.uniform(*limits.elbow)
        x, y, z = forward_kinematics(b, s, e, geom)

        # Skip poses the envelope would refuse anyway; they are not solver bugs.
        if math.hypot(x, y) < 1e-3:
            continue
        tested += 1

        sol = solve_ik(x, y, z, geom, limits, envelope=None,
                       elbow_up=(e <= 0.0), check_limits=False)
        if not sol:
            fails += 1
            continue

        x2, y2, z2 = forward_kinematics(sol.base, sol.shoulder, sol.elbow, geom)
        d = math.dist((x, y, z), (x2, y2, z2))
        sum_cart += d
        worst_cart = max(worst_cart, d)

        # A pose can fold the tip through to the far side of the base axis, so
        # the planar radius goes negative. IK then legitimately answers with
        # the mirror image: base rotated 180 deg, shoulder/elbow reflected. The
        # tip lands in exactly the same place -- which is why the cartesian
        # error stays at zero -- but the joint angles are not the ones we
        # started from, so comparing them directly is meaningless. Count these
        # separately instead of letting them poison the joint metric.
        if abs(abs(b - sol.base) - 180.0) < 1.0:
            mirrored += 1
            continue
        worst_joint = max(worst_joint,
                          abs(b - sol.base), abs(s - sol.shoulder), abs(e - sol.elbow))

    good = fails == 0 and worst_cart < 1e-6 and worst_joint < 1e-6
    if verbose:
        print(f"  geometry      l0={geom.l0} l1={geom.l1} l2={geom.l2} "
              f"offset={geom.base_offset}")
        print(f"  samples       {tested}")
        print(f"  IK failures   {fails}")
        print(f"  cartesian     mean {sum_cart / max(tested, 1):.3e} mm   "
              f"worst {worst_cart:.3e} mm")
        print(f"  joint         worst {worst_joint:.3e} deg  "
              f"({mirrored} mirrored poses excluded)")
        print(f"  verdict       {'PASS' if good else 'FAIL - solver disagrees with FK'}")
        if mirrored:
            print(f"  note          {100.0 * mirrored / max(tested, 1):.0f}% of the joint-limit "
                  f"box puts the tip behind the base axis.")
            print( "                Those poses come back as base+/-180, which your base limit")
            print( "                rejects. That is correct: it is the arm reaching back over")
            print( "                itself. It does mean the usable workspace is smaller than")
            print( "                the joint limits suggest -- check it with reach_report().")
    return good


def reach_report(geom: Optional[ArmGeometry] = None,
                 limits: Optional[JointLimits] = None,
                 envelope: Optional[SafetyEnvelope] = None,
                 n: int = 20000) -> dict:
    """How much of the bounding box is actually commandable, and why not.

    Run this before blaming the vision system for targets that get refused.
    """
    import random
    geom = geom or ArmGeometry()
    limits = limits or JointLimits()
    envelope = envelope or SafetyEnvelope()
    rng = random.Random(0x11FE)

    R = geom.reach_max + geom.base_offset
    tally: dict = {}
    for _ in range(n):
        x = rng.uniform(-R, R)
        y = rng.uniform(-R, R)
        z = rng.uniform(0.0, geom.l0 + R)
        sol = solve_both(x, y, z, geom, limits, envelope)
        key = "ok" if sol.ok else sol.status
        tally[key] = tally.get(key, 0) + 1
    return {k: (v, 100.0 * v / n) for k, v in sorted(tally.items(),
                                                     key=lambda kv: -kv[1])}


def _main(argv) -> int:
    geom = ArmGeometry()
    limits = JointLimits()
    envelope = SafetyEnvelope()

    if len(argv) >= 4:
        x, y, z = (float(v) for v in argv[1:4])
        print(f"target        x={x:.1f} y={y:.1f} z={z:.1f} mm")
        print(f"reach         {geom.reach_min:.1f} .. {geom.reach_max:.1f} mm from the shoulder")
        for up in (True, False):
            sol = solve_ik(x, y, z, geom, limits, envelope, elbow_up=up,
                           check_limits=True)
            print(f"  elbow {'up  ' if up else 'down'}  {sol}")
            if sol:
                fk = forward_kinematics(sol.base, sol.shoulder, sol.elbow, geom)
                print(f"              FK check -> {fk[0]:.3f} {fk[1]:.3f} {fk[2]:.3f} mm")
        best = solve_both(x, y, z, geom, limits, envelope)
        print(f"  chosen      {best}")
        return 0 if best else 1

    print("3-DOF IK self-test")
    print("  geometry: measured hardware 2026-09-02, base rotates fully.")
    ok = self_test()
    print()
    print("workspace census (random points in the bounding box):")
    for status, (count, pct) in reach_report(geom, limits, envelope).items():
        print(f"  {status:<20} {count:6d}  {pct:5.1f}%")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(_main(sys.argv))
