"""Cycloidal joint - the maths, with no Fusion in it.

Everything the generator builds is decided here first, so it can be checked
with plain Python (python cyclo_core.py) before Fusion draws a single sketch.

TOPOLOGY (v7, "inverted" - the one the arm actually uses)
---------------------------------------------------------
The pin carrier is bolted to the MOTOR and does not turn. The roller ring is a
DRUM that becomes part of the next link, and it is the drum that turns. With
the discs held against rotation by the pins, one eccentric turn walks the
ring by one roller pitch, so:

    ratio = number of rollers N   (lobes = N - 1)
    the drum turns the SAME way as the motor

(v6 built the classic layout - ring fixed, pins as output - where the ratio
is the lobe count and the output turns backwards. Same parts, different
ratio: that is why the arm's "15:1" boxes are really 16:1.)

Axial stack, z = 0 on the face away from the motor, motor on +z:

    two bearings:  [far carrier][far bearing][disc A][disc B][near bearing][near carrier][motor]
    one bearing :  [drum floor ][screw heads][disc A][disc B][near bearing][near carrier][motor]

The carriers and the output screws form one rigid cage bolted to the motor.
The drum spins on the big bearing(s) around it.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

# ----------------------------------------------------------------- presets --
#  Motors: shaft lengths are measured from the mounting face.
MOTORS = {
    "NEMA 17 (5 mm D-shaft)":   dict(shaft_d=5.0, flat=4.5, boss_d=22.0, boss_h=2.0, round_len=3.0,
                                     dcut_len=18.0, bolt_spacing=31.0, bolt_clear=3.4, cbore_d=6.5,
                                     head_h=3.0, body=42.3, length=40.0),
    "NEMA 14 (5 mm D-shaft)":   dict(shaft_d=5.0, flat=4.5, boss_d=22.0, boss_h=2.0, round_len=2.0,
                                     dcut_len=15.0, bolt_spacing=26.0, bolt_clear=3.4, cbore_d=6.5,
                                     head_h=3.0, body=35.2, length=34.0),
    "NEMA 23 (6.35 mm D-shaft)": dict(shaft_d=6.35, flat=5.8, boss_d=38.1, boss_h=1.6, round_len=3.0,
                                      dcut_len=15.0, bolt_spacing=47.14, bolt_clear=5.3, cbore_d=9.5,
                                      head_h=5.0, body=56.4, length=56.0),
    "Custom D-shaft":           dict(shaft_d=5.0, flat=4.5, boss_d=22.0, boss_h=2.0, round_len=3.0,
                                     dcut_len=18.0, bolt_spacing=31.0, bolt_clear=3.4, cbore_d=6.5,
                                     head_h=3.0, body=42.3, length=40.0),
}

#  Deep-groove bearings: (bore, OD, width)
BIG_BEARINGS = {
    "6808  40x52x7":  (40.0, 52.0, 7.0),
    "6806  30x42x7":  (30.0, 42.0, 7.0),
    "6807  35x47x7":  (35.0, 47.0, 7.0),
    "6810  50x65x7":  (50.0, 65.0, 7.0),
    "6812  60x78x10": (60.0, 78.0, 10.0),
    "6908  40x62x12": (40.0, 62.0, 12.0),
}
CAM_BEARINGS = {
    "608  8x22x7":  (8.0, 22.0, 7.0),
    "688  8x16x5":  (8.0, 16.0, 5.0),
    "625  5x16x5":  (5.0, 16.0, 5.0),
    "607  7x19x6":  (7.0, 19.0, 6.0),
    "6000 10x26x8": (10.0, 26.0, 8.0),
}
#  Socket-head cap screws used as the output pins.
SCREWS = {
    "M4 cap screw": dict(d=4.0, head_d=7.0, head_h=4.0, tap=3.4, clear=4.4, lengths=(20, 25, 30, 35, 40, 45, 50)),
    "M3 cap screw": dict(d=3.0, head_d=5.5, head_h=3.0, tap=2.6, clear=3.4, lengths=(16, 20, 25, 30, 35, 40)),
}


@dataclass
class Spec:
    """What the user chooses. Every length in mm."""
    rollers: int = 16                       # = the ratio
    two_bearings: bool = True
    motor: str = "NEMA 17 (5 mm D-shaft)"
    custom_shaft_d: float = 5.0
    custom_flat: float = 4.5
    big_bearing: str = "6808  40x52x7"
    cam_bearing: str = "608  8x22x7"
    screw: str = "M4 cap screw"
    screw_count: int = 6
    wall: float = 3.0                       # drum wall outside the roller circle
    clearance: float = 0.05                 # cycloid profile clearance (baked into the spline)
    auto_geometry: bool = True
    ring_radius: float = 26.0               # used when auto_geometry is False
    roller_radius: float = 3.06
    eccentricity: float = 1.02
    disc_gap: float = 0.3                   # disc A <-> disc B
    side_gap: float = 0.5                   # discs <-> carriers / bearings
    near_flange: float = 0.0                # motor-side plate; 0 = auto (screw head + 1.5)
    far_flange: float = 2.5                 # far carrier plate under the screw heads
    floor: float = 3.0                      # drum floor, one-bearing version
    pts_per_lobe: int = 16
    # link arm
    add_link: bool = False
    link_length: float = 130.0              # centre to centre
    link_skin: float = 2.0
    link_hollow: bool = True
    reference_hardware: bool = True


@dataclass
class Plan:
    spec: Spec
    N: int = 0
    Z: int = 0
    R: float = 0.0          # roller circle radius
    rp: float = 0.0         # roller radius
    E: float = 0.0          # eccentricity
    lam: float = 0.0
    rmin: float = 0.0
    drum_od: float = 0.0
    flange_od: float = 0.0
    near_flange: float = 0.0
    screw_r: float = 0.0
    hole_d: float = 0.0
    web: float = 0.0
    motor: dict = field(default_factory=dict)
    big: tuple = ()
    cam: tuple = ()
    scr: dict = field(default_factory=dict)
    z: dict = field(default_factory=dict)       # named axial positions
    screw_len: float = 0.0
    screw_buy: int = 0
    engage: float = 0.0
    errors: list = field(default_factory=list)
    warnings: list = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return not self.errors


def plan(spec: Spec) -> Plan:
    p = Plan(spec=spec)
    err, warn = p.errors, p.warnings
    p.N = N = int(spec.rollers)
    p.Z = N - 1
    p.motor = dict(MOTORS[spec.motor])
    if spec.motor.startswith("Custom"):
        p.motor["shaft_d"], p.motor["flat"] = spec.custom_shaft_d, spec.custom_flat
    p.big = BIG_BEARINGS[spec.big_bearing]
    p.cam = CAM_BEARINGS[spec.cam_bearing]
    p.scr = SCREWS[spec.screw]
    d_b, D_b, w_b = p.big
    d_c, D_c, w_c = p.cam

    if N < 8:
        err.append("Use at least 8 rollers.")
    if spec.screw_count % 2 or spec.screw_count < 4:
        err.append("Output screw count must be even and at least 4 (two discs, 180 deg apart).")

    # --- tooth geometry -------------------------------------------------------
    if spec.auto_geometry:
        # The ring sits on the big-bearing seat radius, so the rollers double as
        # the shoulder the bearing's outer race stops against.
        p.R = D_b / 2.0
        p.rp = round(0.30 * 2 * math.pi * p.R / N, 2)      # ~30 % of the pitch
        p.E = round(0.63 * p.R / N, 2)                      # lambda ~ 0.63
    else:
        p.R, p.rp, p.E = spec.ring_radius, spec.roller_radius, spec.eccentricity
    R, rp, E = p.R, p.rp, p.E
    p.lam = E * N / R
    p.rmin = R - (rp + spec.clearance) - E
    p.drum_od = 2.0 * (R + spec.wall)

    if p.lam >= 1.0:
        err.append(f"lambda = E*N/R = {p.lam:.3f} must be below 1 (the profile loops). Lower E.")
    elif p.lam > 0.8:
        warn.append(f"lambda = {p.lam:.2f}: sharp lobes, high contact stress. 0.5-0.7 prints best.")
    if rp <= 2.0 * E:
        err.append(f"Roller radius {rp:.2f} must exceed 2E = {2 * E:.2f}, or the lobes undercut.")
    if 2 * rp >= 2 * math.pi * R / N * 0.6:
        warn.append("Rollers are crowding each other; fewer rollers or a bigger ring.")
    if D_c / 2.0 >= p.rmin - 2.0:
        err.append(f"Cam bearing OD {D_c:g} does not fit inside the disc (root radius {p.rmin:.1f}).")
    if rp - spec.wall > 0.3:
        warn.append(f"Rollers stand {rp - spec.wall:.2f} mm past the wall and are trimmed flush. "
                    f"Wall >= {rp + 1:.1f} gives them real backing.")
    if p.drum_od / 2.0 - D_b / 2.0 < 1.6:
        err.append("Drum wall around the bearing seat is under 1.6 mm. Raise the wall.")

    # --- output screws / disc holes -------------------------------------------
    p.screw_r = round((D_c / 2.0 + p.rmin) / 2.0, 2)
    p.hole_d = p.scr["d"] + 2.0 * E
    p.web = (p.rmin - D_c / 2.0) / 2.0 - p.hole_d / 2.0
    if p.web < 1.2:
        err.append(f"Only {p.web:.2f} mm of web beside the output holes. Smaller screws or cam bearing.")
    elif p.web < 1.8:
        warn.append(f"Thin web beside the output holes ({p.web:.2f} mm).")
    if p.screw_r + p.scr["tap"] / 2.0 > d_b / 2.0 - 1.0:
        err.append("Output screws land outside the carrier journal. Use a bigger big bearing.")
    chord = 2 * p.screw_r * math.sin(math.pi / spec.screw_count)
    if chord - p.hole_d < 1.5:
        err.append(f"{spec.screw_count} screws are too close together on a {p.screw_r:.1f} mm circle.")

    # --- motor flange ---------------------------------------------------------
    m = p.motor
    bolt_r = m["bolt_spacing"] / math.sqrt(2.0)
    p.flange_od = max(p.drum_od, 2.0 * (bolt_r + m["cbore_d"] / 2.0 + 1.5))
    p.near_flange = spec.near_flange or (m["head_h"] + 1.5)
    if p.near_flange < m["head_h"] + 1.5:
        err.append(f"Motor-side plate {p.near_flange:g} mm cannot hide the motor screw heads "
                   f"({m['head_h']:g} mm) with 1.5 mm left. Raise it.")
    if m["flat"] >= m["shaft_d"]:
        err.append("D-flat must be smaller than the shaft diameter.")

    # --- axial stack ----------------------------------------------------------
    z = {}
    g, gd = spec.side_gap, spec.disc_gap
    if spec.two_bearings:
        z["far_plate0"] = -spec.far_flange
        z["far_bearing0"], z["far_bearing1"] = 0.0, w_b
        z["discA0"] = w_b + g
        z["drum0"] = 0.0
    else:
        z["floor0"], z["floor1"] = 0.0, spec.floor
        z["heads0"] = spec.floor + g                              # screw heads float here
        z["discA0"] = z["heads0"] + p.scr["head_h"] + g
        z["drum0"] = 0.0
    z["discA1"] = z["discA0"] + w_c
    z["discB0"] = z["discA1"] + gd
    z["discB1"] = z["discB0"] + w_c
    z["near_bearing0"] = z["discB1"] + g
    z["near_bearing1"] = z["near_bearing0"] + w_b
    z["drum1"] = z["near_bearing1"]
    z["lip1"] = z["near_bearing1"] + 0.6                          # clears the drum end face
    z["motor_face"] = z["lip1"] + p.near_flange
    p.z = z

    # D-cut engagement: the eccentric spans disc A..disc B (see assembly note)
    dcut_hi = z["motor_face"] - m["boss_h"] - m["round_len"]
    dcut_lo = dcut_hi - m["dcut_len"]
    p.engage = max(0.0, min(z["discB1"], dcut_hi) - max(z["discA0"], dcut_lo))
    if p.engage < 6.0:
        err.append(f"Only {p.engage:.1f} mm of eccentric sits on the D-cut. The shaft is too short "
                   f"for this stack (thinner plate, or a longer-shaft motor).")
    elif p.engage < 10.0:
        warn.append(f"{p.engage:.1f} mm of eccentric on the D-cut (10+ is comfortable).")

    # Screw: head on the far plate (two bearings) or floating above the floor
    # (one bearing); threads into the motor-side carrier for >= 2 x d.
    head_seat = z["far_plate0"] if spec.two_bearings else z["heads0"] + p.scr["head_h"]
    need = (z["near_bearing0"] + 2.0 * p.scr["d"]) - head_seat
    room = z["motor_face"] - 0.5 - head_seat
    p.screw_len = need
    fits = [L for L in p.scr["lengths"] if need <= L <= room]
    if fits:
        p.screw_buy = fits[0]
    else:
        p.screw_buy = 0
        err.append(f"No stock {spec.screw.split()[0]} length between {need:.1f} and {room:.1f} mm.")

    if spec.add_link and spec.link_length < p.drum_od + 2 * spec.link_skin + 4:
        err.append("Link is shorter than two drums side by side.")
    return p


def disc_profile(p: Plan, phase: float = 0.0) -> list:
    """Closed cycloid outline (mm, gearbox frame) of a disc at eccentric phase
    `phase`: 0 for disc A (centre at +E), pi for disc B (centre at -E).

    Disc B is NOT disc A turned 180 deg. Relative to the ring, the disc at
    phase pi is also turned back by pi/Z (half a lobe). A plain 180 deg turn
    only lands on that when the roller count is even - with an odd count
    (21 rollers) it cuts straight into the ring.
    """
    N, R, E, Z = p.N, p.R, p.E, p.Z
    Rr = p.rp + p.spec.clearance
    n = p.spec.pts_per_lobe * Z
    a = -phase / Z
    ca, sa = math.cos(a), math.sin(a)
    cx, cy = E * math.cos(phase), E * math.sin(phase)
    pts = []
    for k in range(n + 1):
        th = 2.0 * math.pi * k / n
        psi = math.atan2(math.sin((1 - N) * th), (R / (E * N)) - math.cos((1 - N) * th))
        x = R * math.cos(th) - Rr * math.cos(th + psi) - E * math.cos(N * th)   # relative to centre
        y = -R * math.sin(th) + Rr * math.sin(th + psi) + E * math.sin(N * th)
        pts.append((cx + ca * x - sa * y, cy + sa * x + ca * y))
    return pts


def mesh_check(p: Plan) -> tuple:
    """Smallest gap (mm) between any roller and disc A / disc B in the
    assembled position. Should equal the profile clearance for both discs;
    a negative number means a disc cuts into the ring."""
    def inside(poly, x, y):
        c = False
        for (x1, y1), (x2, y2) in zip(poly, poly[1:] + poly[:1]):
            if (y1 > y) != (y2 > y) and x < x1 + (y - y1) * (x2 - x1) / (y2 - y1):
                c = not c
        return c

    out = []
    for phase in (0.0, math.pi):
        prof = disc_profile(p, phase)
        worst = float("inf")
        for k in range(p.N):
            rx, ry = p.R * math.cos(2 * math.pi * k / p.N), p.R * math.sin(2 * math.pi * k / p.N)
            d = min(math.hypot(x - rx, y - ry) for x, y in prof) - p.rp
            worst = min(worst, -d if inside(prof, rx, ry) else d)
        out.append(worst)
    return tuple(out)


def summary(p: Plan) -> str:
    s, z = p.spec, p.z
    lines = [
        f"{p.N}:1   ({p.N} rollers, {p.Z} lobes)   drum turns WITH the motor",
        f"{'two' if s.two_bearings else 'one'} big bearing{'s' if s.two_bearings else ''} "
        f"{s.big_bearing.split()[0]},  cam bearings {s.cam_bearing.split()[0]},  {s.motor.split(' (')[0]}",
        "",
        f"ring R {p.R:.2f}   roller r {p.rp:.2f}   E {p.E:.2f}   lambda {p.lam:.2f}",
        f"drum OD {p.drum_od:.1f}   motor plate OD {p.flange_od:.1f}   "
        f"length {z['motor_face'] - z.get('far_plate0', 0.0):.1f} mm (far face to motor face)",
        f"output: {s.screw_count} x {s.screw.split()[0]} on R {p.screw_r:.2f}, "
        f"disc holes {p.hole_d:.2f} + tol, web {p.web:.2f}",
        f"screw length {p.screw_len:.1f} mm needed -> buy "
        f"{s.screw.split()[0]}x{p.screw_buy}" if p.screw_buy else "screw: no stock length fits",
        f"eccentric on the D-cut: {p.engage:.1f} mm",
    ]
    if p.errors:
        lines += ["", "CANNOT BUILD:"] + ["  - " + e for e in p.errors]
    if p.warnings:
        lines += ["", "Warnings:"] + ["  - " + w for w in p.warnings]
    return "\n".join(lines)


if __name__ == "__main__":
    for spec in (Spec(), Spec(rollers=21), Spec(two_bearings=False),
                 Spec(motor="NEMA 23 (6.35 mm D-shaft)", big_bearing="6812  60x78x10",
                      cam_bearing="6000 10x26x8"),
                 Spec(rollers=11, big_bearing="6806  30x42x7", cam_bearing="688  8x16x5",
                      screw="M3 cap screw", motor="NEMA 14 (5 mm D-shaft)")):
        p = plan(spec)
        print("=" * 72)
        print(summary(p))
        prof = disc_profile(p)
        rr = [math.hypot(x - p.E, y) for x, y in prof]
        print(f"profile: {len(prof)} pts, radius {min(rr):.2f}..{max(rr):.2f} "
              f"(expect ~{p.rmin:.2f}..{p.R - p.rp - p.spec.clearance + p.E:.2f})")

    print("=" * 72)
    print("mesh check: smallest roller gap for disc A / disc B (should both equal the clearance)")
    bad = 0
    for n in range(11, 26):
        p = plan(Spec(rollers=n))
        a, b = mesh_check(p)
        flag = "" if min(a, b) > 0 else "   <-- INTERFERES"
        bad += bool(flag)
        print(f"  {n:2d} rollers: {a:+.3f} / {b:+.3f} mm{flag}")
    raise SystemExit(1 if bad else 0)
