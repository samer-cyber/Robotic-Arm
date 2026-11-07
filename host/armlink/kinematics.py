"""Forward and inverse kinematics for the 6-DOF arm.

Everything comes from description/arm.json - the same file the 3D twin and the
URDF are built from - so the maths, the picture and the simulator cannot
disagree about where a joint is.

    from armlink.kinematics import ArmModel
    m = ArmModel()
    T = m.fk([0, 30, -60, 0, 0, 0]).tcp          # 4x4, mm
    sol = m.ik(target_pos=[180, 40, 200], seed=[0, 30, -60, 0, 0, 0])
    if sol: arm.goto_named(m.to_firmware(sol.q))

Angles here are MODEL angles (zero = the CAD pose, arm straight up, right-hand
rule about each axis in arm.json). The firmware counts from wherever the arm
was taught, possibly the other way round; `firmware_map` in arm.json converts:

    model = sign * firmware + offset

Run this file directly for a self-test (FK -> IK round trips).
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable, Optional, Sequence

import numpy as np

DEFAULT_SPEC = Path(__file__).resolve().parents[2] / "description" / "arm.json"


def axis_angle(axis: np.ndarray, rad: float) -> np.ndarray:
    """3x3 rotation, Rodrigues."""
    x, y, z = axis / np.linalg.norm(axis)
    c, s = math.cos(rad), math.sin(rad)
    C = 1.0 - c
    return np.array([[c + x * x * C, x * y * C - z * s, x * z * C + y * s],
                     [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
                     [z * x * C - y * s, z * y * C + x * s, c + z * z * C]])


def rot_error(R_target: np.ndarray, R: np.ndarray) -> np.ndarray:
    """Axis-angle vector (rad, world frame) that rotates R onto R_target."""
    E = R_target @ R.T
    cos = max(-1.0, min(1.0, (np.trace(E) - 1.0) / 2.0))
    ang = math.acos(cos)
    if ang < 1e-9:
        return np.zeros(3)
    if math.pi - ang < 1e-6:                      # 180 deg: pick any valid axis
        w, v = np.linalg.eigh((E + np.eye(3)) / 2.0)
        return v[:, int(np.argmax(w))] * ang
    vec = np.array([E[2, 1] - E[1, 2], E[0, 2] - E[2, 0], E[1, 0] - E[0, 1]])
    return vec / (2.0 * math.sin(ang)) * ang


def quat_to_matrix(q: Sequence[float]) -> np.ndarray:
    """(x, y, z, w) -> 3x3. The order three.js uses."""
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


@dataclass
class FK:
    frames: list            # 4x4 per joint, the CHILD link frame (mm)
    tcp: np.ndarray         # 4x4

    @property
    def tcp_pos(self) -> np.ndarray:
        return self.tcp[:3, 3]


@dataclass
class IKSolution:
    q: list                 # model degrees, one per joint
    ok: bool
    pos_err_mm: float
    rot_err_deg: float
    iterations: int
    reason: str = ""
    fk: Optional[FK] = field(default=None, repr=False)

    def __bool__(self) -> bool:
        return self.ok


class ArmModel:
    def __init__(self, spec_path: Path | str = DEFAULT_SPEC):
        self.spec = json.loads(Path(spec_path).read_text())
        js = self.spec["joints"]
        self.names = [j["name"] for j in js]
        self.n = len(js)
        self.pos0 = np.array([j["position"] for j in js], float)
        self.axes = np.array([j["axis"] for j in js], float)
        self.axes /= np.linalg.norm(self.axes, axis=1, keepdims=True)
        self.tcp0 = np.array(self.spec["tcp"]["position"], float)

        fm = self.spec.get("firmware_map", {})
        self.sign = np.array([float(fm.get(n, {}).get("sign", 1)) for n in self.names])
        self.offset = np.array([float(fm.get(n, {}).get("offset", 0.0)) for n in self.names])

        # arm.json limits are FIRMWARE limits (they are what the firmware will
        # accept); carry them into model space through the map.
        fw_lo = np.array([j["limit"][0] for j in js], float)
        fw_hi = np.array([j["limit"][1] for j in js], float)
        a, b = self.sign * fw_lo + self.offset, self.sign * fw_hi + self.offset
        self.lo, self.hi = np.minimum(a, b), np.maximum(a, b)

    # ------------------------------------------------------------ mapping --
    def to_model(self, fw) -> np.ndarray:
        """Firmware angles (dict by name, or list in joint order) -> model."""
        if isinstance(fw, dict):
            fw = [float(fw.get(n, 0.0)) for n in self.names]
        return self.sign * np.asarray(fw, float)[: self.n] + self.offset

    def to_firmware(self, q: Iterable[float]) -> dict:
        q = np.asarray(list(q), float)
        fw = (q - self.offset) / self.sign
        return {n: round(float(v), 3) for n, v in zip(self.names, fw)}

    # ----------------------------------------------------------------- FK --
    def fk(self, q_deg: Sequence[float]) -> FK:
        q = np.radians(np.asarray(q_deg, float))
        T = np.eye(4)
        prev = np.zeros(3)
        frames = []
        for i in range(self.n):
            step = np.eye(4)
            step[:3, 3] = self.pos0[i] - prev
            step[:3, :3] = axis_angle(self.axes[i], q[i])
            T = T @ step
            frames.append(T.copy())
            prev = self.pos0[i]
        tcp = T.copy()
        tcp[:3, 3] = T[:3, 3] + T[:3, :3] @ (self.tcp0 - prev)
        return FK(frames, tcp)

    def jacobian(self, f: FK) -> np.ndarray:
        """6xN geometric Jacobian at the TCP. Rows: v (mm/rad), w (rad/rad)."""
        J = np.zeros((6, self.n))
        p = f.tcp[:3, 3]
        for i, T in enumerate(f.frames):
            z = T[:3, :3] @ self.axes[i]
            J[:3, i] = np.cross(z, p - T[:3, 3])
            J[3:, i] = z
        return J

    # ----------------------------------------------------------------- IK --
    def ik(self, target_pos: Sequence[float], target_rot: Optional[np.ndarray] = None,
           seed: Optional[Sequence[float]] = None, *, tol_mm: float = 1.0,
           tol_deg: float = 1.0, max_iter: int = 150, restarts: int = 6,
           rot_weight_mm_per_rad: float = 120.0, rng_seed: int = 1) -> IKSolution:
        """Damped least squares, joint limits enforced every step.

        Position only when target_rot is None - the six joints then have
        three spare degrees of freedom, and starting from `seed` (the current
        pose) gives the smallest motion that gets there, which is what you
        want when dragging a target around.

        Restarts from random poses only if the seeded attempt fails.
        """
        target_pos = np.asarray(target_pos, float)
        seed = np.zeros(self.n) if seed is None else np.clip(np.asarray(seed, float), self.lo, self.hi)
        rng = np.random.default_rng(rng_seed)
        best: Optional[IKSolution] = None
        starts = [seed] + [rng.uniform(self.lo, self.hi) for _ in range(restarts)]
        for attempt, q0 in enumerate(starts):
            sol = self._solve(target_pos, target_rot, q0, tol_mm, tol_deg, max_iter,
                              rot_weight_mm_per_rad)
            if best is None or self._score(sol) < self._score(best):
                best = sol
            if sol.ok:
                break
        assert best is not None
        if not best.ok:
            best.reason = (f"closest reach is {best.pos_err_mm:.1f} mm"
                           + (f" / {best.rot_err_deg:.1f} deg" if target_rot is not None else "")
                           + " away - outside the workspace or blocked by joint limits")
        return best

    @staticmethod
    def _score(s: IKSolution) -> float:
        return s.pos_err_mm + s.rot_err_deg

    def _solve(self, p_t, R_t, q0, tol_mm, tol_deg, max_iter, w) -> IKSolution:
        lo, hi = np.radians(self.lo), np.radians(self.hi)
        q = np.radians(q0)
        lam = 8.0
        max_step = math.radians(8.0)
        f = self.fk(np.degrees(q))
        pe = re = float("inf")
        it = 0
        for it in range(1, max_iter + 1):
            e_p = p_t - f.tcp[:3, 3]
            pe = float(np.linalg.norm(e_p))
            if R_t is not None:
                e_r = rot_error(R_t, f.tcp[:3, :3])
                re = math.degrees(float(np.linalg.norm(e_r)))
                e = np.concatenate([e_p, w * e_r])
                J = self.jacobian(f)
                J[3:] *= w
            else:
                re = 0.0
                e = e_p
                J = self.jacobian(f)[:3]
            if pe <= tol_mm and re <= tol_deg:
                break
            JJt = J @ J.T + (lam ** 2) * np.eye(J.shape[0])
            dq = J.T @ np.linalg.solve(JJt, e)
            mx = float(np.max(np.abs(dq)))
            if mx > max_step:
                dq *= max_step / mx
            q = np.clip(q + dq, lo, hi)
            f = self.fk(np.degrees(q))
        ok = pe <= tol_mm and re <= tol_deg
        return IKSolution(q=[round(float(v), 4) for v in np.degrees(q)], ok=ok,
                          pos_err_mm=pe, rot_err_deg=re, iterations=it, fk=f)


def self_test(n: int = 300) -> bool:
    m = ArmModel()
    rng = np.random.default_rng(7)
    fails, worst = 0, 0.0
    for _ in range(n):
        q = rng.uniform(m.lo * 0.8, m.hi * 0.8)
        f = m.fk(q)
        # seed near the answer, the way a twin drag or teleop stream would
        seed = q + rng.normal(0, 10, m.n)
        s = m.ik(f.tcp_pos, f.tcp[:3, :3], seed=seed)
        if not s:
            fails += 1
        worst = max(worst, s.pos_err_mm)
    print(f"pose IK, seeded within ~10 deg: {n - fails}/{n} solved, worst {worst:.2f} mm")
    zero = m.fk([0] * m.n).tcp_pos
    print("tcp at zero pose (mm):", np.round(zero, 1))
    for name, q in [("J2 +90", [0, 90, 0, 0, 0, 0]), ("J3 -90", [0, 0, -90, 0, 0, 0])]:
        print(f"tcp at {name}:", np.round(m.fk(q).tcp_pos, 1))
    return fails == 0


if __name__ == "__main__":
    raise SystemExit(0 if self_test() else 1)
