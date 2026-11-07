#!/usr/bin/env python3
"""Solve the camera-to-arm-base transform from touched correspondences.

"Configurable static extrinsic transform" is easy to write and miserable to
guess. Six numbers hand-tuned by eye will be wrong by centimetres and you will
blame the IK. This measures them instead.

Method: put a coloured object somewhere the arm can reach. Jog the arm until
its tip touches the object, and press Enter. The script records two things --
where the arm says its tip is (forward kinematics on the current joint angles)
and where the camera says the object is (camera optical frame). Repeat at four
or more well-spread positions, then Kabsch/Umeyama solves the rigid transform
that best maps one cloud onto the other.

    ros2 run arm_vla calibrate_extrinsics --ros-args -p port:=/dev/ttyUSB0

Spread the samples out. Four points clustered in one corner of the workspace
give a transform that is excellent there and drifts badly everywhere else. Vary
height too, not just the table plane -- samples that are coplanar leave the
out-of-plane rotation poorly conditioned.
"""

from __future__ import annotations

import math
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from arm_vla_interfaces.srv import DetectTarget
from arm_vla.kinematics_3dof import ArmGeometry, forward_kinematics

try:
    from armlink import connect
except ImportError as exc:      # pragma: no cover
    connect = None
    _IMPORT_ERROR = exc


def kabsch(p_cam: np.ndarray, p_base: np.ndarray):
    """Best-fit rotation and translation mapping p_cam -> p_base."""
    c_cam = p_cam.mean(axis=0)
    c_base = p_base.mean(axis=0)
    H = (p_cam - c_cam).T @ (p_base - c_base)
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1.0, 1.0, d]) @ U.T
    t = c_base - R @ c_cam
    return R, t


def matrix_to_rpy(R: np.ndarray):
    """Inverse of the ZYX construction used by the vision node."""
    if abs(R[2, 0]) > 1.0 - 1e-9:                 # gimbal lock
        yaw = 0.0
        pitch = math.copysign(math.pi / 2.0, -R[2, 0])
        roll = math.atan2(-R[0, 1], R[1, 1])
    else:
        pitch = -math.asin(max(-1.0, min(1.0, R[2, 0])))
        roll = math.atan2(R[2, 1], R[2, 2])
        yaw = math.atan2(R[1, 0], R[0, 0])
    return tuple(math.degrees(v) for v in (roll, pitch, yaw))


class CalibNode(Node):
    def __init__(self):
        super().__init__("calibrate_extrinsics")
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("simulate", False)
        self.declare_parameter("label", "red")
        self.declare_parameter("geometry.l0_mm", 107.65)
        self.declare_parameter("geometry.l1_mm", 180.0)
        self.declare_parameter("geometry.l2_mm", 65.0)
        self.declare_parameter("geometry.base_offset_mm", 0.0)

        g = self.get_parameter
        self.geom = ArmGeometry(
            l0=float(g("geometry.l0_mm").value),
            l1=float(g("geometry.l1_mm").value),
            l2=float(g("geometry.l2_mm").value),
            base_offset=float(g("geometry.base_offset_mm").value),
        )
        self.label = g("label").value
        self.client = self.create_client(DetectTarget, "/vla/detect")

        if connect is None:
            raise SystemExit(f"cannot import armlink: {_IMPORT_ERROR}")
        simulate = bool(g("simulate").value)
        self.arm = connect(port=None if simulate else g("port").value,
                           baud=int(g("baud").value), simulate=simulate)

    def tip_now(self):
        """Where the arm believes its tip is, from joint angles + FK."""
        st = self.arm.state()
        by = st.by_name()
        missing = [n for n in ("J1", "J2", "J3") if n not in by]
        if missing:
            raise RuntimeError(f"arm did not report {missing}; is it homed and attached?")
        return forward_kinematics(by["J1"], by["J2"], by["J3"], self.geom), by

    def detect_now(self):
        if not self.client.wait_for_service(timeout_sec=3.0):
            raise RuntimeError("/vla/detect not available - is vision_spatial_node running?")
        req = DetectTarget.Request()
        req.label = self.label
        fut = self.client.call_async(req)
        deadline = time.time() + 10.0
        while not fut.done() and time.time() < deadline:
            time.sleep(0.02)
        if not fut.done():
            raise RuntimeError("vision timed out")
        res = fut.result()
        # Before calibration the service reports success=False by design, but
        # the camera-frame point is still valid and is exactly what we need.
        pc = res.point_camera.point
        if pc.x == 0.0 and pc.y == 0.0 and pc.z == 0.0:
            raise RuntimeError(f"no camera-frame point: {res.message}")
        return np.array([pc.x, pc.y, pc.z]), res.message


def main(args=None):
    rclpy.init(args=args)
    node = CalibNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    time.sleep(0.5)

    print(__doc__.split("Method:")[0].strip())
    print()
    print(f"target colour : {node.label}")
    print(f"geometry      : l0={node.geom.l0} l1={node.geom.l1} l2={node.geom.l2} mm")
    print("Jog the arm with jog.py or control.py in another terminal.")
    print("Touch the object, then press Enter here. Type 'done' to solve, 'q' to quit.")
    print()

    cam_pts, base_pts = [], []
    try:
        while True:
            reply = input(f"[{len(cam_pts)} samples] Enter to capture > ").strip().lower()
            if reply == "q":
                return 0
            if reply == "done":
                break
            try:
                (bx, by_, bz), joints = node.tip_now()
                p_cam, msg = node.detect_now()
            except Exception as exc:
                print(f"  skipped: {exc}")
                continue
            cam_pts.append(p_cam)
            base_pts.append([bx, by_, bz])
            print(f"  arm tip  ({bx:8.1f}, {by_:8.1f}, {bz:8.1f}) mm   "
                  f"J1 {joints['J1']:+.2f} J2 {joints['J2']:+.2f} J3 {joints['J3']:+.2f}")
            print(f"  camera   ({p_cam[0]:8.1f}, {p_cam[1]:8.1f}, {p_cam[2]:8.1f}) mm")

        if len(cam_pts) < 3:
            print("Need at least 3 samples (4+ recommended). Nothing solved.")
            return 1

        P = np.array(cam_pts)
        Q = np.array(base_pts)
        R, t = kabsch(P, Q)
        residuals = np.linalg.norm((P @ R.T + t) - Q, axis=1)
        rpy = matrix_to_rpy(R)

        print()
        print(f"samples        {len(cam_pts)}")
        print(f"residual       mean {residuals.mean():.2f} mm   worst {residuals.max():.2f} mm")
        if residuals.max() > 15.0:
            print("               ^ that is high. Likely causes: the tip was not actually")
            print("                 touching, link lengths are wrong, or the samples are")
            print("                 too clustered. Fix before trusting this.")
        print()
        print("Paste into config/vla_params.yaml under vision_spatial_node:")
        print()
        print("    extrinsics.calibrated: true")
        print("    extrinsics.mode: matrix        # <- switch off the overhead model")
        print(f"    extrinsics.translation_mm: [{t[0]:.2f}, {t[1]:.2f}, {t[2]:.2f}]")
        print(f"    extrinsics.rotation_rpy_deg: [{rpy[0]:.3f}, {rpy[1]:.3f}, {rpy[2]:.3f}]")
        print()
        return 0
    except (KeyboardInterrupt, EOFError):
        return 130
    finally:
        try:
            node.arm.close()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
