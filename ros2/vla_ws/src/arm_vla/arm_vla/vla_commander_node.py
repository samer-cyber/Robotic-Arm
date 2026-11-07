#!/usr/bin/env python3
"""VLA commander: text in, arm motion out.

    ros2 service call /vla/execute arm_vla_interfaces/srv/MoveToTarget \
        "{command: 'touch the red cup', dry_run: false}"

    ros2 run arm_vla vla_commander_node --ros-args -p simulate:=true -- --once "touch the red"

About the "language" in vision-language-action: what is here is keyword
matching against the detector vocabulary, not a language model. "touch the red
cup" works because "red" is a colour band the detector knows; "cup" is ignored
unless you switch the detector to YOLO. That is enough to drive the pipeline
end to end, and it keeps the failure modes legible. The place a real VLM slots
in is _resolve_label(): swap keyword matching for a model that returns a
grounded label or a bounding box, and nothing downstream changes.

Motion sequence for a touch, and why it is not just "go there":
    home -> standoff -> target -> dwell -> standoff -> home
The standoff point is pulled back along the radial direction. Going straight to
a target sweeps the forearm through whatever is between here and there,
including the object itself.
"""

from __future__ import annotations

import argparse
import math
import sys
import threading
import time
from typing import Optional

import rclpy
from rclpy.utilities import remove_ros_args
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import JointState

from arm_vla_interfaces.srv import DetectTarget, MoveToTarget
from arm_vla.vocabulary import COLOR_BANDS, STOP_WORDS
from arm_vla.kinematics_3dof import (
    ArmGeometry, JointLimits, SafetyEnvelope, IKSolution,
    forward_kinematics, solve_both,
)

try:
    from armlink import connect, ArmError
except ImportError as exc:      # pragma: no cover
    connect = None
    ArmError = RuntimeError
    _IMPORT_ERROR = exc
else:
    _IMPORT_ERROR = None


JOINT_NAMES = ("J1", "J2", "J3")


class VlaCommanderNode(Node):

    def __init__(self, cli_args=None):
        super().__init__("vla_commander_node")

        # ------------------------------------------------------------ params
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("simulate", False)

        self.declare_parameter("geometry.l0_mm", 107.65)
        self.declare_parameter("geometry.l1_mm", 180.0)
        self.declare_parameter("geometry.l2_mm", 65.0)
        self.declare_parameter("geometry.base_offset_mm", 0.0)

        self.declare_parameter("limits.base_deg", [-180.0, 180.0])
        self.declare_parameter("limits.shoulder_deg", [-20.0, 135.0])
        self.declare_parameter("limits.elbow_deg", [-150.0, 0.0])

        self.declare_parameter("envelope.z_floor_mm", 10.0)
        self.declare_parameter("envelope.z_ceiling_mm", 600.0)
        self.declare_parameter("envelope.min_radius_mm", 60.0)

        self.declare_parameter("home_pose_deg", [0.0, 90.0, -90.0])
        self.declare_parameter("approach_standoff_mm", 60.0)
        self.declare_parameter("touch_backoff_mm", 5.0)
        self.declare_parameter("feed_transit", 0.35)
        self.declare_parameter("feed_approach", 0.12)
        self.declare_parameter("dwell_s", 1.0)
        self.declare_parameter("move_timeout_s", 45.0)
        self.declare_parameter("return_home_after", True)
        self.declare_parameter("detect_timeout_s", 8.0)
        self.declare_parameter("publish_joint_states", True)

        g = self.get_parameter
        self.geom = ArmGeometry(
            l0=float(g("geometry.l0_mm").value),
            l1=float(g("geometry.l1_mm").value),
            l2=float(g("geometry.l2_mm").value),
            base_offset=float(g("geometry.base_offset_mm").value),
        )
        self.limits = JointLimits(
            base=tuple(g("limits.base_deg").value),
            shoulder=tuple(g("limits.shoulder_deg").value),
            elbow=tuple(g("limits.elbow_deg").value),
        )
        self.envelope = SafetyEnvelope(
            z_floor=float(g("envelope.z_floor_mm").value),
            z_ceiling=float(g("envelope.z_ceiling_mm").value),
            min_radius=float(g("envelope.min_radius_mm").value),
            max_radius=self.geom.reach_max + self.geom.base_offset,
        )
        self.home_pose = [float(v) for v in g("home_pose_deg").value]
        self.standoff = float(g("approach_standoff_mm").value)
        self.touch_backoff = float(g("touch_backoff_mm").value)
        self.feed_transit = float(g("feed_transit").value)
        self.feed_approach = float(g("feed_approach").value)
        self.dwell = float(g("dwell_s").value)
        self.move_timeout = float(g("move_timeout_s").value)
        self.return_home = bool(g("return_home_after").value)
        self.detect_timeout = float(g("detect_timeout_s").value)

        self.get_logger().info(
            f"geometry l0={self.geom.l0} l1={self.geom.l1} l2={self.geom.l2} "
            f"offset={self.geom.base_offset} mm, reach "
            f"{self.geom.reach_min:.0f}..{self.geom.reach_max:.0f} mm")

        # ------------------------------------------------------------ hardware
        self.arm = None
        self.arm_lock = threading.RLock()
        self._connect_arm()

        # ------------------------------------------------------------ ROS I/O
        # Three groups: the service blocks for a long time, the detect client
        # must be serviced while it blocks, and the joint-state timer must not
        # be starved by either.
        self.cb_srv = MutuallyExclusiveCallbackGroup()
        self.cb_client = MutuallyExclusiveCallbackGroup()
        self.cb_timer = MutuallyExclusiveCallbackGroup()

        self.detect_client = self.create_client(
            DetectTarget, "/vla/detect", callback_group=self.cb_client)
        self.execute_srv = self.create_service(
            MoveToTarget, "/vla/execute", self._on_execute, callback_group=self.cb_srv)
        self.target_pub = self.create_publisher(PointStamped, "/vla/commanded_point", 10)

        if bool(g("publish_joint_states").value):
            self.js_pub = self.create_publisher(JointState, "/joint_states", 10)
            self.create_timer(0.2, self._publish_joint_states, callback_group=self.cb_timer)

        self.get_logger().info("commander ready. service: /vla/execute")

    # --------------------------------------------------------------- hardware
    def _connect_arm(self):
        if connect is None:
            self.get_logger().error(
                f"cannot import armlink ({_IMPORT_ERROR}). Did you build and source "
                f"the workspace? armlink_py must be on the ament python path.")
            return
        simulate = bool(self.get_parameter("simulate").value)
        port = self.get_parameter("port").value
        baud = int(self.get_parameter("baud").value)
        try:
            self.arm = connect(port=None if simulate else port,
                               baud=baud, simulate=simulate)
        except Exception as exc:
            self.get_logger().error(
                f"could not open the arm on {port}: {exc}\n"
                f"  - is the ESP32 attached to WSL?  usbipd attach --wsl --busid <id>\n"
                f"  - are you in the dialout group?  sudo usermod -aG dialout $USER\n"
                f"  - is control.py or a serial monitor already holding the port?\n"
                f"  - to work without hardware: -p simulate:=true")
            return
        mode = "SIMULATOR" if simulate else port
        joints = ", ".join(j.name for j in self.arm.meta)
        self.get_logger().info(f"arm connected on {mode}. joints reported: {joints}")

    def _publish_joint_states(self):
        if self.arm is None:
            return
        # Skip rather than queue: the serial link is single-threaded and a move
        # in progress is more important than a status frame.
        if not self.arm_lock.acquire(blocking=False):
            return
        try:
            st = self.arm.state()
        except Exception as exc:
            # Swallowing this hid a dying serial link: the topic kept
            # publishing, just with nothing in it.
            self.get_logger().warn(f"state() failed: {exc}", throttle_duration_sec=5.0)
            return
        finally:
            self.arm_lock.release()

        if not st.joints:
            # An empty JointState is worse than no JointState - it looks like a
            # published, healthy topic that happens to describe no joints, and
            # that is exactly how a broken link went unnoticed.
            self.get_logger().warn(
                "firmware returned no joints. The link is up but the reply was "
                "empty or truncated - re-attach the ESP32 and restart this node.",
                throttle_duration_sec=5.0)
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [j.name for j in st.joints]
        msg.position = [math.radians(j.position) for j in st.joints]
        self.js_pub.publish(msg)

    # ---------------------------------------------------------------- parsing
    def _resolve_label(self, command: str) -> Optional[str]:
        """Keyword match, deliberately. See the module docstring."""
        words = command.lower().replace(",", " ").split()
        for w in words:
            if w in COLOR_BANDS:
                return w
        # Fall through to the last noun-ish token so YOLO labels still work.
        nouns = [w for w in words if w not in STOP_WORDS]
        return nouns[-1] if nouns else None

    # -------------------------------------------------------------- detection
    def _detect(self, label: str):
        if not self.detect_client.wait_for_service(timeout_sec=3.0):
            return None, "vision node is not running (/vla/detect never appeared)"
        req = DetectTarget.Request()
        req.label = label
        future = self.detect_client.call_async(req)

        deadline = time.time() + self.detect_timeout
        while not future.done() and time.time() < deadline:
            time.sleep(0.02)
        if not future.done():
            return None, f"vision did not answer within {self.detect_timeout:.0f}s"

        res = future.result()
        if res is None:
            return None, "vision call failed"
        if not res.success:
            return None, res.message
        return res, ""

    # ----------------------------------------------------------------- motion
    def _goto(self, sol: IKSolution, feed: float) -> tuple[bool, str]:
        with self.arm_lock:
            reply = self.arm.goto_named(sol.as_named(), feed=feed)
            if not reply:
                return False, f"firmware refused the move: {reply.err_name} {reply.err_detail}"
            if not self.arm.wait_idle(timeout=self.move_timeout):
                self.arm.stop()
                return False, f"move did not finish within {self.move_timeout:.0f}s"
        return True, ""

    def _solve(self, x: float, y: float, z: float, what: str) -> tuple[Optional[IKSolution], str]:
        sol = solve_both(x, y, z, self.geom, self.limits, self.envelope)
        if not sol:
            return None, (f"{what} ({x:.1f}, {y:.1f}, {z:.1f}) mm rejected: "
                          f"[{sol.status}] {sol.detail}")
        return sol, ""

    # ---------------------------------------------------------------- service
    def _on_execute(self, req: MoveToTarget.Request,
                    res: MoveToTarget.Response) -> MoveToTarget.Response:
        command = (req.command or "").strip()
        self.get_logger().info(f"command: {command!r} (dry_run={req.dry_run})")

        if self.arm is None and not req.dry_run:
            res.success, res.message = False, "no arm connection"
            return res

        label = self._resolve_label(command)
        if not label:
            res.success, res.message = False, f"no target label found in {command!r}"
            return res

        detection, why = self._detect(label)
        if detection is None:
            res.success, res.message = False, f"detect '{label}': {why}"
            return res

        p = detection.point_base.point
        tx, ty, tz = p.x, p.y, p.z

        # Pull the commanded point slightly back toward the base: the detection
        # is the visible front face of the object, and driving to its centroid
        # means driving into it.
        r_xy = math.hypot(tx, ty)
        if r_xy > 1e-6 and self.touch_backoff:
            k = (r_xy - self.touch_backoff) / r_xy
            tx, ty = tx * k, ty * k

        touch_sol, why = self._solve(tx, ty, tz, "target")
        if touch_sol is None:
            res.success, res.message = False, why
            return res

        r_xy = math.hypot(tx, ty)
        k = (r_xy - self.standoff) / r_xy if r_xy > 1e-6 else 1.0
        sx, sy, sz = tx * k, ty * k, tz
        stand_sol, why = self._solve(sx, sy, sz, "standoff")
        if stand_sol is None:
            res.success, res.message = False, why + " (try a smaller approach_standoff_mm)"
            return res

        res.theta_base = float(touch_sol.base)
        res.theta_shoulder = float(touch_sol.shoulder)
        res.theta_elbow = float(touch_sol.elbow)
        res.target_base = detection.point_base

        commanded = PointStamped()
        commanded.header = detection.point_base.header
        commanded.point.x, commanded.point.y, commanded.point.z = tx, ty, tz
        self.target_pub.publish(commanded)

        summary = (f"{label} at ({p.x:.1f}, {p.y:.1f}, {p.z:.1f}) mm -> "
                   f"J1 {touch_sol.base:+.2f}  J2 {touch_sol.shoulder:+.2f}  "
                   f"J3 {touch_sol.elbow:+.2f} deg")

        if req.dry_run:
            fk = forward_kinematics(touch_sol.base, touch_sol.shoulder,
                                    touch_sol.elbow, self.geom)
            res.success = True
            res.message = (f"DRY RUN. {summary}. FK check "
                           f"({fk[0]:.1f}, {fk[1]:.1f}, {fk[2]:.1f}) mm")
            self.get_logger().info(res.message)
            return res

        ok, why = self._execute_touch(stand_sol, touch_sol)
        res.success = ok
        res.message = summary if ok else why
        self.get_logger().info(res.message if ok else f"FAILED: {why}")
        return res

    def _execute_touch(self, stand: IKSolution, touch: IKSolution) -> tuple[bool, str]:
        try:
            with self.arm_lock:
                st = self.arm.state()
                if st.estopped:
                    return False, "arm is in E-STOP. clear it first."
                if not st.enabled:
                    self.get_logger().info("enabling drivers")
                    if not self.arm.enable(True):
                        return False, "could not enable the drivers"

            for stage, sol, feed in (
                ("standoff", stand, self.feed_transit),
                ("touch",    touch, self.feed_approach),
            ):
                self.get_logger().info(
                    f"  {stage}: J1 {sol.base:+.2f} J2 {sol.shoulder:+.2f} "
                    f"J3 {sol.elbow:+.2f} at feed {feed}")
                ok, why = self._goto(sol, feed)
                if not ok:
                    self._safe_retreat()
                    return False, f"{stage} move failed: {why}"

            time.sleep(self.dwell)

            ok, why = self._goto(stand, self.feed_approach)
            if not ok:
                return False, f"retreat failed: {why}"

            if self.return_home:
                home = IKSolution(True, "home",
                                  base=self.home_pose[0],
                                  shoulder=self.home_pose[1],
                                  elbow=self.home_pose[2])
                ok, why = self._goto(home, self.feed_transit)
                if not ok:
                    return False, f"return home failed: {why}"
            return True, ""

        except ArmError as exc:
            self._safe_retreat()
            return False, f"arm error: {exc}"
        except Exception as exc:                     # noqa: BLE001
            # Anything unexpected while the arm is energised gets an E-STOP.
            # A stuck Python exception is not a reason to leave a stepper arm
            # mid-trajectory.
            self._estop(f"unhandled exception: {exc}")
            return False, f"aborted, E-STOP applied: {exc}"

    def _safe_retreat(self):
        try:
            with self.arm_lock:
                self.arm.stop()
        except Exception:
            pass

    def _estop(self, why: str):
        self.get_logger().error(f"E-STOP: {why}")
        try:
            with self.arm_lock:
                self.arm.estop()
        except Exception:
            pass

    # -------------------------------------------------------------- CLI entry
    def run_once(self, command: str, dry_run: bool) -> int:
        req = MoveToTarget.Request()
        req.command = command
        req.dry_run = dry_run
        res = self._on_execute(req, MoveToTarget.Response())
        print(("OK   " if res.success else "FAIL ") + res.message)
        return 0 if res.success else 1


def main(args=None):
    ros_argv = remove_ros_args(sys.argv)[1:]
    parser = argparse.ArgumentParser(prog="vla_commander_node")
    parser.add_argument("--once", metavar="COMMAND",
                        help="run one command and exit, instead of serving")
    parser.add_argument("--dry-run", action="store_true",
                        help="solve and report, move nothing")
    cli, _ = parser.parse_known_args(ros_argv)

    rclpy.init(args=args)
    node = VlaCommanderNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    code = 0
    try:
        if cli.once:
            spin_thread = threading.Thread(target=executor.spin, daemon=True)
            spin_thread.start()
            time.sleep(0.5)          # let discovery find the vision service
            code = node.run_once(cli.once, cli.dry_run)
        else:
            executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if node.arm is not None:
                node.arm.close()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == "__main__":
    raise SystemExit(main())
