#!/usr/bin/env python3
"""Spatial detection node for the OAK-D SR.

Finds a named target in the image, reads its depth, de-projects to 3D, and
transforms into the arm base frame. Answers one measurement per service call.

Why a service and not a stream: a touch command needs one committed number.
Streaming the latest frame invites the arm to chase depth noise. The service
returns the median of the last few agreeing frames instead.

    ros2 service call /vla/detect arm_vla_interfaces/srv/DetectTarget "{label: red}"

Topics in  : detect image, depth image, and the camera_info for both
Topics out : /vla/target_point (PointStamped, mm, base frame)
             /vla/debug_image  (annotated, for rqt_image_view)
"""

from __future__ import annotations

import math
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import CameraInfo, Image
import message_filters

try:
    from vision_msgs.msg import Detection3DArray
except ImportError:                       # only needed for detector=spatial_nn
    Detection3DArray = None

from arm_vla_interfaces.srv import DetectTarget
from arm_vla.vocabulary import COLOR_BANDS


@dataclass
class Detection:
    ok: bool
    u: int = 0
    v: int = 0
    bbox: Tuple[int, int, int, int] = (0, 0, 0, 0)   # x, y, w, h
    confidence: float = 0.0
    reason: str = ""


@dataclass
class Measurement:
    x_cam: float
    y_cam: float
    z_cam: float
    u: int
    v: int
    depth_mm: float
    confidence: float
    samples: int


def rpy_to_matrix(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    """ZYX intrinsic (yaw, then pitch, then roll) -- the ROS convention."""
    r, p, y = (math.radians(v) for v in (roll_deg, pitch_deg, yaw_deg))
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ], dtype=float)


# Depth 1, best effort. Over the WSL USB bridge the driver can burst frames
# faster than this node consumes them; a deeper queue just means the detection
# you finally act on is several frames stale. Keep only the newest.
CAM_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)


def overhead_transform(height_mm: float, offset_x_mm: float, offset_y_mm: float,
                       yaw_deg: float):
    """Camera bolted above the desk, optical axis pointing straight down.

    Base frame  : origin on the desk on the J1 axis, +Z up.
    Camera frame: +X image-right, +Y image-DOWN, +Z along the optical axis --
                  which for a top-down mount points at the table.

    At yaw 0 that gives:
        cam +Z (optical axis) -> base -Z     looking down
        cam +Y (image down)   -> base -X
        cam +X (image right)  -> base -Y

    yaw_deg spins the camera about the vertical, so 0/90/180/270 cover the four
    ways it can be bolted to the mount. Which one you have is not derivable
    from a height measurement -- put an object at a known spot and read the
    answer back, see LAUNCH.md Stage D.

    Height is the optical centre above the DESK, not above the shoulder axis.
    """
    R_topdown = np.array([[0.0, -1.0,  0.0],
                          [-1.0, 0.0,  0.0],
                          [0.0,  0.0, -1.0]])
    c, sn = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    R_yaw = np.array([[c, -sn, 0.0],
                      [sn,  c, 0.0],
                      [0.0, 0.0, 1.0]])
    return R_yaw @ R_topdown, np.array([offset_x_mm, offset_y_mm, height_mm])


class VisionSpatialNode(Node):

    def __init__(self):
        super().__init__("vision_spatial_node")

        # ------------------------------------------------------------ params
        # Right, not left: on this OAK-D SR the stereo depth map carries
        # frame_id oak_right_camera_optical_frame, so the right camera is the
        # one whose pixels line up with the depth image.
        self.declare_parameter("detect_image_topic", "/oak/right/image_raw")
        self.declare_parameter("detect_info_topic", "/oak/right/camera_info")
        self.declare_parameter("depth_image_topic", "/oak/stereo/image_raw")
        self.declare_parameter("depth_info_topic", "/oak/stereo/camera_info")

        self.declare_parameter("detector", "color")   # color | yolo | spatial_nn
        self.declare_parameter("default_label", "blue")
        self.declare_parameter("spatial_detections_topic", "/oak/nn/spatial_detections")
        self.declare_parameter("min_blob_area_px", 400)
        self.declare_parameter("sync_slop_s", 0.10)

        self.declare_parameter("depth_min_mm", 300.0)
        self.declare_parameter("depth_max_mm", 1000.0)
        self.declare_parameter("depth_roi_shrink", 0.5)    # sample inner 50% of bbox
        self.declare_parameter("min_depth_pixels", 30)
        # Table-plane rejection. The desk is ~610 mm below the camera, so
        # anything at or past this depth IS the desk. Two independent guards
        # because they fail differently: the depth cutoff stops table pixels
        # polluting the median in the first place, while the height check
        # catches a point that survived that but still reconstructs to table
        # level once the extrinsics are applied.
        self.declare_parameter("table_depth_mm", 595.0)
        self.declare_parameter("min_height_above_table_mm", 8.0)

        self.declare_parameter("samples_required", 2)
        self.declare_parameter("sample_window_s", 2.0)
        self.declare_parameter("agreement_mm", 25.0)

        # Extrinsics: camera optical frame -> arm base frame.
        # Fails closed. Nothing downstream is allowed to move the arm until you
        # have actually measured these and flipped the flag.
        self.declare_parameter("extrinsics.calibrated", False)
        # "overhead" builds the transform from the four mount numbers below.
        # "matrix" uses translation_mm/rotation_rpy_deg verbatim - that is the
        # form calibrate_extrinsics prints, and the escape hatch if the mount
        # is not actually square to the table.
        self.declare_parameter("extrinsics.mode", "overhead")
        self.declare_parameter("extrinsics.overhead.height_mm", 610.0)
        self.declare_parameter("extrinsics.overhead.offset_x_mm", 0.0)
        self.declare_parameter("extrinsics.overhead.offset_y_mm", 0.0)
        self.declare_parameter("extrinsics.overhead.yaw_deg", 0.0)
        self.declare_parameter("extrinsics.translation_mm", [0.0, 0.0, 610.0])
        self.declare_parameter("extrinsics.rotation_rpy_deg", [180.0, 0.0, -90.0])

        self.declare_parameter("publish_debug_image", True)

        g = self.get_parameter
        self.detector_kind = g("detector").value
        self.min_area = int(g("min_blob_area_px").value)
        self.depth_min = float(g("depth_min_mm").value)
        self.depth_max = float(g("depth_max_mm").value)
        self.roi_shrink = float(g("depth_roi_shrink").value)
        self.min_depth_px = int(g("min_depth_pixels").value)
        self.table_depth = float(g("table_depth_mm").value)
        self.min_height = float(g("min_height_above_table_mm").value)
        self.samples_required = int(g("samples_required").value)
        self.sample_window = float(g("sample_window_s").value)
        self.agreement_mm = float(g("agreement_mm").value)
        self.publish_debug = bool(g("publish_debug_image").value)

        self.calibrated = bool(g("extrinsics.calibrated").value)
        self.extrinsics_mode = str(g("extrinsics.mode").value).strip().lower()
        if self.extrinsics_mode == "overhead":
            self.R_cam_base, self.T_cam_base = overhead_transform(
                float(g("extrinsics.overhead.height_mm").value),
                float(g("extrinsics.overhead.offset_x_mm").value),
                float(g("extrinsics.overhead.offset_y_mm").value),
                float(g("extrinsics.overhead.yaw_deg").value))
        else:
            self.T_cam_base = np.array(list(g("extrinsics.translation_mm").value),
                                       dtype=float)
            self.R_cam_base = rpy_to_matrix(*list(g("extrinsics.rotation_rpy_deg").value))

        # ------------------------------------------------------------- state
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        # Subscriptions and the service live in different callback groups so a
        # blocking service call can wait while frames keep arriving. Same group,
        # or a single-threaded executor, and the wait below never sees new data.
        self.cb_subs = MutuallyExclusiveCallbackGroup()
        self.cb_srv = MutuallyExclusiveCallbackGroup()
        self.history: deque = deque(maxlen=60)
        self.current_label = str(g("default_label").value).strip().lower()
        self.depth_info: Optional[CameraInfo] = None
        self.detect_info: Optional[CameraInfo] = None
        self.frame_warned = False
        self.yolo = None

        if self.detector_kind == "yolo":
            self._load_yolo()
        if self.detector_kind == "spatial_nn":
            self._setup_spatial_nn(str(g("spatial_detections_topic").value))

        # --------------------------------------------------------------- I/O
        self.create_subscription(CameraInfo, g("depth_info_topic").value,
                                 self._on_depth_info, CAM_QOS,
                                 callback_group=self.cb_subs)
        self.create_subscription(CameraInfo, g("detect_info_topic").value,
                                 self._on_detect_info, CAM_QOS,
                                 callback_group=self.cb_subs)

        img_sub = message_filters.Subscriber(
            self, Image, g("detect_image_topic").value, qos_profile=CAM_QOS)
        depth_sub = message_filters.Subscriber(
            self, Image, g("depth_image_topic").value, qos_profile=CAM_QOS)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [img_sub, depth_sub], queue_size=10, slop=float(g("sync_slop_s").value))
        self.sync.registerCallback(self._on_frame)

        self.point_pub = self.create_publisher(PointStamped, "/vla/target_point", 10)
        self.debug_pub = self.create_publisher(Image, "/vla/debug_image", 2)
        self.srv = self.create_service(DetectTarget, "/vla/detect",
                                       self._on_detect_request,
                                       callback_group=self.cb_srv)

        # Name the four topics we depend on, and check somebody is publishing
        # them. Without this a single wrong topic name is indistinguishable
        # from a camera that simply is not seeing the object: _on_frame returns
        # early forever, and the node sits there logging nothing.
        self._topics = {
            "detect image": g("detect_image_topic").value,
            "detect info ": g("detect_info_topic").value,
            "depth image ": g("depth_image_topic").value,
            "depth info  ": g("depth_info_topic").value,
        }
        self._topics_checked = False
        self.create_timer(5.0, self._check_topics, callback_group=self.cb_subs)

        self.get_logger().info(
            f"vision up. detector={self.detector_kind} "
            f"image={g('detect_image_topic').value} depth={g('depth_image_topic').value}")
        self.get_logger().info(
            f"extrinsics mode={self.extrinsics_mode} "
            f"camera at base ({self.T_cam_base[0]:.1f}, {self.T_cam_base[1]:.1f}, "
            f"{self.T_cam_base[2]:.1f}) mm")
        if self.extrinsics_mode == "overhead":
            self.get_logger().info(
                "  top-down: a target on the desk directly under the camera "
                "should read base ("
                f"{self.T_cam_base[0]:.0f}, {self.T_cam_base[1]:.0f}, ~0) mm")
        if not self.calibrated:
            self.get_logger().warn(
                "EXTRINSICS NOT CALIBRATED. Camera-frame points will be reported, "
                "base-frame points will be refused, and the arm will not move. "
                "Run calibrate_extrinsics, put the numbers in vla_params.yaml, "
                "then set extrinsics.calibrated: true.")

    # ----------------------------------------------------------------- setup
    def _check_topics(self):
        """Runs once, 5 s after start: which configured topics have nobody
        publishing them?

        `ros2 topic echo` reports a missing topic as "Could not determine the
        type for the passed topic", which is the same message you get for a
        typo. Listing them here, next to the values this node actually loaded,
        removes the guesswork.
        """
        if self._topics_checked:
            return
        self._topics_checked = True

        missing = [(k, v) for k, v in self._topics.items()
                   if self.count_publishers(v) == 0]
        if not missing:
            self.get_logger().info("all four camera topics have a publisher")
            return
        self.get_logger().error("NO PUBLISHER FOUND on:")
        for k, v in missing:
            self.get_logger().error(f"    {k}  {v}")
        self.get_logger().error(
            "Nothing will be detected until these resolve. See what the driver "
            "really publishes:   ros2 topic list | grep oak")
        if any("camera_info" in v for _, v in missing):
            self.get_logger().error(
                "A missing camera_info is fatal, not cosmetic: the intrinsics "
                "in it are what turn a pixel and a depth into millimetres.")

    def _setup_spatial_nn(self, topic: str):
        """Consume 3D detections computed on the camera itself.

        The OAK-D SR can run MobileNet-SSD or YOLO on its Myriad X and emit
        detections that ALREADY carry a 3D position, because the chip has the
        depth map right there. That removes this node's entire middle section:
        no HSV mask, no bbox, no median depth, no de-projection. What stays is
        the part that matters -- the camera-to-base transform, the agreement
        gate, and the service contract -- so nothing downstream changes.

        Driver side, in ~/oak_sr_config.yaml:

            camera:
              i_pipeline_type: 'RGBD'      # Depth alone builds no NN node
              i_nn_type: spatial
            nn:
              i_nn_config_path: depthai_ros_driver/yolo

        Use the yolo config, not mobilenet: the bundled YOLO is yolov4-tiny on
        COCO-80, which has `cup` and `bottle`. The bundled MobileNet-SSD is
        VOC-20 -- it has `bottle` but no `cup`. Neither has `box`; the nearest
        COCO class is probably `book` or nothing at all.

        UNVERIFIED ON THIS HARDWARE. The spatial NN normally runs on the colour
        stream, and the OAK-D SR has no dedicated RGB socket -- that is exactly
        why `Depth` is the vendor pipeline type for it. Whether `RGBD` +
        `spatial` comes up on an SR is the one thing to test, and it costs one
        launch to find out. Detection positions arrive in METRES and are scaled
        to mm here.
        """
        if Detection3DArray is None:
            self.get_logger().error(
                "detector=spatial_nn needs vision_msgs: "
                "sudo apt install -y ros-jazzy-vision-msgs. Falling back to colour.")
            self.detector_kind = "color"
            return
        self._nn_labels_seen = set()
        self.create_subscription(Detection3DArray, topic, self._on_spatial,
                                 CAM_QOS, callback_group=self.cb_subs)
        self.get_logger().info(f"spatial NN mode, listening on {topic}")

    def _on_spatial(self, msg):
        with self.lock:
            label = self.current_label

        best = None
        for det in msg.detections:
            for hyp in det.results:
                cid = str(getattr(hyp.hypothesis, "class_id", "")).lower()
                score = float(getattr(hyp, "score", getattr(hyp.hypothesis, "score", 0.0)))
                if cid and cid not in self._nn_labels_seen:
                    self._nn_labels_seen.add(cid)
                    self.get_logger().info(f"NN class seen: {cid}")
                if label in cid and (best is None or score > best[0]):
                    best = (score, det)
        if best is None:
            return

        score, det = best
        pos = det.bbox.center.position          # metres, ROS convention
        x_cam, y_cam, z_cam = pos.x * 1000.0, pos.y * 1000.0, pos.z * 1000.0
        if z_cam <= 0.0 or z_cam >= self.table_depth:
            return
        if self.calibrated:
            z_base = float((self.R_cam_base @ np.array([x_cam, y_cam, z_cam]))[2]
                           + self.T_cam_base[2])
            if z_base <= self.min_height:
                return

        with self.lock:
            self.history.append((self.get_clock().now().nanoseconds * 1e-9, label,
                                 Measurement(x_cam, y_cam, z_cam, 0, 0,
                                             z_cam, score, 0)))

    def _load_yolo(self):
        try:
            from ultralytics import YOLO           # noqa: F401  (optional dep)
            self.yolo = YOLO("yolov8n.pt")
            self.get_logger().info("YOLOv8n loaded.")
        except Exception as exc:
            self.get_logger().warn(
                f"detector=yolo but ultralytics is unavailable ({exc}). "
                "Falling back to colour thresholding.")
            self.detector_kind = "color"

    def _on_depth_info(self, msg: CameraInfo):
        self.depth_info = msg

    def _on_detect_info(self, msg: CameraInfo):
        self.detect_info = msg

    # ------------------------------------------------------------- detection
    def _detect_color(self, bgr: np.ndarray, label: str) -> Detection:
        bands = COLOR_BANDS.get(label)
        if bands is None:
            return Detection(False, reason=f"unknown colour {label}, "
                                           f"known: {', '.join(sorted(COLOR_BANDS))}")
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = None
        for lo, hi in bands:
            band = cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
            mask = band if mask is None else cv2.bitwise_or(mask, band)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return Detection(False, reason=f"no {label} blob found")
        biggest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(biggest)
        if area < self.min_area:
            return Detection(False, reason=f"largest {label} blob is {int(area)} px, "
                                           f"minimum is {self.min_area}")
        x, y, w, h = cv2.boundingRect(biggest)
        # Centroid of the blob, not the centre of the bounding box -- for an
        # L-shaped or partly occluded object the box centre can sit off the
        # object entirely, and then the depth sample is of the background.
        m = cv2.moments(biggest)
        u = int(m["m10"] / m["m00"]) if m["m00"] else x + w // 2
        v = int(m["m01"] / m["m00"]) if m["m00"] else y + h // 2
        fill = area / float(max(w * h, 1))
        return Detection(True, u=u, v=v, bbox=(x, y, w, h), confidence=min(1.0, fill))

    def _detect_yolo(self, bgr: np.ndarray, label: str) -> Detection:
        try:
            results = self.yolo(bgr, verbose=False)[0]
        except Exception as exc:
            return Detection(False, reason=f"yolo inference failed: {exc}")
        best = None
        for box in results.boxes:
            name = results.names.get(int(box.cls[0]), "")
            if label.lower() not in name.lower():
                continue
            conf = float(box.conf[0])
            if best is None or conf > best[0]:
                best = (conf, box.xyxy[0].tolist())
        if best is None:
            return Detection(False, reason=f"yolo saw no {label}")
        conf, (x1, y1, x2, y2) = best
        x, y = int(x1), int(y1)
        w, h = int(x2 - x1), int(y2 - y1)
        return Detection(True, u=x + w // 2, v=y + h // 2, bbox=(x, y, w, h),
                         confidence=conf)

    # ----------------------------------------------------------------- depth
    def _sample_depth(self, depth: np.ndarray, bbox, scale_x: float, scale_y: float
                      ) -> Tuple[float, int, str]:
        """Median depth in mm over the inner part of the bounding box.

        Median, not mean: a few background pixels bleeding in at the object
        edge drag a mean badly, and stereo loves to punch holes right where the
        object meets the table.
        """
        x, y, w, h = bbox
        # shrink toward the centre, then map into depth-image pixel space
        cx, cy = x + w / 2.0, y + h / 2.0
        hw, hh = w * self.roi_shrink / 2.0, h * self.roi_shrink / 2.0
        x0 = int(max(0, (cx - hw) * scale_x))
        x1 = int(min(depth.shape[1], (cx + hw) * scale_x))
        y0 = int(max(0, (cy - hh) * scale_y))
        y1 = int(min(depth.shape[0], (cy + hh) * scale_y))
        if x1 <= x0 or y1 <= y0:
            return 0.0, 0, "depth ROI collapsed to nothing"

        roi = depth[y0:y1, x0:x1].astype(np.float32)
        valid = roi[(roi > self.depth_min) & (roi < self.depth_max)]
        if valid.size < self.min_depth_px:
            return 0.0, int(valid.size), (
                f"only {valid.size} valid depth pixels in the ROI "
                f"(need {self.min_depth_px}). Object may be out of stereo range, "
                f"textureless, or too close.")
        return float(np.median(valid)), int(valid.size), ""

    # ------------------------------------------------------------- main loop
    def _on_frame(self, img_msg: Image, depth_msg: Image):
        if self.depth_info is None:
            # Throttled, because this fires at frame rate. _check_topics()
            # already named the culprit at startup; this is the reminder for
            # anyone who scrolled past it.
            self.get_logger().warn(
                f"waiting for depth camera_info on "
                f"{self.get_parameter('depth_info_topic').value} - cannot "
                f"de-project without intrinsics",
                throttle_duration_sec=10.0)
            return

        if not self.frame_warned and self.detect_info is not None:
            if self.detect_info.header.frame_id != self.depth_info.header.frame_id:
                self.get_logger().warn(
                    f"Depth is aligned to frame '{self.depth_info.header.frame_id}' but you "
                    f"are detecting in '{self.detect_info.header.frame_id}'. Pixel (u,v) in "
                    f"one image does NOT point at the same thing in the other, so every 3D "
                    f"point will be wrong by the stereo baseline. Either set the stereo "
                    f"alignment to the detection camera, or point detect_image_topic at the "
                    f"camera depth is aligned to.")
            self.frame_warned = True

        try:
            bgr = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except Exception as exc:
            self.get_logger().warn(f"cv_bridge: {exc}")
            return

        # 32FC1 depth is metres by ROS convention; 16UC1 is millimetres.
        if depth.dtype == np.float32:
            depth = depth * 1000.0

        with self.lock:
            label = self.current_label

        det = (self._detect_yolo(bgr, label) if self.detector_kind == "yolo"
               else self._detect_color(bgr, label))

        if self.publish_debug:
            self._publish_debug(bgr, det, label)

        if not det.ok:
            return
        if self.detector_kind == "spatial_nn":
            # The camera already did the detection; the image path here exists
            # only to keep /vla/debug_image alive. Appending would double-count.
            return

        scale_x = depth.shape[1] / float(bgr.shape[1])
        scale_y = depth.shape[0] / float(bgr.shape[0])
        depth_mm, n_px, why = self._sample_depth(depth, det.bbox, scale_x, scale_y)
        if depth_mm <= 0.0:
            self.get_logger().debug(why)
            return

        # De-project with the DEPTH camera intrinsics, at depth-image scale.
        k = self.depth_info.k
        fx, fy, cx, cy = k[0], k[4], k[2], k[5]
        if fx == 0.0 or fy == 0.0:
            self.get_logger().warn("depth camera_info has zero focal length, ignoring frame")
            return
        u_d = det.u * scale_x
        v_d = det.v * scale_y
        x_cam = (u_d - cx) * depth_mm / fx
        y_cam = (v_d - cy) * depth_mm / fy
        z_cam = depth_mm

        # --- table-plane rejection, guard 1: raw depth ------------------------
        # The desk sits ~610 mm under the camera. Anything at or past
        # table_depth_mm is the desk, not a thing on it. Rejecting here rather
        # than downstream means a table-coloured smear never becomes a target.
        if depth_mm >= self.table_depth:
            self.get_logger().debug(
                f"rejected: depth {depth_mm:.0f} mm is at or below the table "
                f"plane ({self.table_depth:.0f} mm)")
            return

        # --- table-plane rejection, guard 2: reconstructed height -------------
        # Independent of the first: this one catches a point that passed the
        # depth cutoff but still lands at table level once the extrinsics are
        # applied, which is what you get when the camera height is slightly off.
        z_base = None
        if self.calibrated:
            z_base = float((self.R_cam_base @ np.array([x_cam, y_cam, z_cam]))[2]
                           + self.T_cam_base[2])
            if z_base <= self.min_height:
                self.get_logger().debug(
                    f"rejected: reconstructs to z={z_base:.1f} mm, at or under "
                    f"the {self.min_height:.0f} mm minimum object height")
                return

        with self.lock:
            self.history.append((self.get_clock().now().nanoseconds * 1e-9,
                                 label,
                                 Measurement(x_cam, y_cam, z_cam, det.u, det.v,
                                             depth_mm, det.confidence, n_px)))

    def _publish_debug(self, bgr, det: Detection, label: str):
        img = bgr.copy()
        if det.ok:
            x, y, w, h = det.bbox
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(img, (det.u, det.v), 5, (0, 0, 255), -1)
            cv2.putText(img, f"{label} {det.confidence:.2f}", (x, max(0, y - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        else:
            cv2.putText(img, det.reason[:60], (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        if not self.calibrated:
            cv2.putText(img, "EXTRINSICS NOT CALIBRATED", (10, img.shape[0] - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
        try:
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(img, encoding="bgr8"))
        except Exception:
            pass

    # --------------------------------------------------------------- service
    def _on_detect_request(self, req: DetectTarget.Request,
                           res: DetectTarget.Response) -> DetectTarget.Response:
        label = (req.label or "").strip().lower()
        if not label:
            res.success = False
            res.message = "empty label"
            return res

        with self.lock:
            switching = label != self.current_label
            self.current_label = label
            if switching:
                self.history.clear()

        # Let the pipeline fill up with frames for the newly requested label.
        deadline = self.get_clock().now().nanoseconds * 1e-9 + self.sample_window
        while True:
            now = self.get_clock().now().nanoseconds * 1e-9
            with self.lock:
                fresh = [m for (t, lab, m) in self.history
                         if lab == label and now - t <= self.sample_window]
            if len(fresh) >= self.samples_required or now >= deadline:
                break
            time.sleep(0.02)

        if len(fresh) < self.samples_required:
            res.success = False
            res.message = (f"only {len(fresh)} good frames of '{label}' in "
                           f"{self.sample_window:.1f}s, need {self.samples_required}. "
                           f"Check /vla/debug_image. If the blob is visible there, "
                           f"the depth is being rejected: the object may be under "
                           f"{self.min_height:.0f} mm tall, or past the "
                           f"{self.table_depth:.0f} mm table cutoff.")
            return res

        xs = np.array([m.x_cam for m in fresh])
        ys = np.array([m.y_cam for m in fresh])
        zs = np.array([m.z_cam for m in fresh])
        p_cam = np.array([float(np.median(xs)), float(np.median(ys)), float(np.median(zs))])

        # Reject a target that will not hold still. Better to say so than to
        # average a moving object into a confident-looking wrong answer.
        spread = float(max(np.ptp(xs), np.ptp(ys), np.ptp(zs)))
        if spread > self.agreement_mm:
            res.success = False
            res.message = (f"detections disagree by {spread:.0f} mm across {len(fresh)} "
                           f"frames (tolerance {self.agreement_mm:.0f} mm). "
                           f"Object moving, or depth is unstable on this surface.")
            return res

        last = fresh[-1]
        stamp = self.get_clock().now().to_msg()
        cam_frame = self.depth_info.header.frame_id if self.depth_info else "camera"

        res.point_camera.header.stamp = stamp
        res.point_camera.header.frame_id = cam_frame
        res.point_camera.point.x = float(p_cam[0])
        res.point_camera.point.y = float(p_cam[1])
        res.point_camera.point.z = float(p_cam[2])

        res.pixel_u = int(last.u)
        res.pixel_v = int(last.v)
        res.depth_mm = float(p_cam[2])
        res.depth_samples = int(last.samples)
        res.confidence = float(np.mean([m.confidence for m in fresh]))

        if not self.calibrated:
            res.success = False
            res.message = (f"found '{label}' at camera-frame "
                           f"({p_cam[0]:.1f}, {p_cam[1]:.1f}, {p_cam[2]:.1f}) mm, but "
                           f"extrinsics are not calibrated so there is no base-frame "
                           f"answer. Run calibrate_extrinsics.")
            return res

        p_base = self.R_cam_base @ p_cam + self.T_cam_base
        res.point_base.header.stamp = stamp
        res.point_base.header.frame_id = "arm_base"
        res.point_base.point.x = float(p_base[0])
        res.point_base.point.y = float(p_base[1])
        res.point_base.point.z = float(p_base[2])

        self.point_pub.publish(res.point_base)
        res.success = True
        res.message = (f"{label} at base ({p_base[0]:.1f}, {p_base[1]:.1f}, "
                       f"{p_base[2]:.1f}) mm from {len(fresh)} frames, "
                       f"spread {spread:.1f} mm")
        self.get_logger().info(res.message)
        return res


def main(args=None):
    rclpy.init(args=args)
    node = VisionSpatialNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
