#!/usr/bin/env python3
"""Beambot MCP tools for robot state, poses, camera data, TF and task generation.

A background ROS executor maintains subscriptions and TF for async MCP tools.
Persistent Zivid subscriptions are established before single-shot captures.
"""

import asyncio
import collections
import json
import logging
import math
import os
import re
import sys
import threading
import time
from pathlib import Path
from typing import Any

import cv2
import yaml
import numpy as np
from mcp.server.fastmcp import FastMCP

from beambot.vision.perception import (
    get_3d_position,
    YoloDetectionParams,
    get_yolo_model,
)
from beambot.vision.perception.image_detection import detect_aruco_markers

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
)
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import Image, JointState, PointCloud2
from std_msgs.msg import String
from std_srvs.srv import Trigger
from action_msgs.srv import CancelGoal
from tf2_ros import Buffer, TransformListener, TransformException
from tf_transformations import quaternion_matrix

from cv_bridge import CvBridge

# Vacuum status is available only when epick_msgs is installed.
try:
    from epick_msgs.msg import ObjectDetectionStatus
    _EPICK_MSGS_AVAILABLE = True
except ImportError:
    ObjectDetectionStatus = None
    _EPICK_MSGS_AVAILABLE = False

from beambot.motion.task_builder import wait_for_future
from beambot.utils.task_parser import expand_grid_target

logger = logging.getLogger("beambot-mcp")

# Camera and robot interfaces.
# Zivid single-shot capture.
ZIVID_IMAGE_TOPIC = "/color/image_color"
ZIVID_CLOUD_TOPIC = "/points/xyzrgba"
ZIVID_CAPTURE_SERVICE = "/capture"
ZIVID_FRAME = "zivid_optical_frame"

# ZED continuous streaming.
ZED_IMAGE_TOPIC = "/zed/zed_node/rgb/color/rect/image"
ZED_CLOUD_TOPIC = "/zed/zed_node/point_cloud/cloud_registered"
ZED_FRAME = "zed_left_camera_optical_frame"

ZIVID_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

ZED_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

# Robot state topics
JOINT_STATES_TOPIC = "/joint_states"
CURRENT_GRIPPER_TOPIC = "/beambot/current_gripper"
EXECUTION_STATE_TOPIC = "/beambot/execution_state"
EPICK_STATUS_TOPIC = "/object_detection_status"
BEAMBOT_EXECUTION_ACTION = "/beambot_execution"

# ePick ObjectDetectionStatus labels.
EPICK_STATUS_NAMES = {
    0: "UNKNOWN",
    1: "OBJECT_DETECTED_AT_MIN_PRESSURE",
    2: "OBJECT_DETECTED_AT_MAX_PRESSURE",
    3: "NO_OBJECT_DETECTED",
}

def _poses_file_path() -> str:
    """Resolve poses_file from the active beamline config; return "" on failure."""
    try:
        from beambot.config_loader import load_beamline_config, resolve_beamline_path
        config, config_path = load_beamline_config()
        return resolve_beamline_path(config.get("poses_file", ""), config_path)
    except Exception as e:
        logger.error(f"Failed to resolve poses_file: {e}")
        return ""

DEFAULT_IMAGE_PATH = "/tmp/beambot_capture.jpg"
DEFAULT_ANNOTATED_PATH = "/tmp/beambot_detection.jpg"


def _detect_display() -> tuple[str, str]:
    """Resolve GUI display credentials when the MCP environment omits them."""
    display = os.environ.get("DISPLAY", "")
    xauth = os.environ.get("XAUTHORITY", "")

    if not display:
        # Try the desktop session when DISPLAY is unset.
        import subprocess as _sp
        try:
            result = _sp.run(
                ["bash", "-c", "cat /proc/$(pgrep -u $USER -x gnome-shell || pgrep -u $USER -x Xwayland || echo 1)/environ 2>/dev/null | tr '\\0' '\\n' | grep ^DISPLAY= | head -1 | cut -d= -f2"],
                capture_output=True, text=True, timeout=3,
            )
            display = result.stdout.strip() or ":1"
        except Exception:
            display = ":1"

    if not xauth:
        for candidate in [
            Path(f"/run/user/{os.getuid()}/gdm/Xauthority"),
            Path.home() / ".Xauthority",
        ]:
            if candidate.exists():
                xauth = str(candidate)
                break

    return display, xauth


_DETECTED_DISPLAY, _DETECTED_XAUTHORITY = _detect_display()



def _detect_hsv_color(
    rgb_image: np.ndarray,
    hue_low: int = 100,
    hue_high: int = 130,
    sat_min: int = 80,
    val_min: int = 80,
    min_area: int = 200,
) -> list[tuple[int, int, int]] | None:
    """Return HSV detections as (cx, cy, area), largest first, or None."""
    hsv = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2HSV)
    lower = np.array([hue_low, sat_min, val_min])
    upper = np.array([hue_high, 255, 255])
    mask = cv2.inRange(hsv, lower, upper)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    result = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < min_area:
            continue
        M = cv2.moments(contour)
        if M["m00"] == 0:
            continue
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        result.append((cx, cy, int(area)))
    if not result:
        return None
    result.sort(key=lambda x: x[2], reverse=True)
    return result


def _detect_aruco_markers(
    rgb_image: np.ndarray,
    marker_ids: list[int] | None = None,
    dictionary_name: str = "DICT_4X4_50",
) -> list[tuple[int, int, int, list]] | None:
    """Return (cx, cy, marker_id, corners) detections from an RGB image, or None."""
    aruco_dicts = {
        "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
        "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
        "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
        "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
        "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
        "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    }
    dict_id = aruco_dicts.get(dictionary_name, cv2.aruco.DICT_4X4_50)
    corners_list, ids = detect_aruco_markers(rgb_image, dict_id)

    if ids is None or len(ids) == 0:
        return None

    result = []
    for i, mid in enumerate(ids.flatten()):
        if marker_ids is not None and int(mid) not in marker_ids:
            continue
        corners = corners_list[i][0]
        cx = int(np.mean(corners[:, 0]))
        cy = int(np.mean(corners[:, 1]))
        result.append((cx, cy, int(mid), corners.tolist()))

    return result if result else None


def _annotate_image(
    rgb_image: np.ndarray,
    detections: list[dict[str, Any]],
) -> np.ndarray:
    """Return a BGR image annotated with detection labels and positions."""
    annotated = cv2.cvtColor(rgb_image.copy(), cv2.COLOR_RGB2BGR)
    for i, det in enumerate(detections):
        px, py = det["pixel_x"], det["pixel_y"]
        label_parts = [f"#{i + 1}"]
        if "marker_id" in det:
            label_parts = [f"ID:{det['marker_id']}"]
        if det.get("base_xyz"):
            bx, by, bz = det["base_xyz"]
            label_parts.append(f"({bx:.3f},{by:.3f},{bz:.3f})")
        elif det.get("camera_xyz"):
            cx, cy, cz = det["camera_xyz"]
            label_parts.append(f"cam({cx:.3f},{cy:.3f},{cz:.3f})")
        label = " ".join(label_parts)

        color = (0, 255, 0)
        cv2.circle(annotated, (px, py), 8, color, -1)
        cv2.circle(annotated, (px, py), 3, (0, 0, 255), -1)  # Red center dot

        cv2.putText(
            annotated, label, (px - 10, py - 15),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2,
        )
        cv2.putText(
            annotated, label, (px - 10, py - 15),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1,
        )
    return annotated


# Background ROS bridge.

class ROS2BridgeNode(Node):
    """Maintain camera data, robot state, service clients and TF."""

    def __init__(self):
        super().__init__("beambot_mcp_bridge")
        self._cb_group = ReentrantCallbackGroup()
        self._bridge = CvBridge()

        # Subscribe before triggering single-shot captures.
        self._image_sub = self.create_subscription(
            Image, ZIVID_IMAGE_TOPIC, self._on_zivid_image, ZIVID_QOS,
            callback_group=self._cb_group,
        )
        self._cloud_sub = self.create_subscription(
            PointCloud2, ZIVID_CLOUD_TOPIC, self._on_zivid_cloud, ZIVID_QOS,
            callback_group=self._cb_group,
        )

        self._capture_client = self.create_client(
            Trigger, ZIVID_CAPTURE_SERVICE, callback_group=self._cb_group,
        )

        # Latest ZED stream data.
        self._zed_image_sub = self.create_subscription(
            Image, ZED_IMAGE_TOPIC, self._on_zed_image, ZED_QOS,
            callback_group=self._cb_group,
        )
        self._zed_cloud_sub = self.create_subscription(
            PointCloud2, ZED_CLOUD_TOPIC, self._on_zed_cloud, ZED_QOS,
            callback_group=self._cb_group,
        )

        # TF updates are processed by the background executor.
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # An empty CancelGoal request cancels all orchestrator goals.
        self._cancel_client = self.create_client(
            CancelGoal, f"{BEAMBOT_EXECUTION_ACTION}/_action/cancel_goal",
            callback_group=self._cb_group,
        )

        # Robot state subscriptions
        latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self._joint_states_sub = self.create_subscription(
            JointState, JOINT_STATES_TOPIC, self._on_joint_states, 10,
            callback_group=self._cb_group,
        )
        self._gripper_sub = self.create_subscription(
            String, CURRENT_GRIPPER_TOPIC, self._on_current_gripper, latched_qos,
            callback_group=self._cb_group,
        )
        self._exec_state_sub = self.create_subscription(
            String, EXECUTION_STATE_TOPIC, self._on_execution_state, 10,
            callback_group=self._cb_group,
        )

        # Optional ePick status subscription.
        if _EPICK_MSGS_AVAILABLE:
            self._epick_status_sub = self.create_subscription(
                ObjectDetectionStatus, EPICK_STATUS_TOPIC,
                self._on_epick_status, 10,
                callback_group=self._cb_group,
            )
        else:
            self._epick_status_sub = None

        # Cached robot state; None means no data received.
        self.joint_names: list[str] | None = None
        self.joint_positions: list[float] | None = None
        self.current_gripper: str | None = None
        self.execution_state: str | None = None
        self.epick_status: int | None = None

        # Zivid state
        self.last_rgb: np.ndarray | None = None
        self.last_cloud: PointCloud2 | None = None
        self.capture_stamp = None  # Pre-capture timestamp for TF accuracy

        # ZED streaming state.
        self.zed_rgb: np.ndarray | None = None
        self.zed_cloud: PointCloud2 | None = None
        self.zed_stamp = None

        # Events used by the blocking Zivid capture worker.
        self._image_event = threading.Event()
        self._cloud_event = threading.Event()
        self._waiting_for_capture = False

        self.get_logger().info("ROS2BridgeNode initialized")

    def cancel_all_goals(self) -> str:
        """Request cancellation of all orchestrator goals and report acceptance."""
        if not self._cancel_client.wait_for_service(timeout_sec=2.0):
            return "Cancel service not available (orchestrator not running?)"
        request = CancelGoal.Request()
        future = self._cancel_client.call_async(request)
        if not wait_for_future(future, timeout=5.0, poll_interval=0.05):
            return "Cancel request timed out"
        result = future.result()
        if result is not None:
            n = len(result.goals_canceling)
            if n > 0:
                return f"Cancel accepted — {n} goal(s) being cancelled"
            return "No active goals to cancel"
        return "Cancel sent but no response received"

    def _on_zivid_image(self, msg: Image):
        try:
            self.last_rgb = self._bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        except Exception as e:
            self.get_logger().error(f"Zivid image conversion failed: {e}")
            return
        if self._waiting_for_capture:
            self._image_event.set()

    def _on_zivid_cloud(self, msg: PointCloud2):
        self.last_cloud = msg
        if self._waiting_for_capture:
            self._cloud_event.set()

    def _on_zed_image(self, msg: Image):
        self.zed_stamp = msg.header.stamp
        try:
            self.zed_rgb = self._bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        except Exception as e:
            self.get_logger().error(f"ZED image conversion failed: {e}")

    def _on_zed_cloud(self, msg: PointCloud2):
        self.zed_cloud = msg

    def _on_joint_states(self, msg: JointState):
        self.joint_names = list(msg.name)
        self.joint_positions = list(msg.position)

    def _on_current_gripper(self, msg: String):
        self.current_gripper = msg.data

    def _on_execution_state(self, msg: String):
        self.execution_state = msg.data

    def _on_epick_status(self, msg):
        self.epick_status = int(msg.status)

    def trigger_capture(self, timeout: float = 30.0, need_cloud: bool = True) -> bool:
        """Trigger Zivid capture from a worker and wait for image/cloud callbacks."""
        if not self._capture_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error(f"Capture service '{ZIVID_CAPTURE_SERVICE}' not available")
            return False

        self._image_event.clear()
        self._cloud_event.clear()
        self._waiting_for_capture = True

        # Record pre-capture timestamp for TF accuracy
        self.capture_stamp = self.get_clock().now().to_msg()

        request = Trigger.Request()
        future = self._capture_client.call_async(request)

        # Service and data waits share one monotonic deadline.
        deadline = time.monotonic() + timeout
        if not wait_for_future(future, timeout=timeout, poll_interval=0.05):
            self.get_logger().error("Capture service call timed out")
            self._waiting_for_capture = False
            return False

        result = future.result()
        if not result.success:
            self.get_logger().error(f"Capture failed: {result.message}")
            self._waiting_for_capture = False
            return False

        self.get_logger().info("Capture triggered, waiting for data...")

        remaining = max(0.1, deadline - time.monotonic())
        if not self._image_event.wait(timeout=remaining):
            self.get_logger().error("Timeout waiting for image after capture")
            self._waiting_for_capture = False
            return False

        # The point cloud can arrive later than the image.
        if need_cloud:
            remaining = max(0.1, deadline - time.monotonic())
            if not self._cloud_event.wait(timeout=remaining):
                self.get_logger().error("Timeout waiting for point cloud after capture")
                self._waiting_for_capture = False
                return False

        self._waiting_for_capture = False
        self.get_logger().info("Capture complete — image and cloud received")
        return True

    def lookup_transform(
        self, target_frame: str, source_frame: str, stamp=None, timeout_sec: float = 2.0,
    ):
        """Return a timestamped or latest TransformStamped, or None on failure."""
        try:
            if stamp is not None:
                lookup_time = rclpy.time.Time.from_msg(stamp)
                self.get_logger().debug(
                    f"TF lookup {source_frame} → {target_frame} at "
                    f"{stamp.sec}.{stamp.nanosec:09d}"
                )
            else:
                lookup_time = rclpy.time.Time()
                self.get_logger().debug(
                    f"TF lookup {source_frame} → {target_frame} (latest)"
                )

            if not self._tf_buffer.can_transform(
                target_frame, source_frame, lookup_time,
                timeout=rclpy.duration.Duration(seconds=timeout_sec),
            ):
                self.get_logger().warning(
                    f"can_transform returned False: {source_frame} → {target_frame}"
                )
                return None

            return self._tf_buffer.lookup_transform(
                target_frame, source_frame, lookup_time,
                timeout=rclpy.duration.Duration(seconds=timeout_sec),
            )
        except TransformException as e:
            self.get_logger().error(f"TF lookup failed: {e}")
            return None


class ROS2Bridge:
    """Start the bridge node and background executor on first use."""

    def __init__(self):
        self._node: ROS2BridgeNode | None = None
        self._executor: MultiThreadedExecutor | None = None
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self._initialized = False

    def _ensure_initialized(self):
        if self._initialized:
            return
        with self._lock:
            if self._initialized:
                return
            logger.info("Initializing ROS2 bridge...")
            if not rclpy.ok():
                rclpy.init()
            self._node = ROS2BridgeNode()
            self._executor = MultiThreadedExecutor(num_threads=4)
            self._executor.add_node(self._node)
            self._thread = threading.Thread(
                target=self._spin, daemon=True, name="ros2-bridge",
            )
            self._thread.start()
            self._initialized = True
            logger.info("ROS2 bridge running in background thread")

    def _spin(self):
        try:
            self._executor.spin()
        except Exception as e:
            logger.error(f"ROS2 executor error: {e}")

    @property
    def node(self) -> ROS2BridgeNode:
        self._ensure_initialized()
        return self._node

    def shutdown(self):
        if self._executor:
            self._executor.shutdown()
        if self._node:
            self._node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


# Camera state.

def _resolve_camera_state(
    node: ROS2BridgeNode,
    camera: str,
) -> tuple[np.ndarray | None, PointCloud2 | None, Any | None, str]:
    """Resolve camera name to (rgb, cloud, stamp, frame) from node state."""
    if camera == "zivid":
        return node.last_rgb, node.last_cloud, node.capture_stamp, ZIVID_FRAME
    elif camera == "zed":
        return node.zed_rgb, node.zed_cloud, node.zed_stamp, ZED_FRAME
    else:
        raise ValueError(f"Unknown camera '{camera}'. Use 'zivid' or 'zed'.")


# MCP tools.

mcp = FastMCP("beambot")

# The bridge node is initialized on first access.
bridge = ROS2Bridge()


@mcp.tool()
async def ping() -> str:
    """Return pong if the MCP server is reachable."""
    return "pong"


@mcp.tool()
async def stop_robot() -> str:
    """Request cancellation of all /beambot_execution goals; no goal ID needed.

    Stops after the active task or batch, not mid-motion. This is not an emergency stop.
    """
    node = bridge.node
    result = await asyncio.get_event_loop().run_in_executor(
        None, node.cancel_all_goals
    )
    return json.dumps({"result": result})


@mcp.tool()
async def get_robot_state() -> str:
    """Return beamline identity and cached gripper, execution state and joints_deg.

    Joint values are degrees keyed by name; missing data is null or "unknown".
    Check the gripper before choosing start_gripper. system_running only means
    joint data was received, not that MoveIt or action servers are currently ready.
    Includes cached vacuum status when ePick is selected.
    """
    node = bridge.node

    system_running = node.joint_positions is not None
    gripper = node.current_gripper or "unknown"
    exec_state = node.execution_state

    joints_deg = None
    if node.joint_positions is not None and node.joint_names is not None:
        joints_deg = {
            name: round(math.degrees(pos), 2)
            for name, pos in zip(node.joint_names, node.joint_positions)
        }

    vacuum_status = None
    if gripper == "epick" and node.epick_status is not None:
        status_int = node.epick_status
        vacuum_status = {
            "status": EPICK_STATUS_NAMES.get(status_int, f"UNKNOWN({status_int})"),
            "object_detected": status_int in (1, 2),
        }

    beamline_name = None
    try:
        from beambot.config_loader import load_beamline_config
        config, _ = load_beamline_config()
        beamline_name = config.get("beamline")
    except Exception:
        pass

    result = {
        "beamline": beamline_name,
        "system_running": system_running,
        "gripper": gripper,
        "execution_state": exec_state,
        "joints_deg": joints_deg,
    }
    if vacuum_status is not None:
        result["vacuum_status"] = vacuum_status
    return json.dumps(result, indent=2)


@mcp.tool()
async def get_vacuum_status() -> str:
    """Return cached ePick status, object_detected and availability.

    MIN/MAX_PRESSURE indicate detection; NO_OBJECT_DETECTED means no seal.
    UNKNOWN means detection is not established. available means data was received,
    not that it is fresh. Verify current status after picking; do not transport
    when object_detected is false or current grasp status is unconfirmed.
    """
    node = bridge.node
    gripper = node.current_gripper or "unknown"

    if node.epick_status is None:
        return json.dumps({
            "available": False,
            "gripper": gripper,
            "note": "No data received on /object_detection_status yet. "
                    "The ePick status controller may not be running "
                    "(only active when ePick gripper is loaded).",
        })

    status_int = node.epick_status
    return json.dumps({
        "available": True,
        "gripper": gripper,
        "status": EPICK_STATUS_NAMES.get(status_int, f"UNKNOWN({status_int})"),
        "object_detected": status_int in (1, 2),
    }, indent=2)


def _read_poses_file() -> dict:
    """Read saved poses; return an empty dict if the file is missing."""
    path = Path(_poses_file_path()).resolve()
    if not path.exists():
        return {}
    with path.open() as f:
        data = yaml.safe_load(f) or {}
    return data


def _write_poses_file(poses: dict):
    """Atomically replace the poses YAML to avoid partial writes."""
    import tempfile

    path = Path(_poses_file_path()).resolve()
    dir_path = path.parent
    dir_path.mkdir(parents=True, exist_ok=True)
    fd, tmp_path = tempfile.mkstemp(dir=dir_path, suffix=".yaml.tmp")
    tmp_path = Path(tmp_path)
    try:
        with os.fdopen(fd, "w") as f:
            yaml.dump(poses, f, default_flow_style=None, width=200)
        tmp_path.replace(path)
    except BaseException:
        tmp_path.unlink()
        raise


@mcp.tool()
async def get_saved_poses(filter: str = "") -> str:
    """Return saved joint angles in degrees and their count.

    filter is a case-insensitive name substring. Use for discovery or inspection;
    the orchestrator resolves saved pose names without this call.
    """
    poses = _read_poses_file()

    if not poses:
        return json.dumps({
            "poses": {},
            "count": 0,
            "message": f"No poses file found at {Path(_poses_file_path()).resolve()}. "
                       "Use save_pose to create one.",
        })

    if filter:
        filter_lower = filter.lower()
        poses = {k: v for k, v in poses.items() if filter_lower in k.lower()}

    return json.dumps({"poses": poses, "count": len(poses)}, indent=2)


@mcp.tool()
async def save_pose(
    name: str,
    joints_deg: list = None,
    description: str = "",
) -> str:
    """Save or overwrite a named pose in the registry.

    joints_deg: Six joint angles in degrees, in configured arm-joint order.
        If omitted, copies cached joint positions in their received order.
    description: Accepted but currently ignored.
    """
    if joints_deg is not None:
        if len(joints_deg) != 6:
            return json.dumps({"error": f"Expected 6 joint values, got {len(joints_deg)}"})
        values = [round(float(v), 2) for v in joints_deg]
    else:
        node = bridge.node
        if node.joint_positions is None:
            return json.dumps({
                "error": "No joint data available. Robot system not running. "
                         "Provide joints_deg explicitly.",
            })
        values = [round(math.degrees(v), 2) for v in node.joint_positions]

    poses = _read_poses_file()
    overwritten = name in poses
    poses[name] = values
    _write_poses_file(poses)

    return json.dumps({
        "saved": name,
        "joints_deg": values,
        "overwritten": overwritten,
        "total_poses": len(poses),
    })


@mcp.tool()
async def delete_pose(name: str) -> str:
    """Delete the named pose from the registry."""
    poses = _read_poses_file()

    if name not in poses:
        return json.dumps({
            "error": f"Pose '{name}' not found.",
            "available": list(poses.keys()),
        })

    del poses[name]
    _write_poses_file(poses)

    return json.dumps({
        "deleted": name,
        "remaining_poses": len(poses),
    })


@mcp.tool()
async def set_cup_profile(name: str) -> str:
    """Set the orchestrator's ePick cup_profile parameter.

    name selects a profile from epick_config/config/suction_cups.yaml.
    Takes effect on the next ePick MoveIt launch, not the running model.
    """
    import subprocess as _sp

    try:
        result = _sp.run(
            ["ros2", "param", "set", "/beambot_orchestrator", "cup_profile", name],
            capture_output=True, text=True, timeout=5,
        )
        if result.returncode == 0:
            return json.dumps({
                "cup_profile": name,
                "message": f"Cup profile set to '{name}'. Takes effect on next ePick MoveIt launch.",
            })
        else:
            return json.dumps({
                "error": f"Failed to set parameter: {result.stderr.strip()}",
                "note": "Is the orchestrator running?",
            })
    except Exception as e:
        return json.dumps({"error": f"Failed to set cup profile: {e}"})


@mcp.tool()
async def capture_image(
    camera: str = "zivid",
    mode: str = "3d",
    save_path: str = DEFAULT_IMAGE_PATH,
    timeout: float = 30.0,
) -> str:
    """Save a camera image; return its path, dimensions, frame and cloud availability.

    camera: "zivid" triggers a capture; "zed" reads the latest streaming data.
    mode: For Zivid, "3d" waits for image and cloud; "2d" waits only for the image.
    save_path: Output image path.
    timeout: Capture wait budget in seconds; ZED's initial-frame wait is at most 5s.

    Use mode="3d" before requesting 3D detection or pixel positions.
    """
    node = bridge.node

    if camera == "zed":
        # Use the latest ZED stream data.
        rgb = node.zed_rgb
        cloud = node.zed_cloud
        stamp = node.zed_stamp
        frame = ZED_FRAME

        if rgb is None:
            # Allow up to five seconds for the first ZED frame.
            loop = asyncio.get_event_loop()
            def _wait_for_zed():
                deadline = time.monotonic() + min(timeout, 5.0)
                while node.zed_rgb is None and time.monotonic() < deadline:
                    time.sleep(0.1)
                return node.zed_rgb is not None
            got_frame = await loop.run_in_executor(None, _wait_for_zed)
            if not got_frame:
                return json.dumps({
                    "error": "No ZED image available. Is the ZED camera running? "
                             "Check: ros2 topic hz /zed/zed_node/rgb/color/rect/image",
                })
            rgb = node.zed_rgb
            cloud = node.zed_cloud
            stamp = node.zed_stamp

    elif camera == "zivid":
        need_cloud = mode == "3d"
        loop = asyncio.get_event_loop()
        success = await loop.run_in_executor(
            None, lambda: node.trigger_capture(timeout=timeout, need_cloud=need_cloud),
        )
        if not success:
            return json.dumps({
                "error": "Zivid capture failed. Is the camera connected and driver running? "
                         "Check: ros2 service list | grep capture",
            })
        rgb = node.last_rgb
        cloud = node.last_cloud
        stamp = node.capture_stamp
        frame = ZIVID_FRAME
    else:
        return json.dumps({"error": f"Unknown camera '{camera}'. Use 'zivid' or 'zed'."})

    if rgb is None:
        return json.dumps({"error": f"No image data received from {camera}"})

    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    Path(save_path).parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(save_path, bgr)

    h, w = rgb.shape[:2]
    result = {
        "image_path": save_path,
        "camera": camera,
        "camera_frame": frame,
        "width": w,
        "height": h,
        "has_pointcloud": cloud is not None,
        "capture_timestamp": f"{stamp.sec}.{stamp.nanosec:09d}" if stamp else None,
    }
    return json.dumps(result)


@mcp.tool()
async def detect_objects(
    method: str = "hsv_color",
    camera: str = "zivid",
    hue_low: int = 100,
    hue_high: int = 130,
    sat_min: int = 80,
    val_min: int = 80,
    min_area: int = 200,
    marker_ids: str = "",
    yolo_model: str = "yolov8n.pt",
    yolo_confidence: float = 0.25,
    yolo_classes: str = "",
    transform_to_base: bool = True,
    save_path: str = DEFAULT_ANNOTATED_PATH,
) -> str:
    """Detect objects in cached camera data and save an annotated image.

    Call capture_image with the same camera first; use mode="3d" for XYZ results.
    Returns pixel detections and available camera/base_link coordinates in metres.

    method: "hsv_color", "marker" (OpenCV ArUco), or "yolo".
    camera: "zivid" or "zed".
    hue_low/hue_high: HSV hue bounds, 0-180.
    sat_min/val_min: HSV saturation/brightness thresholds, 0-255.
    min_area: Minimum HSV contour area in pixels squared.
    marker_ids: Comma-separated ArUco IDs; empty selects all.
    yolo_model: Model name or custom weights path.
    yolo_confidence: Minimum confidence, 0-1.
    yolo_classes: Comma-separated class names or IDs; empty selects all.
    transform_to_base: Also resolve XYZ in base_link using TF.
    save_path: Output annotated image path.
    """
    node = bridge.node

    try:
        rgb, cloud, stamp, camera_frame = _resolve_camera_state(node, camera)
    except ValueError as e:
        return json.dumps({"error": str(e)})

    if rgb is None:
        return json.dumps({
            "error": f"No image data from {camera}. Call capture_image(camera='{camera}') first.",
        })

    raw_detections = None
    if method == "hsv_color":
        raw = _detect_hsv_color(
            rgb, hue_low=hue_low, hue_high=hue_high,
            sat_min=sat_min, val_min=val_min, min_area=min_area,
        )
        if raw:
            raw_detections = [{"pixel_x": cx, "pixel_y": cy, "area": a} for cx, cy, a in raw]

    elif method == "marker":
        ids_list = None
        if marker_ids.strip():
            ids_list = [int(x.strip()) for x in marker_ids.split(",")]
        raw = _detect_aruco_markers(rgb, marker_ids=ids_list)
        if raw:
            raw_detections = [
                {"pixel_x": cx, "pixel_y": cy, "marker_id": mid, "corners": corners}
                for cx, cy, mid, corners in raw
            ]

    elif method == "yolo":
        class_ids = None
        class_filter_names = []
        if yolo_classes.strip():
            parts = [c.strip() for c in yolo_classes.split(",")]
            if all(p.isdigit() for p in parts):
                class_ids = [int(p) for p in parts]
            else:
                class_filter_names = [p.lower() for p in parts]

        params = YoloDetectionParams(
            model_path=yolo_model,
            confidence=yolo_confidence,
            classes=class_ids,
        )
        yolo = get_yolo_model(yolo_model)
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        raw = yolo.detect(bgr, params)

        if class_filter_names:
            raw = [d for d in raw if d[0].lower() in class_filter_names]

        if raw:
            annotated_yolo = yolo.annotate(bgr, raw)
            Path(save_path).parent.mkdir(parents=True, exist_ok=True)
            cv2.imwrite(save_path, annotated_yolo)

            raw_detections = [
                {
                    "pixel_x": cx, "pixel_y": cy,
                    "class": cls_name, "confidence": round(conf, 3),
                    "bbox": [x1, y1, x2, y2],
                }
                for cls_name, conf, cx, cy, x1, y1, x2, y2 in raw
            ]

    else:
        return json.dumps({"error": f"Unknown method '{method}'. Use: hsv_color, marker, yolo"})

    if not raw_detections:
        return json.dumps({
            "detections": [],
            "count": 0,
            "method": method,
            "message": f"No objects detected with method '{method}'",
        })

    # Resolve pixel detections to camera and base coordinates.
    for det in raw_detections:
        det["camera_xyz"] = None
        det["base_xyz"] = None

        if cloud is not None:
            xyz = get_3d_position(cloud, det["pixel_x"], det["pixel_y"])
            if xyz is not None:
                det["camera_xyz"] = list(xyz)

                if transform_to_base:
                    base_xyz = await _transform_point_to_base(
                        node, xyz, camera_frame, stamp,
                    )
                    if base_xyz is not None:
                        det["base_xyz"] = list(base_xyz)

    # YOLO annotations are saved in the detection branch.
    if method != "yolo":
        annotated = _annotate_image(rgb, raw_detections)
        Path(save_path).parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(save_path, annotated)

    result = {
        "detections": raw_detections,
        "count": len(raw_detections),
        "method": method,
        "annotated_image_path": save_path,
        "camera": camera,
        "coordinate_frame": "base_link" if transform_to_base else camera_frame,
    }
    return json.dumps(result)


async def _transform_point_to_base(
    node: ROS2BridgeNode,
    camera_xyz: tuple[float, float, float],
    camera_frame: str,
    capture_stamp=None,
) -> tuple[float, float, float] | None:
    """Transform a camera point to base_link, falling back from capture-time to latest TF."""
    loop = asyncio.get_event_loop()

    def _do_transform():
        transform = None
        if capture_stamp is not None:
            transform = node.lookup_transform(
                "base_link", camera_frame, stamp=capture_stamp,
            )
            if transform is None:
                logger.warning(
                    "TF lookup at capture_stamp failed, falling back to latest transform"
                )

        if transform is None:
            transform = node.lookup_transform(
                "base_link", camera_frame, stamp=None,
            )
            if transform is None:
                logger.error(
                    f"TF lookup failed: {camera_frame} → base_link not available "
                    "(even latest). Is the robot driver running?"
                )
                return None


        q = transform.transform.rotation
        t = transform.transform.translation
        mat = quaternion_matrix([q.x, q.y, q.z, q.w])
        mat[0, 3] = t.x
        mat[1, 3] = t.y
        mat[2, 3] = t.z

        point_cam = np.array([camera_xyz[0], camera_xyz[1], camera_xyz[2], 1.0])
        point_base = mat @ point_cam
        return (float(point_base[0]), float(point_base[1]), float(point_base[2]))

    return await loop.run_in_executor(None, _do_transform)


async def _confirm_point_via_gui(
    image_path: str,
    pixel_x: int,
    pixel_y: int,
    timeout: float = 120.0,
) -> dict[str, Any] | None:
    """Run point confirmation; return selected pixels, None on cancel, or an error dict."""
    gui_script = Path(__file__).with_name("point_selector_gui.py")
    if not gui_script.exists():
        return {"error": f"GUI script not found: {gui_script}"}

    cmd = [
        sys.executable, str(gui_script), image_path,
        "--x", str(pixel_x), "--y", str(pixel_y),
        "--title", "Confirm Point — Click to adjust, Enter to confirm",
    ]

    # Supply detected X11 credentials when absent from the subprocess environment.
    env = os.environ.copy()
    env.setdefault("DISPLAY", _DETECTED_DISPLAY)
    env.setdefault("XAUTHORITY", _DETECTED_XAUTHORITY)

    try:
        proc = await asyncio.create_subprocess_exec(
            *cmd,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE,
            env=env,
        )
        stdout, stderr = await asyncio.wait_for(
            proc.communicate(), timeout=timeout,
        )
    except asyncio.TimeoutError:
        proc.kill()
        return {"error": f"GUI timed out after {timeout}s"}
    except Exception as e:
        return {"error": f"Failed to launch GUI: {e}"}

    stderr_text = stderr.decode().strip() if stderr else ""
    if stderr_text:
        logger.info(f"GUI stderr: {stderr_text}")

    if proc.returncode != 0:
        # The GUI may return error JSON even when it exits unsuccessfully.
        try:
            result = json.loads(stdout.decode().strip())
            if stderr_text:
                result["stderr"] = stderr_text
            return result
        except (json.JSONDecodeError, ValueError):
            return {"error": f"GUI exited with code {proc.returncode}: {stderr_text}"}

    try:
        result = json.loads(stdout.decode().strip())
    except (json.JSONDecodeError, ValueError) as e:
        return {"error": f"Failed to parse GUI output: {e}. stderr: {stderr_text}"}

    if not result.get("confirmed", False):
        return None

    return result



# Sample model loaded on first use.
_sample_yolo_model = None


def _get_sample_yolo_model():
    """Load and cache the sample detector weights."""
    global _sample_yolo_model
    if _sample_yolo_model is None:
        from ultralytics import YOLO
        model_path = Path(__file__).parents[1] / "models" / "sample_detector.pt"
        if not model_path.exists():
            raise FileNotFoundError(
                f"YOLO model not found at {model_path}. "
                "Provide a trained model at that path."
            )
        _sample_yolo_model = YOLO(str(model_path))
        logger.info(f"YOLO model loaded from {model_path}")
    return _sample_yolo_model


@mcp.tool()
async def detect_sample_yolo(
    camera: str = "zivid",
    confidence: float = 0.5,
    save_path: str = "/tmp/sample_detection_yolo.jpg",
) -> str:
    """Return the highest-confidence sample detection using sample_detector.pt.

    First call capture_image(camera=..., mode="3d") with the same camera.
    camera is "zivid" or "zed"; confidence is the minimum score, 0-1.
    Saves annotations to save_path. Returns the pixel center, bounding box,
    confidence and pickup_base_xyz in metres, or null if depth/TF is unavailable.
    """
    node = bridge.node

    try:
        rgb, cloud, stamp, camera_frame = _resolve_camera_state(node, camera)
    except ValueError as e:
        return json.dumps({"error": str(e)})

    if rgb is None or cloud is None:
        return json.dumps({
            "error": f"No image/point cloud from {camera}. "
                     f"Call capture_image(camera='{camera}', mode='3d') first."
        })

    try:
        model = _get_sample_yolo_model()
    except FileNotFoundError as e:
        return json.dumps({"error": str(e)})

    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    results = model(bgr, conf=confidence, verbose=False)

    detections = []
    for r in results:
        if r.boxes is None or len(r.boxes) == 0:
            continue
        for box in r.boxes:
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            conf = box.conf[0].item()
            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)
            detections.append({
                "cx": cx, "cy": cy, "conf": conf,
                "x1": int(x1), "y1": int(y1),
                "x2": int(x2), "y2": int(y2),
            })

    if not detections:
        # Preserve the annotated frame when no sample is detected.
        if results:
            annotated = results[0].plot()
            cv2.imwrite(save_path, annotated)
        return json.dumps({
            "error": "No sample detected by YOLO model. "
                     "Ensure a sample is visible in the camera view.",
            "annotated_image_path": save_path,
        })

    best = max(detections, key=lambda d: d["conf"])
    cx, cy = best["cx"], best["cy"]

    pickup_xyz = get_3d_position(cloud, cx, cy, search_radius=20)
    pickup_base = None
    if pickup_xyz is not None:
        pickup_base = await _transform_point_to_base(node, pickup_xyz, camera_frame, stamp)

    if results:
        annotated = results[0].plot()
        cv2.imwrite(save_path, annotated)

    result = {
        "pickup_base_xyz": [round(v, 6) for v in pickup_base] if pickup_base else None,
        "detection_pixel": [cx, cy],
        "confidence": round(best["conf"], 3),
        "bbox": [best["x1"], best["y1"], best["x2"], best["y2"]],
        "bbox_size_px": [best["x2"] - best["x1"], best["y2"] - best["y1"]],
        "num_detections": len(detections),
        "annotated_image_path": save_path,
    }

    return json.dumps(result)


@mcp.tool()
async def get_point_3d(
    pixel_x: int,
    pixel_y: int,
    camera: str = "zivid",
    transform_to_base: bool = True,
    search_radius: int = 10,
    confirm: bool = True,
    save_path: str = "/tmp/beambot_point3d.jpg",
) -> str:
    """Resolve an image pixel using cached camera depth and optionally confirm it in a GUI.

    First call capture_image(camera=..., mode="3d") with the same camera.
    pixel_x/pixel_y: Image column/row, in pixels.
    camera: "zivid" or "zed".
    search_radius: Pixel radius to search when the selected pixel has no depth.
    transform_to_base: Also resolve the point in base_link using TF.
    confirm: Show point selection; Enter confirms, Esc cancels.
    save_path: Output annotated image path.

    Returns camera_xyz and available base_xyz in metres, or a cancellation/error.
    """
    node = bridge.node

    try:
        rgb, cloud, stamp, camera_frame = _resolve_camera_state(node, camera)
    except ValueError as e:
        return json.dumps({"error": str(e)})

    if cloud is None:
        return json.dumps({
            "error": f"No point cloud data from {camera}. Call capture_image(camera='{camera}', mode='3d') first.",
        })

    if rgb is None:
        return json.dumps({
            "error": f"No image data from {camera}. Call capture_image(camera='{camera}', mode='3d') first.",
        })

    h, w = rgb.shape[:2]
    if pixel_x < 0 or pixel_x >= w or pixel_y < 0 or pixel_y >= h:
        return json.dumps({
            "error": f"Pixel ({pixel_x}, {pixel_y}) out of bounds. "
                     f"Image size: {w}x{h}.",
        })

    if confirm:
        # Reuse the saved capture image if present.
        image_path = DEFAULT_IMAGE_PATH
        if not Path(image_path).exists():
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            cv2.imwrite(image_path, bgr)

        gui_result = await _confirm_point_via_gui(image_path, pixel_x, pixel_y)

        if gui_result is None:
            return json.dumps({
                "cancelled": True,
                "message": "User cancelled point selection.",
            })

        if "error" in gui_result:
            return json.dumps(gui_result)

        pixel_x = gui_result["pixel_x"]
        pixel_y = gui_result["pixel_y"]

        # Validate the user-adjusted pixel against the current image.
        if pixel_x < 0 or pixel_x >= w or pixel_y < 0 or pixel_y >= h:
            return json.dumps({
                "error": f"User-selected pixel ({pixel_x}, {pixel_y}) out of bounds. "
                         f"Image size: {w}x{h}.",
            })

    xyz = get_3d_position(cloud, pixel_x, pixel_y, search_radius)

    if xyz is None:
        return json.dumps({
            "error": f"No valid depth at pixel ({pixel_x}, {pixel_y}) "
                     f"within search_radius={search_radius}. "
                     "The point may be on a reflective surface or out of range.",
        })

    result = {
        "pixel_x": pixel_x,
        "pixel_y": pixel_y,
        "camera_xyz": [round(v, 6) for v in xyz],
        "camera": camera,
        "camera_frame": camera_frame,
        "base_xyz": None,
        "base_frame": "base_link",
    }

    if transform_to_base:
        base_xyz = await _transform_point_to_base(
            node, xyz, camera_frame, stamp,
        )
        if base_xyz is not None:
            result["base_xyz"] = [round(v, 6) for v in base_xyz]

    annotated = cv2.cvtColor(rgb.copy(), cv2.COLOR_RGB2BGR)
    cv2.drawMarker(
        annotated, (pixel_x, pixel_y), (0, 0, 255),
        cv2.MARKER_CROSS, 20, 2,
    )
    label_parts = [f"({pixel_x}, {pixel_y})"]
    if result["base_xyz"]:
        bx, by, bz = result["base_xyz"]
        label_parts.append(f"base: ({bx:.3f}, {by:.3f}, {bz:.3f})")
    else:
        cx, cy, cz = result["camera_xyz"]
        label_parts.append(f"cam: ({cx:.3f}, {cy:.3f}, {cz:.3f})")
    label = " ".join(label_parts)
    cv2.putText(
        annotated, label, (pixel_x + 15, pixel_y - 10),
        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2,
    )
    cv2.putText(
        annotated, label, (pixel_x + 15, pixel_y - 10),
        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1,
    )
    Path(save_path).parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(save_path, annotated)
    result["annotated_image_path"] = save_path

    return json.dumps(result)


@mcp.tool()
async def get_tf_transform(
    source_frame: str = "flange",
    target_frame: str = "base_link",
    timeout: float = 2.0,
) -> str:
    """Return the latest transform from source_frame to target_frame.

    Uses flange by default for MoveIt Cartesian targets; tool0 has UR tool axes.
    These frames share an origin but differ in orientation. Common frames include
    base_link, flange, tool0, zivid_optical_frame and world.
    timeout: Transform wait timeout in seconds.

    Returns translation in metres, quaternion, RPY in degrees and a 4x4 matrix.
    """
    loop = asyncio.get_event_loop()

    def _lookup():
        node = bridge.node
        transform = node.lookup_transform(target_frame, source_frame, timeout_sec=timeout)
        if transform is None:
            return None
        return transform

    transform = await loop.run_in_executor(None, _lookup)

    if transform is None:
        return json.dumps({
            "error": f"Transform {source_frame} → {target_frame} not available. "
                     "Is the robot driver running?",
        })

    t = transform.transform.translation
    q = transform.transform.rotation

    from tf_transformations import euler_from_quaternion

    rpy = euler_from_quaternion([q.x, q.y, q.z, q.w])
    rpy_deg = [math.degrees(a) for a in rpy]

    mat = quaternion_matrix([q.x, q.y, q.z, q.w])
    mat[0, 3] = t.x
    mat[1, 3] = t.y
    mat[2, 3] = t.z

    result = {
        "source_frame": source_frame,
        "target_frame": target_frame,
        "translation": {"x": round(t.x, 6), "y": round(t.y, 6), "z": round(t.z, 6)},
        "rotation_quaternion": {
            "x": round(q.x, 6), "y": round(q.y, 6),
            "z": round(q.z, 6), "w": round(q.w, 6),
        },
        "rotation_rpy_degrees": {
            "roll": round(rpy_deg[0], 4),
            "pitch": round(rpy_deg[1], 4),
            "yaw": round(rpy_deg[2], 4),
        },
        "matrix_4x4": [[round(float(mat[r, c]), 6) for c in range(4)] for r in range(4)],
    }
    return json.dumps(result)


BEAMBOT_LOG_FILE = "/tmp/beambot_launch.log"

# ROS launch format: [process-N] [LEVEL] [timestamp] [logger]: message.
_LOG_LINE_RE = re.compile(
    r'^\[([^\]]+)\]\s+'          # [process_name-N]
    r'\[(\w+)\]\s+'              # [LEVEL]
    r'\[[\d.]+\]\s+'             # [timestamp]
    r'\[([^\]]+)\]:\s+'          # [logger_name]
    r'(.+)$'                     # message
)

_SEVERITY_ORDER = {"DEBUG": 10, "INFO": 20, "WARN": 30, "ERROR": 40, "FATAL": 50}


@mcp.tool()
async def get_recent_logs(
    severity: str = "ERROR",
    logger: str = "",
    count: int = 30,
) -> str:
    """Return recent matching entries from /tmp/beambot_launch.log, newest first.

    The log is written by start_mcp.sh; only its final 2,000 lines are searched.
    severity: Minimum level: DEBUG, INFO, WARN, ERROR or FATAL.
    logger: Logger-name prefix; empty matches all.
    count: Maximum entries to return.
    Entries contain process, logger, severity and message.
    """
    min_level = _SEVERITY_ORDER.get(severity.upper(), 40)

    if not Path(BEAMBOT_LOG_FILE).exists():
        return json.dumps({
            "error": f"Log file not found: {BEAMBOT_LOG_FILE}. "
                     "Make sure beambot was started with start_mcp.sh.",
            "logs": [],
            "count": 0,
        })

    # Retain at most 2,000 lines while reading the launch log.
    loop = asyncio.get_event_loop()
    def _read_tail():
        try:
            with open(BEAMBOT_LOG_FILE, 'r', errors='replace') as f:
                return collections.deque(f, maxlen=2000)
        except Exception as e:
            return str(e)

    lines = await loop.run_in_executor(None, _read_tail)
    if isinstance(lines, str):
        return json.dumps({"error": f"Failed to read log file: {lines}", "logs": [], "count": 0})

    # Return matching entries newest first.
    filtered = []
    for line in reversed(lines):
        m = _LOG_LINE_RE.match(line.strip())
        if not m:
            continue
        process, level, logger_name, message = m.groups()
        level_num = _SEVERITY_ORDER.get(level, 0)
        if level_num < min_level:
            continue
        if logger and not logger_name.startswith(logger):
            continue
        filtered.append({
            "process": process,
            "logger": logger_name,
            "severity": level,
            "message": message,
        })
        if len(filtered) >= count:
            break

    result = {
        "logs": filtered,
        "count": len(filtered),
        "filter": {
            "min_severity": severity,
            "logger_prefix": logger or "(all)",
            "max_count": count,
        },
    }
    return json.dumps(result)


# Vision-target task generation.


def _load_beamline_config() -> dict:
    """Load the full beamline config from $BEAMBOT_BEAMLINE_CONFIG."""
    try:
        from beambot.config_loader import load_beamline_config
        config, _ = load_beamline_config()
        return config
    except Exception as e:
        logger.error(f"Failed to load beamline config: {e}")
        return {}


def _load_vision_targets() -> dict:
    """Load vision target configs from $BEAMBOT_BEAMLINE_CONFIG."""
    return _load_beamline_config().get("vision_targets", {})



def _build_vision_target_tasks(
    target_name: str,
    element_index: int = 0,
    row: int = -1,
    col: int = -1,
    tag_id: int = -1,
) -> dict:
    """Build offset/grid task JSON without executing it; return an error dict on failure.

    Row and column override the row-major index; tag_id=-1 uses the configured ID.
    """
    targets = _load_vision_targets()
    if target_name not in targets:
        available = list(targets.keys()) if targets else []
        return {"error": f"Unknown target '{target_name}'. Available: {available}"}

    cfg = targets[target_name]
    mode = cfg.get("mode", "offset")
    scan_pose_name = cfg.get("scan_pose", "")
    start_gripper = cfg.get("start_gripper", "pipettor")

    poses_dict = {}
    all_poses = _read_poses_file()
    if scan_pose_name and scan_pose_name in all_poses:
        poses_dict[scan_pose_name] = all_poses[scan_pose_name]

    if mode == "offset":
        marker_id = tag_id if tag_id >= 0 else cfg["marker_id"]
        marker_offset = cfg.get("marker_offset", {})
        tasks = [{"task_type": "moveto", "target": scan_pose_name}] if scan_pose_name else []
        vision_step = {
            "task_type": "vision_moveto",
            "tag_id": marker_id,
            "marker_offset_x": float(marker_offset.get("x", 0.0)),
            "marker_offset_y": float(marker_offset.get("y", 0.0)),
            "marker_offset_z": float(marker_offset.get("z", 0.0)),
        }
        if "z_offset" in cfg:
            vision_step["z_offset"] = float(cfg["z_offset"])
        tasks.append(vision_step)

        task_json = json.dumps({
            "start_gripper": start_gripper,
            "tasks": tasks,
            "poses": poses_dict,
        })
        return {"task_json": task_json, "target": target_name, "mode": "offset"}

    elif mode == "grid":
        # Share the orchestrator's pickup-macro expansion and validation.
        task = {"row": row, "col": col} if row >= 0 and col >= 0 else {
            "element_index": element_index
        }
        if tag_id >= 0:
            task["config"] = {"marker_id": tag_id}
        try:
            grid_tasks, element = expand_grid_target(target_name, targets, task, target_name)
        except ValueError as error:
            return {"error": str(error)}

        task_json = json.dumps({
            "start_gripper": start_gripper,
            "tasks": grid_tasks,  # Includes the scan-pose move.
            "poses": poses_dict,
        })
        (col_dir, col_dist), (row_dir, row_dist) = (
            element["moves"]["col"], element["moves"]["row"]
        )
        return {
            "task_json": task_json,
            "target": target_name,
            "mode": "grid",
            "element": {k: element[k] for k in ("row", "col", "index")},
            "offsets_mm": {
                col_dir: round(col_dist * 1000, 1),
                row_dir: round(row_dist * 1000, 1),
            },
        }

    else:
        return {"error": f"Unknown mode '{mode}' for target '{target_name}'"}


@mcp.tool()
async def vision_target(
    target_name: str,
    element_index: int = 0,
    row: int = -1,
    col: int = -1,
    tag_id: int = -1,
) -> str:
    """Build task_json for a configured vision target without executing it.

    target_name: Entry under vision_targets in BEAMBOT_BEAMLINE_CONFIG.
    Offset targets use marker-relative positioning; grid targets add relative moves.
    element_index: Zero-based row-major grid index; ignored for offset targets.
    row/col: Zero-based grid coordinates; supply both to override element_index.
    tag_id: Override the marker ID; -1 uses the configured value.

    Send the returned task_json to the orchestrator via send_action_goal.
    """
    result = _build_vision_target_tasks(target_name, element_index, row, col, tag_id=tag_id)
    return json.dumps(result, indent=2)


@mcp.tool()
async def pickup_tip(
    tip_index: int = 0,
    row: int = -1,
    col: int = -1,
) -> str:
    """Build tip-rack pickup task_json without executing it.

    tip_index is zero-based and row-major; supplying both row and col overrides it.
    Bounds follow the configured tip_rack grid. Send task_json to the orchestrator.
    """
    result = _build_vision_target_tasks("tip_rack", tip_index, row, col)
    return json.dumps(result, indent=2)


# Entry point.

def main():
    # Reserve stdout for MCP stdio transport.
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
        stream=sys.stderr,
    )
    logger.info("Starting EROBS MCP server...")

    # Keep crash details when the MCP client hides stderr.
    crash_log = "/tmp/beambot_mcp_crash.log"

    try:
        mcp.run(transport="stdio")
    except Exception:
        import traceback
        tb = traceback.format_exc()
        logger.error(f"MCP server crashed:\n{tb}")
        with open(crash_log, "a") as f:
            f.write(f"\n{'='*60}\n{time.strftime('%Y-%m-%d %H:%M:%S')}\n{tb}\n")
    finally:
        bridge.shutdown()


if __name__ == "__main__":
    main()
