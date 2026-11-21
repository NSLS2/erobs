"""Shared vision detection, transforms, scan caching and motion helpers."""

import math
import time
from dataclasses import dataclass

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from moveit.task_constructor import stages
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive
from tf2_geometry_msgs import do_transform_pose_stamped
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, TransformException
from tf_transformations import (
    quaternion_multiply,
    quaternion_from_euler,
    quaternion_matrix,
    euler_from_quaternion,
)

from beambot.vision.camera import DetectionResult, get_camera
from beambot.motion.task_builder import (
    MtcTaskBuilder,
    DEFAULT_JOINT_NAMES,
    DIRECTION_VECTORS,
    apply_constraints,
    wait_for_future,
)


@dataclass
class ObjectInfo:
    """Collision-object geometry and marker-local offset."""

    name: str
    shape: str
    dimensions: list
    tag_offset: list


@dataclass
class GripperDetection:
    """Detected gripper tip frame and its vertical offset."""

    ik_frame: str
    z_offset: float


class TargetLocalizer(MtcTaskBuilder):
    """Coordinate camera detection, TF, approach motion and collision objects."""

    DEFAULT_RETRY_COUNT = 3  # Retries after the first attempt.
    DEFAULT_RETRY_DELAY = 0.5  # Seconds between attempts.

    # Seconds to let robot vibration settle before capture.
    DEFAULT_SETTLE_TIME = 0.3

    # Initialization and configuration

    def __init__(
        self,
        rclpy_node,
        arm_group: str = "",
        ik_frame: str = "",
        camera_type: str = None,
        camera_frame: str = None,
        marker_dictionary: str = None,
        retry_count: int = None,
        retry_delay: float = None,
        settle_time: float = None,
    ):
        """Initialize vision resources; camera settings must be supplied by the caller."""
        super().__init__(rclpy_node, arm_group, ik_frame=ik_frame)

        missing = [
            name
            for name, val in (
                ("camera_type", camera_type),
                ("camera_frame", camera_frame),
                ("marker_dictionary", marker_dictionary),
            )
            if not val
        ]
        if missing:
            raise ValueError(
                f"TargetLocalizer requires {', '.join(missing)} from the active "
                f"beamline YAML's `camera:` block (set BEAMBOT_BEAMLINE_CONFIG)."
            )
        self._camera_type = camera_type
        self._camera_frame = camera_frame
        self._marker_dictionary = marker_dictionary

        # Detection retries and capture settling.
        self._retry_count = (
            retry_count if retry_count is not None else self.DEFAULT_RETRY_COUNT
        )
        self._retry_delay = (
            retry_delay if retry_delay is not None else self.DEFAULT_RETRY_DELAY
        )

        self._settle_time = (
            settle_time if settle_time is not None else self.DEFAULT_SETTLE_TIME
        )

        # Transforms and service clients.
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, rclpy_node)
        self._tf_broadcaster = TransformBroadcaster(rclpy_node)

        self._camera = get_camera(self._camera_type)
        self._capture_client = self._camera.create_client(rclpy_node)

        self._apply_scene_client = rclpy_node.create_client(
            ApplyPlanningScene, "/apply_planning_scene"
        )

        self._object_database: dict[int, ObjectInfo] = {}

        self._load_vision_objects_config()

        # Averaged scan poses, local to this engine instance.
        self._tag_pose_cache: dict[int, PoseStamped] = {}

        self.logger.info(
            f"TargetLocalizer initialized (camera: {self._camera_type}, "
            f"frame: {self._camera_frame}, ik_frame: "
            f"{'auto-detect' if not ik_frame else ik_frame}, "
            f"settle_time: {self._settle_time:.2f}s)"
        )

    def _load_vision_objects_config(self):
        """Load tag-keyed collision objects, skipping malformed entries."""
        try:
            from beambot.config_loader import load_beamline_config

            config, _ = load_beamline_config()
        except Exception as e:
            self.logger.error(
                f"Failed to load beamline YAML for collision_objects: {e}"
            )
            return

        objects = config.get("collision_objects") or {}
        for tag_id, obj in objects.items():
            try:
                info = ObjectInfo(
                    name=obj["name"],
                    shape=obj["shape"],
                    dimensions=obj["dimensions"],
                    tag_offset=obj["tag_offset"],
                )
                self._object_database[int(tag_id)] = info
            except (KeyError, TypeError, ValueError) as e:
                self.logger.error(
                    f"collision_objects[{tag_id}]: skipped — {e}. "
                    f"Required keys: name, shape, dimensions, tag_offset."
                )

        self.logger.info(f"Loaded {len(self._object_database)} vision objects")

    # Scanning and pose caching

    def get_cached_pose(self, tag_id: int) -> PoseStamped | None:
        """Return this engine's cached tag pose, or None."""
        return self._tag_pose_cache.get(tag_id)

    def scan_all_tags(
        self,
        scan_positions: list[list[float]],
        scans_per_position: int = 3,
        timeout: float = 10.0,
        settle_time: float = 0.3,
    ) -> int:
        """Scan joint poses (radians) and return the number of tags cached."""
        self._tag_pose_cache.clear()

        all_detections: dict[int, list[PoseStamped]] = {}

        total_scans = len(scan_positions) * scans_per_position
        self.logger.info(
            f"Starting batch scan: {len(scan_positions)} positions × "
            f"{scans_per_position} scans = {total_scans} total captures"
        )

        for pos_idx, joint_pose in enumerate(scan_positions):
            self.logger.info(f"Position {pos_idx + 1}/{len(scan_positions)}")

            if not self._move_to_joint_pose(joint_pose):
                self.logger.warning(f"Failed to reach position {pos_idx + 1}, skipping")
                continue

            if settle_time > 0:
                time.sleep(settle_time)

            for scan_idx in range(scans_per_position):
                self.logger.info(f"  Scan {scan_idx + 1}/{scans_per_position}")

                result = self._camera.detect_markers(
                    self._capture_client,
                    self.rclpy_node,
                    marker_ids=None,  # Capture all visible markers.
                    dictionary=self._marker_dictionary,
                    timeout=timeout,
                    settle_time=0,  # Already settled after the move.
                )

                if not result.markers:
                    self.logger.debug("    No markers in this scan")
                    continue

                for marker_id, marker_pose in result.markers:
                    pose_base = self._transform_to_base_link(
                        marker_pose, capture_stamp=result.capture_stamp
                    )
                    if pose_base:
                        all_detections.setdefault(marker_id, []).append(pose_base)

                self.logger.info(f"    Detected {len(result.markers)} markers")

        self.logger.info(f"Processing {len(all_detections)} unique tags...")
        for tag_id, poses in sorted(all_detections.items()):
            if len(poses) >= 2:
                averaged = self._average_poses(poses)
                if averaged:
                    self._tag_pose_cache[tag_id] = averaged
                    pos = averaged.pose.position
                    self.logger.info(
                        f"  Tag {tag_id}: [{pos.x * 1000:.2f}, {pos.y * 1000:.2f}, "
                        f"{pos.z * 1000:.2f}] mm (from {len(poses)} detections)"
                    )
            else:
                self.logger.warning(
                    f"  Tag {tag_id}: only {len(poses)} detection(s), need ≥2 for averaging"
                )

        self.logger.info(
            f"Batch scan complete: {len(self._tag_pose_cache)} tags cached"
        )
        return len(self._tag_pose_cache)

    def detect_tag_multiposition(
        self,
        tag_id: int,
        scan_positions: list[list[float]],
        timeout: float = 45.0,
        settle_time: float = 0.3,
    ) -> PoseStamped | None:
        """Average tag poses from at least two successful scans; joints are radians."""
        detected_poses = []

        self.logger.info(
            f"Multi-position detection: tag {tag_id} from {len(scan_positions)} positions"
        )

        for i, joint_pose in enumerate(scan_positions):
            position_name = f"position {i + 1}/{len(scan_positions)}"

            self.logger.info(f"Moving to {position_name}...")
            if not self._move_to_joint_pose(joint_pose):
                self.logger.warning(f"Failed to reach {position_name}, skipping")
                continue

            if settle_time > 0:
                self.logger.debug(f"Settling for {settle_time:.2f}s...")
                time.sleep(settle_time)

            pose = self.detect_and_transform_tag(tag_id, timeout)

            if pose is not None:
                detected_poses.append(pose)
                pos = pose.pose.position
                self.logger.info(
                    f"  {position_name}: detected at "
                    f"[{pos.x * 1000:.2f}, {pos.y * 1000:.2f}, {pos.z * 1000:.2f}] mm"
                )
            else:
                self.logger.warning(f"  {position_name}: detection failed")

        success_count = len(detected_poses)
        self.logger.info(
            f"Multi-position results: {success_count}/{len(scan_positions)} successful"
        )

        if success_count < 2:
            self.logger.error(
                f"Need ≥2 successful detections for averaging, got {success_count}"
            )
            return None

        averaged_pose = self._average_poses(detected_poses)

        if averaged_pose is not None:
            pos = averaged_pose.pose.position
            self.logger.info(
                f"Averaged pose: [{pos.x * 1000:.2f}, {pos.y * 1000:.2f}, {pos.z * 1000:.2f}] mm"
            )

            x_vals = [p.pose.position.x for p in detected_poses]
            y_vals = [p.pose.position.y for p in detected_poses]
            z_vals = [p.pose.position.z for p in detected_poses]
            self.logger.info(
                f"Spread (σ): X={np.std(x_vals) * 1000:.3f}mm, "
                f"Y={np.std(y_vals) * 1000:.3f}mm, Z={np.std(z_vals) * 1000:.3f}mm"
            )

        return averaged_pose

    def _move_to_joint_pose(
        self,
        joint_positions: list[float],
    ) -> bool:
        """Execute a PTP scan move using six joint angles in radians; return success."""
        if len(joint_positions) != 6:
            self.logger.error(
                f"Invalid joint positions: expected 6, got {len(joint_positions)}"
            )
            return False

        self.logger.debug(
            f"Moving to joint pose: [{', '.join(f'{j:.3f}' for j in joint_positions)}]"
        )

        task = self.create_task_template("Scan Position Move")
        planner = self.make_pilz_planner("PTP")

        stage = stages.MoveTo("move to scan position", planner)
        stage.group = self.arm_group
        self._set_ik_frame(stage)
        joint_dict = dict(zip(DEFAULT_JOINT_NAMES, joint_positions))
        stage.setGoal(joint_dict)
        task.add(stage)

        error = self.load_plan_execute(task)
        if error:
            self.logger.error(f"Scan position move failed: {error}")
            return False
        return True

    def _average_poses(self, poses: list[PoseStamped]) -> PoseStamped | None:
        """Average at least two poses, aligning quaternion signs before normalization."""
        if len(poses) < 2:
            self.logger.error(f"Need ≥2 poses to average, got {len(poses)}")
            return None

        avg_x = sum(p.pose.position.x for p in poses) / len(poses)
        avg_y = sum(p.pose.position.y for p in poses) / len(poses)
        avg_z = sum(p.pose.position.z for p in poses) / len(poses)

        # Align q and -q before averaging; they represent the same rotation.
        quats = []
        ref_q = np.array(
            [
                poses[0].pose.orientation.x,
                poses[0].pose.orientation.y,
                poses[0].pose.orientation.z,
                poses[0].pose.orientation.w,
            ]
        )

        for p in poses:
            q = np.array(
                [
                    p.pose.orientation.x,
                    p.pose.orientation.y,
                    p.pose.orientation.z,
                    p.pose.orientation.w,
                ]
            )
            if np.dot(ref_q, q) < 0:
                q = -q
            quats.append(q)

        avg_q = np.mean(quats, axis=0)
        avg_q = avg_q / np.linalg.norm(avg_q)

        result = PoseStamped()
        result.header = poses[0].header
        result.pose.position.x = avg_x
        result.pose.position.y = avg_y
        result.pose.position.z = avg_z
        result.pose.orientation.x = avg_q[0]
        result.pose.orientation.y = avg_q[1]
        result.pose.orientation.z = avg_q[2]
        result.pose.orientation.w = avg_q[3]

        return result

    # Marker and sample detection

    def detect_and_transform_tag(
        self, tag_id: int, timeout: float = 45.0
    ) -> PoseStamped | None:
        """Return a detected tag pose in base_link; timeout is per attempt."""
        total_attempts = 1 + self._retry_count
        last_error = "Unknown error"

        for attempt in range(total_attempts):
            if attempt > 0:
                self.logger.info(
                    f"Retry {attempt}/{self._retry_count} for tag {tag_id} "
                    f"(waiting {self._retry_delay}s...)"
                )
                time.sleep(self._retry_delay)

            result = self._single_detection_attempt(tag_id, timeout)

            if result is not None:
                pose_base = self._process_detection_result(tag_id, result)
                if pose_base is not None:
                    if attempt > 0:
                        self.logger.info(f"Tag {tag_id} detected on retry {attempt}")
                    return pose_base
                else:
                    last_error = f"Tag {tag_id} not found in detection results"
            else:
                last_error = "Detection attempt failed"

        self.logger.error(
            f"Failed to detect tag {tag_id} after {total_attempts} attempts: {last_error}"
        )
        return None

    def _single_detection_attempt(
        self, tag_id: int, timeout: float
    ) -> DetectionResult | None:
        """Capture one marker detection, returning None if no markers are found."""
        self.logger.info(f"Detecting tag {tag_id}...")

        result = self._camera.detect_markers(
            self._capture_client,
            self.rclpy_node,
            marker_ids=[tag_id],
            dictionary=self._marker_dictionary,
            timeout=timeout,
            settle_time=self._settle_time,
        )

        if not result.markers:
            self.logger.warning("No markers detected")
            return None

        return result

    def _process_detection_result(
        self, tag_id: int, detection_result: DetectionResult
    ) -> PoseStamped | None:
        """Transform a tag into base_link and update its TF and configured scene object."""
        for marker_id, marker_pose in detection_result.markers:
            if marker_id == tag_id:
                self.logger.info(
                    f"Tag {marker_id} at [{marker_pose.position.x:.3f}, "
                    f"{marker_pose.position.y:.3f}, {marker_pose.position.z:.3f}] "
                    f"in camera frame"
                )

                pose_base = self._transform_to_base_link(
                    marker_pose, capture_stamp=detection_result.capture_stamp
                )
                if pose_base is None:
                    return None

                self.logger.info(
                    f"Transformed to base_link: [{pose_base.pose.position.x:.3f}, "
                    f"{pose_base.pose.position.y:.3f}, {pose_base.pose.position.z:.3f}]"
                )
                self.logger.debug(
                    f"  BASE_LINK pose (mm): ({pose_base.pose.position.x * 1000:.2f}, "
                    f"{pose_base.pose.position.y * 1000:.2f}, {pose_base.pose.position.z * 1000:.2f})"
                )

                self._broadcast_detection_tf(f"aruco_{tag_id}", pose_base)

                self._add_collision_object_for_tag(tag_id, pose_base)

                return pose_base

        self.logger.warning(
            f"Tag {tag_id} not in results "
            f"({len(detection_result.markers)} markers detected)"
        )
        return None

    def detect_and_transform_sample_roi(
        self,
        tag_id: int,
        strategy: str = "farthest_edge",
        edge_inset_mm: float = 0.0,
        timeout: float = 45.0,
    ) -> PoseStamped | None:
        """Return a sample-ROI pickup pose in base_link; timeout is per attempt."""
        total_attempts = 1 + self._retry_count

        for attempt in range(total_attempts):
            if attempt > 0:
                self.logger.info(
                    f"Retry {attempt}/{self._retry_count} for sample_roi tag {tag_id} "
                    f"(waiting {self._retry_delay}s...)"
                )
                time.sleep(self._retry_delay)

            result = self._camera.detect_sample_roi(
                self.rclpy_node,
                tag_id=tag_id,
                strategy=strategy,
                edge_inset_mm=edge_inset_mm,
                dictionary=self._marker_dictionary,
                timeout=timeout,
            )

            if result is None:
                continue

            pickup_pose, capture_stamp = result

            self.logger.info(
                f"Sample ROI: pickup at [{pickup_pose.position.x:.4f}, "
                f"{pickup_pose.position.y:.4f}, {pickup_pose.position.z:.4f}] "
                f"in camera frame"
            )

            pose_base = self._transform_to_base_link(
                pickup_pose, capture_stamp=capture_stamp
            )
            if pose_base is None:
                continue

            self.logger.info(
                f"Transformed to base_link: [{pose_base.pose.position.x:.4f}, "
                f"{pose_base.pose.position.y:.4f}, {pose_base.pose.position.z:.4f}]"
            )

            self._broadcast_detection_tf(f"sample_roi_tag{tag_id}", pose_base)

            if attempt > 0:
                self.logger.info(f"sample_roi detection succeeded on retry {attempt}")
            return pose_base

        self.logger.error(
            f"Failed sample_roi detection for tag {tag_id} after {total_attempts} attempts"
        )
        return None

    # Coordinate transforms

    def reset_tf(self):
        """Replace cached transforms and restart the TF listener."""
        self.logger.info("Resetting TF buffer (URDF changed)")
        self._tf_listener.unregister()
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self.rclpy_node)
        self.logger.info("TF buffer replaced, listener re-created")

    def _transform_to_base_link(
        self, pose_camera: Pose, capture_stamp=None
    ) -> PoseStamped | None:
        """Transform into base_link at capture time, or use the latest TF if absent."""
        try:
            if capture_stamp is not None:
                lookup_time = rclpy.time.Time.from_msg(capture_stamp)
                self.logger.debug(
                    f"Using capture timestamp for TF lookup: "
                    f"{capture_stamp.sec}.{capture_stamp.nanosec:09d}"
                )
            else:
                lookup_time = rclpy.time.Time()
                self.logger.debug("Using latest TF (no capture timestamp provided)")

            if not self._tf_buffer.can_transform(
                "base_link",
                self._camera_frame,
                lookup_time,
                timeout=rclpy.duration.Duration(seconds=2.0),
            ):
                self.logger.error(
                    f"TF {self._camera_frame} -> base_link not available "
                    f"at requested time"
                )
                return None

            pose_in = PoseStamped()
            pose_in.header.frame_id = self._camera_frame
            if capture_stamp is not None:
                pose_in.header.stamp = capture_stamp
            else:
                pose_in.header.stamp = self.rclpy_node.get_clock().now().to_msg()
            pose_in.pose = pose_camera

            transform = self._tf_buffer.lookup_transform(
                "base_link",
                self._camera_frame,
                lookup_time,
                timeout=rclpy.duration.Duration(seconds=2.0),
            )
            pose_out = do_transform_pose_stamped(pose_in, transform)
            pose_out.header.frame_id = "base_link"

            return pose_out

        except TransformException as e:
            self.logger.error(f"TF failed: {e}")
            return None

    def _broadcast_detection_tf(self, frame_name: str, pose: PoseStamped):
        """Broadcast a base_link detection pose as a named TF frame."""
        tf = TransformStamped()
        tf.header.stamp = self.rclpy_node.get_clock().now().to_msg()
        tf.header.frame_id = "base_link"
        tf.child_frame_id = frame_name
        tf.transform.translation.x = pose.pose.position.x
        tf.transform.translation.y = pose.pose.position.y
        tf.transform.translation.z = pose.pose.position.z
        tf.transform.rotation = pose.pose.orientation
        self._tf_broadcaster.sendTransform(tf)

    # Approach geometry and motion

    def compute_approach_pose(
        self,
        target: PoseStamped,
        z_offset_override: float = 0.0,
        marker_offset_x: float = 0.0,
        marker_offset_y: float = 0.0,
        marker_offset_z: float = 0.0,
        ik_frame_override: str = "",
    ) -> tuple[PoseStamped, str]:
        """Return the base_link approach pose and active IK frame."""
        # IK frame: override, configured non-flange frame, then TF detection.
        if ik_frame_override:
            active_ik_frame = ik_frame_override
            active_z_offset = self._z_offset_for_frame(active_ik_frame)
            self.logger.info(
                f"Using orchestrator IK frame: {active_ik_frame} "
                f"(z_offset: {active_z_offset:.3f})"
            )
        elif self.ik_frame and self.ik_frame != "flange":
            active_ik_frame = self.ik_frame
            active_z_offset = self._z_offset_for_frame(active_ik_frame)
        else:
            detection = self._detect_current_gripper()
            active_ik_frame = detection.ik_frame
            active_z_offset = detection.z_offset
            self.logger.info(
                f"Auto-detected: {active_ik_frame} (z_offset: {active_z_offset:.3f})"
            )

        # A zero z offset selects the gripper default.
        if z_offset_override != 0.0:
            self.logger.info(f"Using z_offset override: {z_offset_override:.3f}")
            active_z_offset = z_offset_override

        # Rotate marker-local offsets (metres) into base_link.
        if marker_offset_x != 0.0 or marker_offset_y != 0.0 or marker_offset_z != 0.0:
            q = [
                target.pose.orientation.x,
                target.pose.orientation.y,
                target.pose.orientation.z,
                target.pose.orientation.w,
            ]
            rot_matrix = quaternion_matrix(q)[:3, :3]
            local_offset = np.array([marker_offset_x, marker_offset_y, marker_offset_z])
            world_offset = rot_matrix @ local_offset
            self.logger.info(
                f"Marker offset: [{marker_offset_x:.3f}, {marker_offset_y:.3f}, {marker_offset_z:.3f}] → "
                f"world [{world_offset[0]:.3f}, {world_offset[1]:.3f}, {world_offset[2]:.3f}]"
            )
        else:
            world_offset = np.array([0.0, 0.0, 0.0])

        approach = PoseStamped()
        approach.header = target.header
        approach.pose.position.x = target.pose.position.x + world_offset[0]
        approach.pose.position.y = target.pose.position.y + world_offset[1]
        approach.pose.position.z = (
            target.pose.position.z + world_offset[2] + active_z_offset
        )

        # Use roll=180°, pitch=0° and marker yaw rotated by 180°.
        q_marker = [
            target.pose.orientation.x,
            target.pose.orientation.y,
            target.pose.orientation.z,
            target.pose.orientation.w,
        ]
        q_z180 = quaternion_from_euler(0, 0, math.pi)
        q_rotated = quaternion_multiply(q_marker, q_z180)
        _, _, approach_yaw = euler_from_quaternion(q_rotated)
        q_approach = quaternion_from_euler(math.pi, 0, approach_yaw)
        approach.pose.orientation.x = q_approach[0]
        approach.pose.orientation.y = q_approach[1]
        approach.pose.orientation.z = q_approach[2]
        approach.pose.orientation.w = q_approach[3]

        self.logger.info(
            f"Approach pose: [{approach.pose.position.x:.6f}, "
            f"{approach.pose.position.y:.6f}, {approach.pose.position.z:.6f}] "
            f"yaw={math.degrees(approach_yaw):.4f}°, z_offset={active_z_offset:.3f}"
        )

        return approach, active_ik_frame

    def _apply_flange_offset(
        self, pose: PoseStamped, direction: str, distance: float
    ) -> PoseStamped:
        """Shift a base_link pose in place along the current flange axes (metres)."""
        if direction not in DIRECTION_VECTORS:
            raise ValueError(
                f"Unknown flange offset direction: {direction!r} "
                f"(valid: {list(DIRECTION_VECTORS)})"
            )

        try:
            tf = self._tf_buffer.lookup_transform(
                "base_link",
                "flange",
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0),
            )
        except TransformException as e:
            raise RuntimeError(f"Failed to look up flange TF for offset: {e}") from e

        q = [
            tf.transform.rotation.x,
            tf.transform.rotation.y,
            tf.transform.rotation.z,
            tf.transform.rotation.w,
        ]
        rot = quaternion_matrix(q)[:3, :3]

        flange_vec = np.array(DIRECTION_VECTORS[direction])
        base_vec = rot @ flange_vec

        pose.pose.position.x += base_vec[0] * distance
        pose.pose.position.y += base_vec[1] * distance
        pose.pose.position.z += base_vec[2] * distance

        self.logger.info(
            f"Applied offset: {distance * 1000:.1f}mm {direction} "
            f"(flange {flange_vec} → base [{base_vec[0]:.3f}, {base_vec[1]:.3f}, {base_vec[2]:.3f}])"
        )

        return pose

    def _move_to_approach(
        self, approach: PoseStamped, ik_frame: str = "", constraints=None
    ) -> "str | None":
        """Execute a PTP joint goal or LIN pose goal; return an error string or None."""
        if not ik_frame:
            detection = self._detect_current_gripper()
            ik_frame = detection.ik_frame

        # Resolve IK once; use a LIN pose goal if no joint solution is available.
        joint_goal = self.compute_deterministic_ik(approach, ik_frame)
        if joint_goal is None:
            self.logger.warning(
                "Deterministic IK failed, falling back to Cartesian goal"
            )

        task = self.create_task_template("Vision Move")
        planner = self.make_pilz_planner("LIN" if joint_goal is None else "PTP")
        stage = stages.MoveTo("move to tag", planner)
        stage.group = self.arm_group
        if joint_goal is None:
            ik_frame_pose = PoseStamped()
            ik_frame_pose.header.frame_id = ik_frame
            stage.ik_frame = ik_frame_pose
            stage.setGoal(approach)
        else:
            self._set_ik_frame(stage)
            stage.setGoal(joint_goal)
        apply_constraints(stage, constraints)
        task.add(stage)

        return self.load_plan_execute(task)

    def _detect_current_gripper(self) -> GripperDetection:
        """Find the first configured tip frame available in TF, falling back to flange."""
        from beambot.config_loader import configured_tip_frames

        for frame in configured_tip_frames():
            if self._tf_buffer.can_transform(
                "base_link",
                frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            ):
                return GripperDetection(frame, self._z_offset_for_frame(frame))

        self.logger.info("No gripper detected, using flange")
        return GripperDetection("flange", 0.0)

    @staticmethod
    def _z_offset_for_frame(ik_frame: str) -> float:
        """Return the configured tip-frame offset, defaulting to zero."""
        from beambot.config_loader import z_offset_for_tip_frame

        return z_offset_for_tip_frame(ik_frame, default=0.0)

    # Collision objects

    def _add_collision_object_for_tag(self, tag_id: int, tag_pose: PoseStamped):
        """Add the configured collision object at a detected tag pose."""
        if tag_id not in self._object_database:
            return

        info = self._object_database[tag_id]

        object_pose = self._calculate_object_pose(tag_pose, info.tag_offset)

        obj = CollisionObject()
        obj.header.frame_id = object_pose.header.frame_id
        obj.header.stamp = self.rclpy_node.get_clock().now().to_msg()
        obj.id = info.name
        obj.operation = CollisionObject.ADD

        primitive = SolidPrimitive()
        if info.shape == "box" and len(info.dimensions) == 3:
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = info.dimensions
        elif info.shape == "cylinder" and len(info.dimensions) == 2:
            primitive.type = SolidPrimitive.CYLINDER
            primitive.dimensions = info.dimensions
        else:
            raise ValueError(f"Invalid collision-object shape: {info.shape}")

        obj.primitives.append(primitive)
        obj.primitive_poses.append(object_pose.pose)

        self._apply_collision_object(obj)
        self.logger.info(f"Added collision object '{info.name}'")

    def _calculate_object_pose(
        self, tag_pose: PoseStamped, offset: list
    ) -> PoseStamped:
        """Apply a marker-local XYZ offset (metres), preserving the marker orientation."""
        q = [
            tag_pose.pose.orientation.x,
            tag_pose.pose.orientation.y,
            tag_pose.pose.orientation.z,
            tag_pose.pose.orientation.w,
        ]
        rot_matrix = quaternion_matrix(q)[:3, :3]

        offset_local = np.array(offset)
        offset_world = rot_matrix @ offset_local

        result = PoseStamped()
        result.header = tag_pose.header
        result.pose.position.x = tag_pose.pose.position.x + offset_world[0]
        result.pose.position.y = tag_pose.pose.position.y + offset_world[1]
        result.pose.position.z = tag_pose.pose.position.z + offset_world[2]
        result.pose.orientation = tag_pose.pose.orientation

        return result

    def _apply_collision_object(self, obj: CollisionObject) -> None:
        """Apply a collision-object change and require MoveGroup acknowledgement."""
        if not self._apply_scene_client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError("/apply_planning_scene service unavailable")

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(obj)
        request = ApplyPlanningScene.Request()
        request.scene = scene
        future = self._apply_scene_client.call_async(request)
        if not wait_for_future(future, timeout=2.0):
            raise RuntimeError("/apply_planning_scene call timed out")
        response = future.result()
        if response is None or not response.success:
            raise RuntimeError("MoveGroup rejected collision-object update")
