"""Run vision detection, target computation and Cartesian or joint motion."""

import json
import math
# import threading
import time
from dataclasses import dataclass
from typing import Any

from geometry_msgs.msg import PoseStamped
from moveit.task_constructor import core, stages

from beambot.utils.task_parser import flange_offset_error
from beambot.vision.approach_strategies import (
    CartesianTarget,
    JointTarget,
    get_approach_strategy,
)
from beambot.vision.vision_methods import get_vision_method
from beambot.vision.target_localizer import TargetLocalizer
from beambot.motion.task_builder import apply_constraints, parse_constraints


@dataclass
class VisionTaskContext:
    """Per-goal inputs and results shared by vision_methods and approach strategies."""

    goal: Any
    vision: TargetLocalizer
    scan_positions: list | None = None
    # Pose returned without executing the computed target.
    detect_only_pose: Any = None
    # Distinguishes computation failure from a successful no-motion result.
    error: str | None = None


class VisionTask:
    """Coordinate vision tasks using the shared TargetLocalizer."""

    def __init__(self, rclpy_node, **vision_kwargs):
        self.rclpy_node = rclpy_node
        self.logger = rclpy_node.get_logger()
        self._vision = TargetLocalizer(rclpy_node, **vision_kwargs)
        self.last_detected_pose = None
        self.vacuum_ok = True
        self.goal = None

    def reset_tf(self):
        self._vision.reset_tf()

    def run(self, goal) -> "str | None":
        """Run a vision task; return None on completion or an error string."""
        self.last_detected_pose = None
        self.vacuum_ok = True
        self.goal = goal
        vision = self._vision

        error = flange_offset_error(
            direction=goal.offset_direction,
            distance=goal.offset_distance,
        )
        if error:
            return f"PIPELINE_CONFIG_ERROR: {error}"

        vision_method_name = goal.vision_method or "marker"
        if vision_method_name == "sample_roi":
            strategy = goal.strategy or "farthest_edge"
            if strategy not in {
                "center",
                "farthest_edge",
                "nearest_edge",
                "farthest_corner",
                "nearest_corner",
            }:
                return (
                    f"PIPELINE_CONFIG_ERROR: invalid sample_roi strategy '{strategy}'"
                )
            if not math.isfinite(goal.edge_inset_mm) or goal.edge_inset_mm < 0:
                return (
                    "PIPELINE_CONFIG_ERROR: edge_inset_mm must be finite and "
                    "nonnegative"
                )

        # Pre-scan motion also runs for detect_only requests.
        error = self._move_to_scan(goal)
        if error is not None:
            return error

        # Allow vibration to settle before capture.
        if vision._settle_time > 0:
            self.logger.info(
                f"Waiting {vision._settle_time:.2f}s for robot to settle..."
            )
            time.sleep(vision._settle_time)
            self.logger.info("Settle complete, starting detection")

        ctx = VisionTaskContext(goal=goal, vision=vision)
        ctx.scan_positions = self._parse_scan_positions(goal)

        approach_strategy_name = goal.approach_strategy or "approach_pose"
        try:
            vision_method = get_vision_method(vision_method_name)
            approach_strategy = get_approach_strategy(approach_strategy_name)
        except KeyError as e:
            return f"PIPELINE_CONFIG_ERROR: {e}"

        detection = vision_method(ctx)
        if detection is None:
            return (
                f"DETECTION_FAILED: vision_method '{vision_method_name}' found nothing "
                f"(tag {goal.tag_id}, timeout {goal.timeout}s)"
            )

        target = approach_strategy(detection, ctx)

        if ctx.error is not None:
            return f"GOAL_COMPUTE_FAILED: {ctx.error}"
        # The approach strategy decides whether detect_only skips target execution.
        if ctx.detect_only_pose is not None:
            self.last_detected_pose = ctx.detect_only_pose
            return None
        if target is None:
            return None

        error = self._execute_motion_target(target)
        if error is not None:
            return error

        # Vacuum check disabled; vacuum_ok remains an unchecked True.
        # if isinstance(target, CartesianTarget) and target.grasp_state:
        #     self.vacuum_ok = self._check_vacuum()
        #     self.logger.info(f"vacuum_ok={self.vacuum_ok}")
        return None

    def _execute_motion_target(self, target) -> "str | None":
        """Select the executor for a Cartesian or joint target."""
        if isinstance(target, CartesianTarget):
            return self._execute_cartesian(target)

        if isinstance(target, JointTarget):
            return self._execute_joints(target)

        return f"PIPELINE_ERROR: no executor arm for {type(target).__name__}"

    def _execute_joints(self, target: JointTarget) -> "str | None":
        """Execute joint positioning, optional contact and gripper actions separately."""
        vision = self._vision
        goal = self.goal
        constraints = parse_constraints(
            json.loads(goal.constraints_json)
            if getattr(goal, "constraints_json", "")
            else None
        )

        pose_key = "_vision_task_joint_target"
        poses = {pose_key: list(target.joints_deg)}
        task = vision.create_task_template("Vision Task Joint Move")
        stage = vision.make_move_to_named_stage(
            "corrected joint move", pose_key, poses, planner=None,
            constraints=constraints,
        )
        if stage is None:
            return "PIPELINE_ERROR: failed to build joint move stage"
        task.add(stage)
        error = vision.load_plan_execute(task)
        if error:
            return f"corrected joint move failed: {error}"

        if target.forward_distance > 0:
            self.logger.info(f"moving forward {target.forward_distance * 1000:.1f}mm")
            fwd_task = vision.create_task_template("Vision Task Forward")
            fwd_stage = vision.create_relative_move_stage(
                "forward contact",
                "forward",
                target.forward_distance,
                constraints=constraints,
            )
            fwd_task.add(fwd_stage)
            error = vision.load_plan_execute(fwd_task)
            if error:
                return f"forward move failed: {error}"

        if target.terminal_state:
            self.logger.info(f"terminal gripper: {target.terminal_state}")
            term_task = vision.create_task_template("Vision Task Terminal")
            term_stage = vision.make_gripper_stage(
                "terminal",
                vision.make_joint_interpolation_planner(),
                target.gripper_group,
                target.terminal_state,
            )
            if term_stage:
                term_task.add(term_stage)
                error = vision.load_plan_execute(term_task)
                if error:
                    return f"terminal action failed: {error}"

        return None

    def _execute_cartesian(self, target: CartesianTarget) -> "str | None":
        """Execute an approach, optionally with a gripper action and retreat."""
        vision = self._vision
        goal = self.goal
        constraints = parse_constraints(
            json.loads(goal.constraints_json)
            if getattr(goal, "constraints_json", "")
            else None
        )

        # Bare approaches use the engine's standalone motion path.
        if not target.grasp_state and not target.retreat_pose_key:
            return vision._move_to_approach(
                target.pose, ik_frame=target.ik_frame, constraints=constraints,
            )

        # Approach, gripper action and retreat share one task.
        joint_goal = vision.compute_deterministic_ik(target.pose, target.ik_frame)
        task = vision.create_task_template("Vision Task Grasp")

        # Prefer a joint-space PTP goal; fall back to a LIN pose goal.
        approach_fb = core.Fallbacks("approach")
        if joint_goal is not None:
            ptp = stages.MoveTo("approach [PTP]", vision.make_pilz_planner("PTP"))
            ptp.group = vision.arm_group
            vision._set_ik_frame(ptp)
            ptp.setGoal(joint_goal)
            apply_constraints(ptp, constraints)
            approach_fb.add(ptp)
        lin = stages.MoveTo("approach [LIN]", vision.make_pilz_planner("LIN"))
        lin.group = vision.arm_group
        ik_frame_pose = PoseStamped()
        ik_frame_pose.header.frame_id = target.ik_frame
        lin.ik_frame = ik_frame_pose
        lin.setGoal(target.pose)
        apply_constraints(lin, constraints)
        approach_fb.add(lin)
        task.add(approach_fb)

        if target.grasp_state:
            grasp = vision.make_gripper_stage(
                "gripper",
                vision.make_joint_interpolation_planner(),
                target.gripper_group,
                target.grasp_state,
            )
            if grasp:
                task.add(grasp)

        if target.retreat_pose_key:
            poses = vision.parse_poses(goal.poses_json) or {}
            retreat = vision.make_move_to_named_stage(
                "retreat",
                target.retreat_pose_key,
                poses,
                constraints=constraints,
            )
            if not retreat:
                return (
                    f"Pose '{target.retreat_pose_key}' not found or invalid (retreat)"
                )
            task.add(retreat)

        return vision.load_plan_execute(task)

    def _move_to_scan(self, goal) -> "str | None":
        """Move to scan_pose, optionally opening the gripper first."""
        scan_pose = getattr(goal, "scan_pose", "") or ""
        if not scan_pose:
            return None

        vision = self._vision
        poses = vision.parse_poses(goal.poses_json)
        if poses is None:
            return "Failed to parse poses_json"
        constraints = parse_constraints(
            json.loads(goal.constraints_json)
            if getattr(goal, "constraints_json", "")
            else None
        )
        states = (
            json.loads(goal.gripper_states_json)
            if getattr(goal, "gripper_states_json", "")
            else {}
        )

        task = vision.create_task_template("Position for Task")
        if getattr(goal, "pre_open", False) and states.get("release"):
            open_stage = vision.make_gripper_stage(
                "open gripper",
                vision.make_joint_interpolation_planner(),
                goal.gripper_group,
                states["release"],
            )
            if open_stage:
                task.add(open_stage)
        scan_stage = vision.make_move_to_named_stage(
            "scan position",
            scan_pose,
            poses,
            constraints=constraints,
        )
        if not scan_stage:
            return f"Pose '{scan_pose}' not found or invalid (scan position)"
        task.add(scan_stage)
        error = vision.load_plan_execute(task)
        return f"Position failed: {error}" if error else None

    # def _check_vacuum(self) -> bool:
    #     """Return False only when ePick reports NO_OBJECT_DETECTED."""
    #     try:
    #         from epick_msgs.msg import ObjectDetectionStatus
    #     except ImportError:
    #         return True
    #
    #     msg_holder = [None]
    #     event = threading.Event()
    #
    #     def _on_status(msg):
    #         msg_holder[0] = msg
    #         event.set()
    #
    #     sub = self.rclpy_node.create_subscription(
    #         ObjectDetectionStatus,
    #         "/object_detection_status",
    #         _on_status,
    #         10,
    #     )
    #     event.wait(timeout=1.0)
    #     self.rclpy_node.destroy_subscription(sub)
    #
    #     if msg_holder[0] is None:
    #         return True  # No status received; grasp is unverified.
    #     NO_OBJECT = 3
    #     detected = msg_holder[0].status != NO_OBJECT
    #     if not detected:
    #         self.logger.warning(
    #             "VACUUM_LOST: ePick reports NO_OBJECT_DETECTED after pick"
    #         )
    #     return detected

    def _parse_scan_positions(self, goal) -> "list | None":
        """Group flattened scan poses into six-joint lists in radians."""
        num = getattr(goal, "num_scan_positions", 0)
        if num <= 0:
            return None
        flat = list(getattr(goal, "scan_positions_flat", []))
        if len(flat) != num * 6:
            self.logger.warning(
                f"Invalid scan_positions_flat length: {len(flat)}, expected "
                f"{num * 6}. Falling back to single-position."
            )
            return None
        self.logger.info(f"Multi-position mode enabled: {num} scan positions")
        return [flat[i * 6 : (i + 1) * 6] for i in range(num)]
