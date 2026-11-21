"""Build and execute MoveTo tasks for relative, Cartesian, and named targets."""

import json
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from moveit.task_constructor import core, stages
from tf2_ros import Buffer, TransformListener
from tf_transformations import quaternion_from_euler, euler_from_quaternion

from beambot.utils.task_parser import moveto_goal_error
from beambot.motion.task_builder import (
    MtcTaskBuilder, parse_constraints, apply_constraints,
)


class MoveToTask(MtcTaskBuilder):
    """Compose MTC movement stages for standalone and batched tasks."""

    def __init__(self, rclpy_node, arm_group: str = "", ik_frame: str = ""):
        super().__init__(rclpy_node, arm_group, ik_frame)
        # Start listening before Cartesian targets need TF.
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self.rclpy_node)

    def close(self) -> None:
        """Release this task instance's TF subscriptions."""
        self._tf_listener.unregister()

    def _get_current_yaw(self, frame: str = "flange") -> float:
        """Return frame yaw relative to base_link in radians; use zero on TF failure."""
        try:
            t = self._tf_buffer.lookup_transform(
                "base_link", frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0)
            )
            q = [
                t.transform.rotation.x,
                t.transform.rotation.y,
                t.transform.rotation.z,
                t.transform.rotation.w
            ]
            _, _, yaw = euler_from_quaternion(q)
            return yaw
        except Exception as e:
            self.logger.warning(f"Failed to get current yaw from {frame}: {e}, defaulting to 0")
            return 0.0

    def _detect_gripper_ik_frame(self) -> str:
        """Return the first configured tip frame found in TF, or flange."""
        from beambot.config_loader import configured_tip_frames
        for frame in configured_tip_frames():
            try:
                if self._tf_buffer.can_transform(
                    "base_link", frame,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=1.0),
                ):
                    self.logger.debug(f"Detected gripper tip frame: {frame}")
                    return frame
            except Exception:
                continue
        self.logger.debug("No gripper tip frame detected, using flange")
        return "flange"

    def add_to_task(self, task: core.Task, goal, planner=None) -> str | None:
        """Append stages without executing; return None or an error string."""
        error = moveto_goal_error(
            target=goal.target,
            direction=goal.direction,
            distance=goal.distance,
            cartesian_target=goal.cartesian_target,
            planning_type=goal.planning_type,
        )
        if error:
            self.logger.error(error)
            return error

        # Only named joint poses should pin the final trajectory endpoint.
        self._pin_goal_joints = None

        planning_type = goal.planning_type if goal.planning_type else ""
        # Empty, auto, and joint select the target-specific fallback chain.
        use_fallback = planning_type in ("", "auto", "joint") and planner is None

        if not use_fallback and planner is None:
            if planning_type == "cartesian":
                planner = self.make_cartesian_planner()
                self.logger.debug("Using Cartesian planner")
            elif planning_type == "pilz":
                planner = self.make_pilz_planner("LIN")
                self.logger.debug("Using Pilz LIN planner")
            elif planning_type == "pilz_ptp":
                planner = self.make_pilz_planner("PTP")
                self.logger.debug("Using Pilz PTP planner")
            else:
                planner = self.make_pipeline_planner()
                self.logger.debug("Using pipeline planner (OMPL)")

        if use_fallback:
            self.logger.debug("Using fallback planning (auto)")

        constraints = parse_constraints(
            json.loads(goal.constraints_json) if goal.constraints_json else None
        )
        if constraints is not None:
            self.logger.debug("Path constraints active for this move")

        # Relative move in the stage's IK frame.
        if goal.direction and goal.distance != 0.0:
            label = f"move_{goal.direction}_{goal.distance:.3f}m"
            stage = self.create_relative_move_stage(
                label, goal.direction, goal.distance,
                planner=planner, constraints=constraints
            )
            task.add(stage)
            self.logger.debug(
                f"Planning relative move: {goal.direction} {goal.distance}m"
            )
            return None

        # Cartesian target: XYZ in meters, optional RPY in degrees.
        elif len(goal.cartesian_target) >= 3:
            # Use the gripper tip for IK and default yaw.
            active_ik_frame = self._detect_gripper_ik_frame()

            pose = PoseStamped()
            pose.header.frame_id = goal.frame_id if goal.frame_id else "base_link"
            pose.pose.position.x = goal.cartesian_target[0]
            pose.pose.position.y = goal.cartesian_target[1]
            pose.pose.position.z = goal.cartesian_target[2]

            if len(goal.cartesian_target) >= 6:
                r = math.radians(goal.cartesian_target[3])
                p = math.radians(goal.cartesian_target[4])
                y = math.radians(goal.cartesian_target[5])
                q = quaternion_from_euler(r, p, y)
            else:
                # XYZ-only: roll=180 deg, pitch=0, live tip yaw in base_link.
                current_yaw = self._get_current_yaw(active_ik_frame)
                q = quaternion_from_euler(math.pi, 0.0, current_yaw)
                self.logger.debug(f"Using current yaw from {active_ik_frame}: {math.degrees(current_yaw):.1f}°")

            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]

            # Precompute joint angles for the automatic PTP candidate.
            joint_goal = self.compute_deterministic_ik(pose, active_ik_frame)

            if use_fallback:
                # Try PTP with precomputed IK, then LIN, then CartesianPath.
                fb = core.Fallbacks("move_to_target")
                if joint_goal is not None:
                    ptp_stage = stages.MoveTo("move_to_target [deterministic IK]", self.make_pilz_planner("PTP"))
                    ptp_stage.group = self.arm_group
                    self._set_ik_frame(ptp_stage)
                    ptp_stage.setGoal(joint_goal)
                    apply_constraints(ptp_stage, constraints)
                    fb.add(ptp_stage)
                for planner_fn, suffix in [
                    (lambda: self.make_pilz_planner("LIN"), "Pilz LIN"),
                    (self.make_cartesian_planner, "CartesianPath"),
                ]:
                    move_stage = stages.MoveTo(
                        f"move_to_target [{suffix}]", planner_fn()
                    )
                    move_stage.group = self.arm_group
                    ik_frame_pose = PoseStamped()
                    ik_frame_pose.header.frame_id = active_ik_frame
                    move_stage.ik_frame = ik_frame_pose
                    move_stage.setGoal(pose)
                    apply_constraints(move_stage, constraints)
                    fb.add(move_stage)
                task.add(fb)
            else:
                if joint_goal is not None and planner is None:
                    move_stage = stages.MoveTo("move_to_target", self.make_pilz_planner("PTP"))
                    move_stage.group = self.arm_group
                    self._set_ik_frame(move_stage)
                    move_stage.setGoal(joint_goal)
                    apply_constraints(move_stage, constraints)
                    task.add(move_stage)
                else:
                    if planner is None:
                        planner = self.make_cartesian_planner()
                    move_stage = stages.MoveTo("move_to_target", planner)
                    move_stage.group = self.arm_group
                    ik_frame_pose = PoseStamped()
                    ik_frame_pose.header.frame_id = active_ik_frame
                    move_stage.ik_frame = ik_frame_pose
                    move_stage.setGoal(pose)
                    apply_constraints(move_stage, constraints)
                    task.add(move_stage)
            self.logger.debug(
                f"Planning Cartesian move to "
                f"[{pose.pose.position.x:.3f}, {pose.pose.position.y:.3f}, "
                f"{pose.pose.position.z:.3f}] in {pose.header.frame_id} "
                f"(ik_frame: {active_ik_frame})"
            )
            return None

        # Named joint pose or SRDF state.
        elif goal.target:
            poses = self.parse_poses(goal.poses_json)
            if poses is None:
                error = f"Failed to parse poses_json for target '{goal.target}'"
                self.logger.error(error)
                return error

            label = f"move_to_{goal.target}"

            if goal.target in poses:
                stage = self.make_move_to_named_stage(
                    label, goal.target, poses,
                    planner=planner, constraints=constraints
                )
                if not stage:
                    return f"Pose '{goal.target}' not found or invalid"
                task.add(stage)
                self.logger.debug(f"Planning move to joint pose: {goal.target}")
            else:
                # SRDF named state.
                if use_fallback:
                    # Try Pilz PTP, then the configured general pipeline.
                    fb = core.Fallbacks(label)
                    for planner_fn, suffix in [
                        (lambda: self.make_pilz_planner("PTP"), "Pilz PTP"),
                        (self.make_pipeline_planner, "OMPL"),
                    ]:
                        s = stages.MoveTo(f"{label} [{suffix}]", planner_fn())
                        s.group = self.arm_group
                        self._set_ik_frame(s)
                        s.setGoal(goal.target)
                        apply_constraints(s, constraints)
                        if suffix == "OMPL":
                            s.timeout = self._ompl_timeout
                        fb.add(s)
                    task.add(fb)
                else:
                    move_stage = stages.MoveTo(label, planner)
                    move_stage.group = self.arm_group
                    self._set_ik_frame(move_stage)
                    move_stage.setGoal(goal.target)
                    apply_constraints(move_stage, constraints)
                    task.add(move_stage)
                self.logger.debug(f"Planning move to named state: {goal.target}")

            return None

        else:
            error = (
                "No valid move target specified. "
                "Provide (direction + distance), cartesian_target, or target."
            )
            self.logger.error(error)
            return error

    def run(self, goal) -> str | None:
        """Build, plan, and execute one goal; return None or an error string."""
        task = self.create_task_template("MoveTo Task")

        error = self.add_to_task(task, goal)
        if error is not None:
            return error

        return self.load_plan_execute(task)
