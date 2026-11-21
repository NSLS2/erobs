"""Shared MTC node setup, planning helpers, and trajectory execution."""

import json
import math
import sys
import threading
import time
import traceback
from typing import Any

import rclcpp
import yaml
from ament_index_python.packages import get_package_share_path
from geometry_msgs.msg import PoseStamped, Vector3, Vector3Stamped
from moveit.task_constructor import core, stages
from moveit_msgs.msg import (
    Constraints, DisplayTrajectory, JointConstraint, MoveItErrorCodes,
    OrientationConstraint, RobotState, RobotTrajectory,
)
from moveit_msgs.srv import GetPositionIK
from moveit_task_constructor_msgs.action import ExecuteTaskSolution
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from tf_transformations import quaternion_from_euler

from beambot.config_loader import (
    arm_joint_names,
    build_pipeline_param_args,
    moveit_config_package,
)
from beambot.utils import MODEL_REVISION_PREFIX, VERIFIED_MODEL_DESCRIPTION

MOVEIT_ERROR_NAMES: dict[int, str] = {
    getattr(MoveItErrorCodes, n): n
    for n in dir(MoveItErrorCodes)
    if n.isupper() and isinstance(getattr(MoveItErrorCodes, n), int)
}


# IK-frame directions; y/-y invert the Y axis.
DIRECTION_VECTORS: dict[str, tuple[float, float, float]] = {
    "forward":  ( 1.0,  0.0,  0.0), "x":  ( 1.0,  0.0,  0.0),
    "backward": (-1.0,  0.0,  0.0), "-x": (-1.0,  0.0,  0.0),
    "right":    ( 0.0, -1.0,  0.0), "y":  ( 0.0, -1.0,  0.0),
    "left":     ( 0.0,  1.0,  0.0), "-y": ( 0.0,  1.0,  0.0),
    "up":       ( 0.0,  0.0,  1.0), "z":  ( 0.0,  0.0,  1.0),
    "down":     ( 0.0,  0.0, -1.0), "-z": ( 0.0,  0.0, -1.0),
}

# arm_joint_names() already falls back to the standard UR order.
DEFAULT_JOINT_NAMES: list[str] = arm_joint_names()

# Default fractions of the configured motion limits.
VELOCITY_SCALING = 0.2
ACCELERATION_SCALING = 0.2

DEFAULT_ARM_GROUP = "ur_arm"
DEFAULT_IK_FRAME = "flange"


# TODO: Await ROS futures once callers are async; preserve timeout/cancellation handling.
def wait_for_future(future, timeout: float, poll_interval: float = 0.01) -> bool:
    """Wait for completion without spinning; return False on timeout."""
    # Requires an executor to process responses concurrently.
    deadline = time.monotonic() + timeout
    while not future.done():
        if time.monotonic() >= deadline:
            return False
        time.sleep(poll_interval)
    return True


def _load_joint_accel_limits() -> dict[str, float]:
    """Merge enabled acceleration limits; warn and keep the first value on conflicts."""
    cfg_root = get_package_share_path(moveit_config_package()) / "config"
    limits: dict[str, float] = {}
    for directory in sorted(cfg_root.iterdir()):
        path = directory / "joint_limits.yaml"
        if not path.is_file():
            continue
        with path.open() as f:
            data = yaml.safe_load(f) or {}
        for joint, spec in (data.get("joint_limits") or {}).items():
            if not spec.get("has_acceleration_limits"):
                continue
            if "max_acceleration" not in spec:
                continue
            value = float(spec["max_acceleration"])
            existing = limits.get(joint)
            if existing is not None and existing != value:
                print(
                    f"[beambot_mtc] WARN: max_acceleration for '{joint}' "
                    f"differs across gripper configs ({existing} vs {value} "
                    f"in {directory.name}/joint_limits.yaml); using first-seen "
                    f"({existing}). Time to make beambot_mtc gripper-aware.",
                    file=sys.stderr,
                )
                continue
            limits[joint] = value
    return limits


def _build_joint_limit_args(
    limits: dict[str, float], robot_description: str = "robot_description"
) -> list[str]:
    args: list[str] = []
    for joint, accel in limits.items():
        prefix = f"{robot_description}_planning.joint_limits.{joint}"
        args += [
            "-p", f"{prefix}.has_acceleration_limits:=true",
            "-p", f"{prefix}.max_acceleration:={accel}",
        ]
    return args


def _build_kinematics_args(robot_description: str) -> list[str]:
    """Load kinematics.yaml as ROS arguments under the supplied description root."""
    path = get_package_share_path(moveit_config_package()) / "config" / "kinematics.yaml"
    with path.open() as f:
        kinematics = yaml.safe_load(f) or {}
    args: list[str] = []
    for group, settings in kinematics.items():
        for key, value in settings.items():
            rendered = json.dumps(value).replace('"', "'")
            args += [
                "-p",
                f"{robot_description}_kinematics.{group}.{key}:={rendered}",
            ]
    return args


def _build_cartesian_limit_args() -> list[str]:
    """Load Pilz Cartesian limits as ROS parameter arguments."""
    path = (
        get_package_share_path(moveit_config_package()) / "config" / "pilz_cartesian_limits.yaml"
    )
    with path.open() as f:
        limits = (yaml.safe_load(f) or {})["cartesian_limits"]
    args: list[str] = []
    for key, value in limits.items():
        args += ["-p", f"robot_description_planning.cartesian_limits.{key}:={value}"]
    return args


_options = rclcpp.NodeOptions()
_options.automatically_declare_parameters_from_overrides = True
_options.allow_undeclared_parameters = True
_options.enable_rosout = False  # Avoid rosout collisions from node-name remaps.
_joint_accel_limits = _load_joint_accel_limits()
# Keep global arguments so launch-provided kinematics remain available.
_options.arguments = [
    "--ros-args",
    "-r", "__node:=beambot_mtc",
    *build_pipeline_param_args(),
    "-p", "ompl.start_state_max_bounds_error:=0.1",
    *_build_kinematics_args(VERIFIED_MODEL_DESCRIPTION),
    *_build_cartesian_limit_args(),
    *_build_joint_limit_args(_joint_accel_limits),
    *_build_joint_limit_args(_joint_accel_limits, VERIFIED_MODEL_DESCRIPTION),
]
_mtc_node = None
_mtc_node_lock = threading.Lock()


def _get_mtc_node():
    """Create the shared C++ node when the first stage is constructed."""
    global _mtc_node
    with _mtc_node_lock:
        if _mtc_node is None:
            rclcpp.init()
            _mtc_node = rclcpp.Node("beambot_mtc", _options)
        return _mtc_node

# Reuse the most recent RobotModel by revision.
_model_cache: dict = {}


def joints_from_degrees(degrees: list[float]) -> dict[str, float]:
    """Map angles in degrees to configured joint names with values in radians."""
    if len(degrees) != len(DEFAULT_JOINT_NAMES):
        raise ValueError(
            f"Expected {len(DEFAULT_JOINT_NAMES)} joint values, "
            f"got {len(degrees)}"
        )
    return {
        name: math.radians(deg)
        for name, deg in zip(DEFAULT_JOINT_NAMES, degrees)
    }


def parse_constraints(constraints_dict: dict[str, Any] | None) -> Constraints | None:
    """Convert joint/orientation constraints from degrees; return None if empty."""
    if not constraints_dict:
        return None

    constraints = Constraints()

    for jc_dict in constraints_dict.get("joint_constraints", []):
        jc = JointConstraint()
        jc.joint_name = jc_dict["joint_name"]
        jc.position = math.radians(jc_dict["position"])
        jc.tolerance_above = math.radians(jc_dict.get("tolerance_above", 1.0))
        jc.tolerance_below = math.radians(jc_dict.get("tolerance_below", 1.0))
        jc.weight = float(jc_dict.get("weight", 1.0))
        constraints.joint_constraints.append(jc)

    for oc_dict in constraints_dict.get("orientation_constraints", []):
        oc = OrientationConstraint()
        oc.header.frame_id = oc_dict.get("frame_id", "base_link")
        oc.link_name = oc_dict["link_name"]

        orient = oc_dict["orientation"]
        q = quaternion_from_euler(
            math.radians(orient[0]),
            math.radians(orient[1]),
            math.radians(orient[2]),
        )
        oc.orientation.x = q[0]
        oc.orientation.y = q[1]
        oc.orientation.z = q[2]
        oc.orientation.w = q[3]

        tol = oc_dict.get("tolerance", [5.0, 5.0, 5.0])
        oc.absolute_x_axis_tolerance = math.radians(tol[0])
        oc.absolute_y_axis_tolerance = math.radians(tol[1])
        oc.absolute_z_axis_tolerance = math.radians(tol[2])

        param = oc_dict.get("parameterization", "rotation_vector")
        if param == "rotation_vector":
            oc.parameterization = OrientationConstraint.ROTATION_VECTOR
        elif param == "euler_xyz":
            oc.parameterization = OrientationConstraint.XYZ_EULER_ANGLES
        elif isinstance(param, int):
            oc.parameterization = param

        oc.weight = float(oc_dict.get("weight", 1.0))
        constraints.orientation_constraints.append(oc)

    if not constraints.joint_constraints and not constraints.orientation_constraints:
        return None

    return constraints


def apply_constraints(stage, constraints: Constraints | None) -> None:
    if constraints is not None:
        stage.path_constraints = constraints


class MtcTaskBuilder:
    """Shared task construction, planning, and execution."""

    def __init__(
        self,
        rclpy_node,
        arm_group: str = "",
        ik_frame: str = ""
    ):
        self.rclpy_node = rclpy_node
        self._mtc_node = _get_mtc_node()
        self.arm_group = arm_group if arm_group else DEFAULT_ARM_GROUP
        self.ik_frame = ik_frame if ik_frame else DEFAULT_IK_FRAME
        self.logger = rclpy_node.get_logger()
        self.last_sol_msg = None  # Serialized solution for preview and replay.

        try:
            from beambot.config_loader import load_beamline_config
            cfg, _ = load_beamline_config()
            robot_cfg = cfg.get("robot", {})
            self._velocity_scaling = float(robot_cfg.get("velocity_scaling", VELOCITY_SCALING))
            self._acceleration_scaling = float(robot_cfg.get("acceleration_scaling", ACCELERATION_SCALING))
        except Exception:
            self._velocity_scaling = VELOCITY_SCALING
            self._acceleration_scaling = ACCELERATION_SCALING

        self._ompl_timeout = 5.0

        self._pipeline = "stomp" if any(
            arg.startswith("stomp.planning_plugins:=") for arg in _options.arguments
        ) else "ompl"

        self._pin_goal_joints = None  # Named target for endpoint correction.

        self._task_planner_cache: dict = {}

    def create_task_template(self, name: str) -> core.Task:
        """Create a task with its robot model and current-state stage."""
        self._pin_goal_joints = None
        # Planner instances cannot be shared across tasks.
        self._task_planner_cache.clear()

        task = core.Task("", False)  # Disable introspection to avoid extra nodes.
        task.name = name

        model_revision = getattr(self.rclpy_node, "_robot_model_revision", "")
        if not model_revision:
            model_revision = getattr(
                getattr(self.rclpy_node, "_moveit_manager", None),
                "model_revision",
                "",
            )

        model = _model_cache.get(model_revision) if model_revision else None
        if model is not None:
            task.setRobotModel(model)
        else:
            managed_model = model_revision.startswith(MODEL_REVISION_PREFIX)
            if managed_model:
                task.loadRobotModel(self._mtc_node, VERIFIED_MODEL_DESCRIPTION)
            else:
                task.loadRobotModel(self._mtc_node)
            # Model structure per gripper is checked offline by test_robot_models.py.
            if model_revision:
                _model_cache.clear()
                _model_cache[model_revision] = task.getRobotModel()

        task.add(stages.CurrentState("current_state"))
        return task

    def make_pipeline_planner(self) -> core.PipelinePlanner:
        """Return a task-local STOMP or OMPL/RRTstar planner."""
        cached = self._task_planner_cache.get((self._pipeline,))
        if cached is not None:
            return cached
        if self._pipeline == "ompl":
            planner = core.PipelinePlanner(self._mtc_node, "ompl", planner_id="RRTstar")
        else:
            planner = core.PipelinePlanner(self._mtc_node, self._pipeline)
        planner.goal_joint_tolerance = 1e-4
        planner.max_velocity_scaling_factor = self._velocity_scaling
        planner.max_acceleration_scaling_factor = self._acceleration_scaling
        self._task_planner_cache[(self._pipeline,)] = planner
        return planner

    def make_cartesian_planner(self) -> core.CartesianPath:
        cached = self._task_planner_cache.get(("cartesian",))
        if cached is not None:
            return cached
        planner = core.CartesianPath()
        planner.max_velocity_scaling_factor = self._velocity_scaling
        planner.max_acceleration_scaling_factor = self._acceleration_scaling
        planner.step_size = 0.0005  # 0.5 mm.
        planner.min_fraction = 0.9  # Accept at least 90% of the requested path.
        self._task_planner_cache[("cartesian",)] = planner
        return planner

    def make_pilz_planner(self, mode: str = "LIN") -> core.PipelinePlanner:
        """Return a task-local Pilz LIN or PTP planner."""
        cached = self._task_planner_cache.get(("pilz", mode))
        if cached is not None:
            return cached
        planner = core.PipelinePlanner(
            self._mtc_node, "pilz_industrial_motion_planner", planner_id=mode
        )
        planner.max_velocity_scaling_factor = self._velocity_scaling
        planner.max_acceleration_scaling_factor = self._acceleration_scaling
        self._task_planner_cache[("pilz", mode)] = planner
        return planner

    def make_joint_interpolation_planner(self) -> core.JointInterpolationPlanner:
        cached = self._task_planner_cache.get(("joint_interp",))
        if cached is not None:
            return cached
        planner = core.JointInterpolationPlanner()
        planner.max_velocity_scaling_factor = self._velocity_scaling
        planner.max_acceleration_scaling_factor = self._acceleration_scaling
        self._task_planner_cache[("joint_interp",)] = planner
        return planner

    def _set_ik_frame(self, stage) -> None:
        ik_frame_pose = PoseStamped()
        ik_frame_pose.header.frame_id = self.ik_frame
        stage.ik_frame = ik_frame_pose

    def create_relative_move_stage(
        self,
        label: str,
        direction: str,
        distance: float,
        planner=None,
        constraints: Constraints | None = None
    ):
        """Build a relative move in the IK frame; distance is in meters."""
        if direction not in DIRECTION_VECTORS:
            raise ValueError(
                f"Unknown direction: '{direction}'. "
                f"Valid options: {list(DIRECTION_VECTORS.keys())}"
            )

        vec = DIRECTION_VECTORS[direction]

        if planner is not None:
            planners = [(planner, None)]
        else:
            planners = [
                (self.make_pilz_planner("LIN"), "Pilz LIN"),
                (self.make_cartesian_planner(), "CartesianPath"),
                (self.make_pilz_planner("PTP"), "Pilz PTP"),
            ]

        fb = core.Fallbacks(label) if len(planners) > 1 else None

        for p, suffix in planners:
            stage_name = f"{label} [{suffix}]" if suffix is not None else label
            stage = stages.MoveRelative(stage_name, p)
            stage.group = self.arm_group
            self._set_ik_frame(stage)
            direction_vec = Vector3Stamped(
                header=Header(frame_id=self.ik_frame),
                vector=Vector3(
                    x=vec[0] * distance,
                    y=vec[1] * distance,
                    z=vec[2] * distance,
                ),
            )
            stage.setDirection(direction_vec)
            apply_constraints(stage, constraints)

            if fb is None:
                return stage
            fb.add(stage)

        return fb

    def load_plan_execute(self, task: core.Task, dry_run: bool = False) -> str | None:
        """Plan and execute a task, or preview a dry run; return None or an error."""
        error = self.init_and_plan(task, dry_run=dry_run)
        if error is not None or dry_run:
            return error
        return self.execute_solution(task)

    def init_and_plan(self, task: core.Task, dry_run: bool = False) -> str | None:
        """Plan a task and optionally publish its preview; return None or an error."""
        self.last_sol_msg = None
        try:
            self.logger.info(f"Initializing task: {task.name}")
            try:
                task.init()
            except Exception as e:
                error = f"Task init failed for '{task.name}': {e}"
                self.logger.error(error)
                return error

            self.logger.info(f"Planning task: {task.name}")
            if not task.plan(max_solutions=1):
                error = f"PLANNING_FAILED: No motion plan found for task '{task.name}'"
                self.logger.error(error)
                return error

            if not task.solutions:
                error = f"PLANNING_FAILED: No solutions found for task '{task.name}'"
                self.logger.error(error)
                return error

            self.logger.info(
                f"Found {len(task.solutions)} solution(s), "
                f"{'previewing' if dry_run else 'ready to execute'}: {task.name}"
            )

            sol_msg = None
            try:
                sol = task.solutions[0]
                sol_msg = sol.toMsg()
                for i, sub in enumerate(sol_msg.sub_trajectory):
                    traj = sub.trajectory.joint_trajectory
                    if traj.joint_names and traj.points:
                        last_pt = traj.points[-1]
                        pairs = [(n, math.degrees(p)) for n, p in zip(traj.joint_names, last_pt.positions)]
                        joint_str = ', '.join(f'{n}={v:.2f}' for n, v in pairs)
                        self.logger.info(f"Planned [{i}]: {joint_str}")
            except Exception as e:
                self.logger.warning(f"Could not extract planned joints: {e}")

            self.last_sol_msg = sol_msg

            if sol_msg is not None:
                self._pin_endpoints(sol_msg)

            if dry_run and sol_msg is not None:
                self._publish_preview_trajectory(sol_msg, task.name)
                self.logger.info(f"Dry-run preview complete: {task.name}")

            return None

        except Exception as e:
            self.logger.error(f"Task planning failed: {task.name} - {e}")
            self.logger.error(traceback.format_exc())
            return f"Task planning exception for '{task.name}': {e}"

    def execute_solution(self, task: core.Task) -> str | None:
        """Execute the saved message or first solution; return None or an error."""
        try:
            if not task.solutions:
                error = f"EXECUTION_FAILED: No cached solution for '{task.name}'"
                self.logger.error(error)
                return error

            if self.last_sol_msg is not None:
                return self.execute_solution_msg(self.last_sol_msg)

            # Native execution bypasses serialized endpoint adjustments.
            result = task.execute(task.solutions[0])
            if result.val != MoveItErrorCodes.SUCCESS:
                error_name = MOVEIT_ERROR_NAMES.get(result.val, "UNKNOWN")
                error = (
                    f"EXECUTION_FAILED: Task '{task.name}' execution failed: "
                    f"{error_name} (error code: {result.val})"
                )
                self.logger.error(error)
                return error

            self.logger.info(f"Task completed successfully: {task.name}")
            return None

        except Exception as e:
            self.logger.error(f"Task execution failed: {task.name} - {e}")
            self.logger.error(traceback.format_exc())
            return f"Task execution exception for '{task.name}': {e}"

    def _pin_endpoints(self, sol_msg) -> None:
        """Adjust serialized endpoints without retiming or revalidation."""
        try:
            mgr = getattr(self.rclpy_node, "_moveit_manager", None)
            cur = getattr(mgr, "_joint_positions", None) if mgr else None
            subs = [s for s in sol_msg.sub_trajectory
                    if s.trajectory.joint_trajectory.joint_names
                    and s.trajectory.joint_trajectory.points]
            if not subs:
                return

            def _set(pt, names, targets):
                pos = list(pt.positions)
                for i, nm in enumerate(names):
                    if nm in targets:
                        pos[i] = targets[nm]
                pt.positions = pos
                if pt.velocities:
                    pt.velocities = [0.0] * len(pt.velocities)
                if pt.accelerations:
                    pt.accelerations = [0.0] * len(pt.accelerations)

            if cur:
                jt = subs[0].trajectory.joint_trajectory
                _set(jt.points[0], jt.joint_names, cur)
            if self._pin_goal_joints:
                jt = subs[-1].trajectory.joint_trajectory
                _set(jt.points[-1], jt.joint_names, self._pin_goal_joints)
        except Exception as e:
            self.logger.warning(f"endpoint-pin skipped: {e}")

    def execute_solution_msg(self, sol_msg, is_replay: bool = False) -> str | None:
        """Execute a serialized MTC solution; return None or an error string."""
        try:
            # Share the action client across stage instances.
            client = getattr(self.rclpy_node, "_exec_task_solution_client", None)
            if client is None:
                client = ActionClient(
                    self.rclpy_node, ExecuteTaskSolution, "execute_task_solution"
                )
                self.rclpy_node._exec_task_solution_client = client
            if not client.wait_for_server(timeout_sec=5.0):
                return "EXECUTION_FAILED: execute_task_solution server unavailable (5s)"

            goal = ExecuteTaskSolution.Goal()
            goal.solution = sol_msg

            send_future = client.send_goal_async(goal)
            if not wait_for_future(send_future, timeout=10.0):
                # Acceptance is unknown; do not retry after this timeout.
                return "REPLAY_TIMEOUT: replay goal acceptance timed out (10s)"
            gh = send_future.result()
            if not gh.accepted:
                return "EXECUTION_FAILED: execute_task_solution goal rejected"

            result_future = gh.get_result_async()
            if not wait_for_future(result_future, timeout=120.0):
                # Request cancellation without assuming the goal has stopped.
                cancel_future = gh.cancel_goal_async()
                wait_for_future(cancel_future, timeout=5.0)
                return "REPLAY_TIMEOUT: replay execution timed out (120s); aborted"

            ec = result_future.result().result.error_code
            if ec.val != MoveItErrorCodes.SUCCESS:
                name = MOVEIT_ERROR_NAMES.get(ec.val, "UNKNOWN")
                return f"EXECUTION_FAILED: replay failed: {name} (error code: {ec.val})"
            if is_replay:
                self.logger.info("Cached trajectory replayed successfully")
            return None
        except Exception as e:
            self.logger.error(f"Replay execution failed: {e}")
            self.logger.error(traceback.format_exc())
            return f"Replay execution exception: {e}"

    def _publish_preview_trajectory(self, sol_msg, task_name: str) -> None:
        """Publish the serialized plan with its start state and original timing."""
        try:
            from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
            publisher = getattr(self.rclpy_node, "_preview_trajectory_pub", None)
            if publisher is None:
                # Retain the preview for transient-local subscribers.
                qos = QoSProfile(
                    depth=1,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                    reliability=ReliabilityPolicy.RELIABLE,
                )
                publisher = self.rclpy_node.create_publisher(
                    DisplayTrajectory, "/beambot/preview_trajectory", qos
                )
                self.rclpy_node._preview_trajectory_pub = publisher

            msg = DisplayTrajectory()
            msg.model_id = ""

            start_state = RobotState()
            for sub in sol_msg.sub_trajectory:
                jt = sub.trajectory.joint_trajectory
                if jt.joint_names and jt.points:
                    js = JointState()
                    js.name = list(jt.joint_names)
                    js.position = list(jt.points[0].positions)
                    start_state.joint_state = js
                    break
            msg.trajectory_start = start_state

            for sub in sol_msg.sub_trajectory:
                rt = RobotTrajectory()
                rt.joint_trajectory = sub.trajectory.joint_trajectory
                rt.multi_dof_joint_trajectory = sub.trajectory.multi_dof_joint_trajectory
                msg.trajectory.append(rt)

            publisher.publish(msg)
            self.logger.info(
                f"Published preview for '{task_name}' "
                f"({len(msg.trajectory)} segment(s))"
            )
        except Exception as e:
            self.logger.warning(f"Failed to publish preview trajectory: {e}")

    def parse_poses(self, poses_json: str) -> dict[str, Any] | None:
        """Decode poses JSON; return {} for empty input or None for invalid JSON."""
        if not poses_json:
            return {}
        try:
            return json.loads(poses_json)
        except json.JSONDecodeError as e:
            self.logger.error(f"Failed to parse poses_json: {e}")
            return None

    def get_joint_pose(
        self, poses: dict[str, Any], pose_key: str
    ) -> list[float] | None:
        """Return a named six-element pose list, or None."""
        if pose_key not in poses:
            self.logger.error(f"Pose '{pose_key}' not found in poses_json")
            return None

        joint_pose = poses[pose_key]
        if not isinstance(joint_pose, list) or len(joint_pose) != 6:
            self.logger.error(
                f"'{pose_key}' must be array of 6 joint angles, "
                f"got {type(joint_pose).__name__}"
            )
            return None

        return joint_pose

    def make_move_to_named_stage(
        self,
        label: str,
        pose_key: str,
        poses: dict[str, Any],
        planner=None,
        constraints: Constraints | None = None
    ):
        """Build a named joint-pose stage, or None if the pose list is invalid."""
        joint_pose = self.get_joint_pose(poses, pose_key)
        if joint_pose is None:
            return None

        joint_goal = joints_from_degrees(joint_pose)
        self._pin_goal_joints = joint_goal

        if planner is not None:
            planners = [(lambda: planner, None, None)]
        else:
            planners = [
                (lambda: self.make_pilz_planner("PTP"), "Pilz PTP", None),
                (self.make_pipeline_planner, self._pipeline.upper(), self._ompl_timeout),
            ]

        fb = core.Fallbacks(label) if len(planners) > 1 else None

        for planner_fn, suffix, timeout in planners:
            stage_name = f"{label} [{suffix}]" if suffix is not None else label
            stage = stages.MoveTo(stage_name, planner_fn())
            stage.group = self.arm_group
            self._set_ik_frame(stage)
            stage.setGoal(joint_goal)
            apply_constraints(stage, constraints)
            if timeout is not None:
                stage.timeout = timeout
            if fb is None:
                return stage
            fb.add(stage)

        return fb

    def make_gripper_stage(
        self,
        label: str,
        planner,
        gripper_group: str,
        state_name: str
    ) -> stages.MoveTo | None:
        """Create a gripper stage, or return None if its group or state is empty."""
        if not gripper_group or not state_name:
            self.logger.info(f"No gripper group or state for '{label}' - skipping")
            return None

        stage = stages.MoveTo(label, planner)
        stage.group = gripper_group
        stage.setGoal(state_name)
        return stage

    def compute_deterministic_ik(
        self, approach: PoseStamped, ik_frame: str
    ) -> dict[str, float] | None:
        """Solve IK using a rounded live-joint seed; return joint radians or None."""
        js_holder = [None]
        js_event = threading.Event()

        def _on_js(msg):
            js_holder[0] = msg
            js_event.set()

        js_sub = self.rclpy_node.create_subscription(JointState, '/joint_states', _on_js, 10)
        js_event.wait(timeout=1.0)
        self.rclpy_node.destroy_subscription(js_sub)

        if not js_holder[0]:
            self.logger.warning("No joint_states for IK seed")
            return None

        # A per-call subscription avoids a 500 Hz Python callback in every stage
        # server. Round the seed to 0.01 degrees to reduce variation between requests.
        quantized = []
        for p in js_holder[0].position:
            deg = math.degrees(p)
            deg_q = round(deg, 2)
            quantized.append(math.radians(deg_q))

        ik_client = None
        try:
            ik_client = self.rclpy_node.create_client(GetPositionIK, '/compute_ik')
            if not ik_client.wait_for_service(timeout_sec=2.0):
                self.logger.warning("/compute_ik service not available")
                return None

            req = GetPositionIK.Request()
            req.ik_request.group_name = self.arm_group
            req.ik_request.ik_link_name = ik_frame
            req.ik_request.robot_state.joint_state.name = list(js_holder[0].name)
            req.ik_request.robot_state.joint_state.position = quantized
            req.ik_request.pose_stamped = approach
            req.ik_request.timeout.sec = 1

            # Blocks until another executor thread delivers the reply (or 5 s).
            result = ik_client.call(req, timeout_sec=5.0)
            if result is None:
                self.logger.warning("/compute_ik timed out")
                return None

            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                joint_goal = {
                    name: pos
                    for name, pos in zip(result.solution.joint_state.name,
                                         result.solution.joint_state.position)
                    if name in DEFAULT_JOINT_NAMES
                }
                if len(joint_goal) == len(DEFAULT_JOINT_NAMES):
                    return joint_goal

            self.logger.warning(f"compute_ik failed: error_code={result.error_code.val}")
            return None

        except Exception as e:
            self.logger.warning(f"Deterministic IK error: {e}")
            return None
        finally:
            if ik_client is not None:
                self.rclpy_node.destroy_client(ik_client)
