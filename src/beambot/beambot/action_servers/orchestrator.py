#!/usr/bin/env python3
"""Coordinate task scripts across MoveIt action servers."""

import json
import threading
import time
from dataclasses import dataclass
from typing import Any

from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.action.server import ServerGoalHandle, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from beambot.action_servers.base_action_server import run_server
from beambot_interfaces.action import (
    BeambotExecution,
    MoveToAction,
    EndEffectorAction,
    PickSampleAction,
    PlaceSampleAction,
    VisionScanAction,
    VisionTaskAction,
    PipettorAction,
)
from std_srvs.srv import Trigger
from std_msgs.msg import ColorRGBA, String

from beambot.utils.moveit_lifecycle_manager import MoveItLifecycleManager
# from beambot.utils.vacuum_monitor import VacuumMonitor
from beambot.utils.trajectory_cache import TrajectoryCache
from beambot.utils.tool_exchange_manager import ToolExchangeManager
from beambot.motion.move_to_task import MoveToTask
from beambot.motion.end_effector_task import EndEffectorTask
from beambot.utils.task_batching import group_into_batches
from beambot.motion.task_builder import wait_for_future
from beambot.utils.task_parser import parse_task_script
from beambot.utils.vision_goals import (
    VISION_TASK_TYPES,
    build_vision_goal,
    flatten_scan_positions,
    resolve_vision_step,
)
from beambot.config_loader import (
    gripper_tip_frame,
    load_beamline_config,
    resolve_beamline_path,
)


@dataclass
class _GoalRun:
    """Per-goal state shared by the batch helpers."""

    handle: ServerGoalHandle
    result: BeambotExecution.Result
    feedback: BeambotExecution.Feedback
    total: int
    poses_json: str
    dry_run: bool
    goal_key: str
    cache_eligible: bool
    cached_trajectory: Any = None  # Serialized MTC solution, or None.
    completed: int = 0


class BeambotOrchestratorServer(Node):
    """Coordinate and execute multi-step robot tasks."""

    # Defaults are overridable through timeout.<action> ROS parameters.
    DEFAULT_TIMEOUTS = {
        "moveto": 120.0,
        "end_effector": 30.0,
        "tool_exchange": 180.0,
        "vision_moveto": 60.0,
        "vision_task": 60.0,
        "vision_scan": 180.0,
        "pick_sample": 180.0,
        "place_sample": 180.0,
        "pipettor": 60.0,
    }

    def __init__(self):
        super().__init__("beambot_orchestrator")

        # Execution state.
        self._executing = False
        self._faulted = False
        # Lock guards check-and-set; single-writer flags are read lock-free.
        self._lock = threading.Lock()
        self._current_gripper = "unknown"

        # Pause control.
        self._pause_requested = False
        self._is_paused = False
        self._pause_event = threading.Event()
        self._pause_event.set()

        # Action results.
        self._last_error = ""  # Error from last failed action/batch
        self._last_result = None  # Full result from last _send_and_wait
        self._last_detected_position = None  # [x, y, z] from detect_only vision
        self._last_detected_orientation = None  # [x, y, z, w] from detect_only vision

        # Cache serialized plans by start state, goal, gripper, and model revision.
        self._trajectory_cache = TrajectoryCache(self.get_logger())
        # Live MTC Tasks become invalid after MoveIt relaunches.
        self._last_planned_sol_msg = None

        # Runtime parameters.
        self.declare_parameter("use_mock_hardware", False)
        self.declare_parameter("enable_joystick", False)
        self.declare_parameter("enable_batching", True)
        # Empty cup_profile uses the beamline default.
        self.declare_parameter("cup_profile", "")
        self._use_mock_hardware = self.get_parameter("use_mock_hardware").value
        self._enable_batching = self.get_parameter("enable_batching").value

        # Beamline configuration from BEAMBOT_BEAMLINE_CONFIG.
        config, config_file = load_beamline_config()
        self._poses_file = resolve_beamline_path(
            config.get("poses_file", ""), config_file
        )

        self._grippers = config["grippers"]
        self._vision_targets = config.get("vision_targets", {})
        self._robot_ip = config["robot"]["ip"]
        self._arm_group = config["robot"].get("arm_group", "ur_arm")
        self.get_logger().info(
            f"Loaded beamline: {config['beamline']} (robot: {self._robot_ip})"
        )
        if self._use_mock_hardware:
            self.get_logger().info("Using FAKE HARDWARE (simulation mode)")
        if not self._enable_batching:
            self.get_logger().info(
                "Batching DISABLED - each task executes via action server"
            )

        # Action timeouts.
        self._timeouts = {}
        for action_type, default_timeout in self.DEFAULT_TIMEOUTS.items():
            param_name = f"timeout.{action_type}"
            self.declare_parameter(param_name, default_timeout)
            self._timeouts[action_type] = self.get_parameter(param_name).value

        self.get_logger().info(f"Timeouts configured: {self._timeouts}")

        # Callback scheduling and MoveIt lifecycle.
        self._callback_group = ReentrantCallbackGroup()

        self._moveit_manager = MoveItLifecycleManager(
            self,
            self._grippers,
            self._robot_ip,
            self._callback_group,
            use_mock_hardware=self._use_mock_hardware,
            enable_joystick=self.get_parameter("enable_joystick").value,
        )

        # Action server and child-action clients.
        self._action_server = ActionServer(
            self,
            BeambotExecution,
            "beambot_execution",
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._callback_group,
        )

        def make_action_client(action_type, name):
            return ActionClient(
                self, action_type, name, callback_group=self._callback_group
            )

        self._moveto_client = make_action_client(MoveToAction, "beambot_moveto")
        self._endeffector_client = make_action_client(
            EndEffectorAction, "beambot_endeffector"
        )
        self._vision_task_client = make_action_client(
            VisionTaskAction, "beambot_vision_task"
        )
        self._vision_scan_client = make_action_client(
            VisionScanAction, "beambot_vision_scan"
        )
        self._pick_sample_client = make_action_client(
            PickSampleAction, "beambot_pick_sample"
        )
        self._place_sample_client = make_action_client(
            PlaceSampleAction, "beambot_place_sample"
        )
        self._pipettor_client = make_action_client(PipettorAction, "beambot_pipettor")

        # Tool exchange.
        self._tool_exchange_manager = ToolExchangeManager(
            self,
            self._grippers,
            self._moveit_manager,
            self._callback_group,
            use_mock_hardware=self._use_mock_hardware,
            send_and_wait=self._send_and_wait,
            on_gripper_changed=self._set_current_gripper,
        )

        # Pause/resume services.
        self._pause_service = self.create_service(
            Trigger,
            "beambot/pause",
            self._pause_callback,
            callback_group=self._callback_group,
        )
        self._resume_service = self.create_service(
            Trigger,
            "beambot/resume",
            self._resume_callback,
            callback_group=self._callback_group,
        )

        # State publishers, latched for late subscribers.
        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self._state_publisher = self.create_publisher(
            String, "beambot/execution_state", latched_qos
        )
        self._publish_state("IDLE")

        self._gripper_publisher = self.create_publisher(
            String, "beambot/current_gripper", latched_qos
        )
        self._publish_gripper(self._current_gripper)

        # Vacuum monitoring is disabled.
        # self._vacuum = VacuumMonitor(self, self._grippers, self._callback_group)

        self.get_logger().info(
            "MTC Orchestrator (Python) started on 'beambot_execution'"
        )
        self.get_logger().info(
            "Pause/Resume services available: beambot/pause, beambot/resume"
        )

    def destroy_node(self):
        """Stop MoveIt while rclpy is still up; the launch runs in its own session."""
        # No next launch follows shutdown, so skip the discovery drain.
        self._moveit_manager.kill_current_process(drain=False)
        super().destroy_node()

    # Goal admission and pause/cancel control.

    def _goal_callback(self, goal_request) -> GoalResponse:
        """Handle incoming goal requests."""
        with self._lock:
            if self._faulted:
                self.get_logger().error("Goal rejected: orchestrator is faulted")
                return GoalResponse.REJECT
            if self._executing:
                self.get_logger().warning("Goal rejected: another task is executing")
                return GoalResponse.REJECT
            self._executing = True
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle: ServerGoalHandle) -> CancelResponse:
        """Handle cancel requests."""
        self.get_logger().info(
            "Cancel request received - will stop after active execution unit"
        )
        self._publish_state("CANCELING")
        return CancelResponse.ACCEPT

    def _pause_callback(self, request, response):
        """Request a pause after the active batch."""
        with self._lock:
            if not self._executing:
                response.success = False
                response.message = "No task is currently executing"
                return response

            if self._is_paused:
                response.success = False
                response.message = "Already paused"
                return response

            if self._pause_requested:
                response.success = False
                response.message = (
                    "Pause already requested, waiting for current task to complete"
                )
                return response

            self._pause_requested = True
            self._publish_state("COMPLETING_TASK")
            response.success = True
            response.message = (
                "Pause requested - will pause after current task completes"
            )

        self.get_logger().info("Pause requested - will pause after current task")
        return response

    def _resume_callback(self, request, response):
        """Resume paused execution."""
        with self._lock:
            if not self._is_paused:
                response.success = False
                response.message = "Not currently paused"
                return response

            self._is_paused = False
            self._pause_event.set()  # Unblock the waiting thread
            response.success = True
            response.message = "Resuming execution"

        self.get_logger().info("Resume requested - continuing execution")
        return response

    def _handle_pause(
        self,
        feedback: BeambotExecution.Feedback,
        goal_handle: ServerGoalHandle,
        completed_steps: int,
        total_steps: int,
        active_step: int | None = None,
    ):
        """Wait for resume or cancellation."""
        with self._lock:
            self._pause_requested = False
            self._is_paused = True
            self._pause_event.clear()  # Block the event

        self._publish_state("PAUSED")
        if active_step is None:
            self.get_logger().info(f"Paused after step {completed_steps}/{total_steps}")
        else:
            self.get_logger().info(
                f"Paused at step {active_step}/{total_steps}; "
                f"completed {completed_steps}/{total_steps}"
            )

        if active_step is not None:
            feedback.current_step = active_step
        feedback.progress_percentage = (
            completed_steps / total_steps * 100.0 if total_steps else 0.0
        )
        feedback.status_message = (
            f"PAUSED - completed {completed_steps}/{total_steps}, waiting for resume"
        )
        goal_handle.publish_feedback(feedback)

        # Poll for cancellation while paused.
        while not self._pause_event.wait(timeout=0.5):
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Cancel received while paused")
                with self._lock:
                    self._is_paused = False
                    self._pause_event.set()
                return

            goal_handle.publish_feedback(feedback)

        self.get_logger().info("Execution resumed")
        self._publish_state("RUNNING")

    # Main execution flow.

    def _execute_callback(self, goal_handle: ServerGoalHandle):
        """Execute the orchestration goal."""
        try:
            return self._execute(goal_handle)
        except Exception:
            self._faulted = True
            raise
        finally:
            with self._lock:
                if not self._faulted:
                    self._executing = False
            self._publish_state("FAULTED" if self._faulted else "IDLE")

    def _execute(self, goal_handle: ServerGoalHandle) -> BeambotExecution.Result:
        """Main execution logic."""
        self.get_logger().info("Executing orchestration goal")

        # self._vacuum.reset()
        self._last_detected_position = None
        self._last_detected_orientation = None

        result = BeambotExecution.Result()
        feedback = BeambotExecution.Feedback()

        dry_run = goal_handle.request.dry_run
        try:
            parsed = parse_task_script(
                goal_handle.request.full_json,
                dry_run=dry_run,
                grippers=self._grippers,
                poses_file=self._poses_file,
                vision_targets=self._vision_targets,
                on_info=self.get_logger().info,
                on_warning=self.get_logger().warning,
            )
        except ValueError as error:
            result.error_message = str(error)
            goal_handle.abort()
            return result

        start_gripper, tasks, poses_json = parsed
        task_count = len(tasks)
        result.total_steps = task_count
        if dry_run:
            self.get_logger().info(
                f"DRY-RUN preview enabled — planning {task_count} step(s) "
                f"without moving the robot"
            )

        self._set_current_gripper(start_gripper)

        # Avoid mutating shared beamline config; empty override uses YAML default.
        self._moveit_manager.cup_override = (
            self.get_parameter("cup_profile").value or ""
        )

        # Start MoveIt with the requested gripper.
        self._update_feedback(
            feedback, goal_handle, 0, task_count, "Initializing MoveIt"
        )

        if not self._moveit_manager.launch_moveit_with_gripper(start_gripper):
            result.error_message = "Failed to initialize MoveIt stack"
            goal_handle.abort()
            return result

        # Pose edits must invalidate cached trajectories.
        goal_key = TrajectoryCache.compute_key(
            json.dumps({"tasks": tasks, "poses": json.loads(poses_json)}),
            self._moveit_manager.model_revision,
            self._moveit_manager.current_arm_joints(),
        )

        self._publish_state("RUNNING")

        # Group tasks into batches; previews use the same boundaries.
        batches = group_into_batches(
            tasks,
            enabled=self._enable_batching,
        )
        self.get_logger().info(
            f"Grouped {task_count} tasks into {len(batches)} batches"
        )

        run = _GoalRun(
            handle=goal_handle,
            result=result,
            feedback=feedback,
            total=task_count,
            poses_json=poses_json,
            dry_run=dry_run,
            goal_key=goal_key,
            # Cache only one-batch goals; one key cannot represent multiple starts.
            cache_eligible=len(batches) == 1 and batches[0][0] == "batched",
        )
        if run.cache_eligible and not dry_run:
            run.cached_trajectory = self._trajectory_cache.get(goal_key)
            if run.cached_trajectory is not None:
                self.get_logger().info(
                    "Trajectory cache hit — replaying stored plan without re-planning"
                )

        for batch_type, batch_tasks in batches:
            stop = self._check_before_batch(run)
            if stop is None:
                stop = (
                    self._run_mtc_batch(run, batch_tasks)
                    if batch_type == "batched"
                    else self._run_single_task(run, batch_tasks[0])
                )
            if stop is not None:
                return stop
            run.completed += len(batch_tasks)
            result.completed_steps = run.completed

        # Final vacuum-loss check remains disabled.
        # if not dry_run:
        #     vacuum_error = self._vacuum.check_lost()
        #     if vacuum_error:
        #         result.error_message = f"Final step aborted: {vacuum_error}"
        #         result.completed_steps = run.completed
        #         goal_handle.abort()
        #         self._publish_state("IDLE")
        #         return result

        result.success = True

        if self._last_detected_position is not None:
            result.detected_position = self._last_detected_position
        if self._last_detected_orientation is not None:
            result.detected_orientation = self._last_detected_orientation

        self._update_feedback(feedback, goal_handle, task_count, task_count, "")
        goal_handle.succeed()
        self.get_logger().info("Orchestration goal completed successfully")
        return result

    # Batch execution. Each returns a terminal Result to stop, or None to continue.

    def _check_before_batch(self, run: _GoalRun) -> BeambotExecution.Result | None:
        """Handle cancel, pause, and MoveIt liveness before dispatch."""
        if (stop := self._stop_if_cancel_requested(run)) is not None:
            return stop

        if self._pause_requested:
            self._handle_pause(run.feedback, run.handle, run.completed, run.total)
            if run.handle.is_cancel_requested:
                self.get_logger().warning(
                    f"Task cancelled while paused at step {run.completed}/{run.total}"
                )
                return self._finish_canceled(run, "Task was cancelled while paused")

        # Vacuum-loss abort checks remain disabled.
        # if not run.dry_run:
        #     vacuum_error = self._vacuum.check_lost()
        #     if vacuum_error:
        #         self._publish_state("IDLE")
        #         return self._finish_aborted(
        #             run, f"Step {run.completed + 1} aborted: {vacuum_error}"
        #         )

        if not self._moveit_manager.is_moveit_alive():
            exit_info = self._moveit_manager.get_moveit_exit_info()
            message = f"MoveIt crashed before step {run.completed + 1}: {exit_info}"
            self.get_logger().error(message)
            return self._finish_aborted(run, message)
        return None

    def _run_mtc_batch(
        self, run: _GoalRun, batch_tasks: list[dict[str, Any]]
    ) -> BeambotExecution.Result | None:
        """Execute or preview one batch; launch owns controller activation."""
        step = run.completed + 1
        batch_desc = ", ".join(t.get("task_type", "?") for t in batch_tasks)
        self._update_feedback(
            run.feedback,
            run.handle,
            step,
            run.total,
            f"batch[{len(batch_tasks)}]: {batch_desc}",
        )

        self._last_planned_sol_msg = None
        ok = self._execute_batch(
            batch_tasks,
            run.poses_json,
            dry_run=run.dry_run,
            cached_trajectory=run.cached_trajectory,
        )
        if not ok:
            self._mark_faulted_if_unsafe()
            return self._finish_aborted(run, f"Batch failed at step {step}: {self._last_error}")

        # Cache eligible fresh plans; replays produce no new solution.
        if run.cache_eligible and self._last_planned_sol_msg is not None:
            self._trajectory_cache.store(run.goal_key, self._last_planned_sol_msg)
            self.get_logger().info("Trajectory cached for replay")
            self._last_planned_sol_msg = None

        # Vacuum bookkeeping is disabled.
        # if not run.dry_run:
        #     self._vacuum.update_after_tasks(batch_tasks, self._current_gripper)
        return self._stop_if_cancel_requested(run, finished=len(batch_tasks))

    def _run_single_task(
        self, run: _GoalRun, task: dict[str, Any]
    ) -> BeambotExecution.Result | None:
        """Dispatch one non-batched task to its action server."""
        task_type = task["task_type"]
        self._update_feedback(
            run.feedback, run.handle, run.completed + 1, run.total, task_type
        )

        # With batching disabled, route dry runs through planning to avoid
        # hardware motion. Single-task dry runs are not cached.
        if run.dry_run:
            self._last_planned_sol_msg = None
            if not self._execute_batch([task], run.poses_json, dry_run=True):
                return self._finish_aborted(
                    run, f"{task_type} preview failed: {self._last_error}"
                )
            if run.handle.is_cancel_requested:
                return self._finish_canceled(run, "Task was canceled", run.completed + 1)
            return None

        if task_type == "pause":
            return self._run_pause_task(run)

        if not self._execute_step(task_type, task, run.poses_json):
            self._mark_faulted_if_unsafe()
            return self._finish_aborted(run, f"{task_type} failed: {self._last_error}")

        # self._vacuum.update_after_tasks([task], self._current_gripper)
        return self._stop_if_cancel_requested(run, finished=1)

    def _run_pause_task(self, run: _GoalRun) -> BeambotExecution.Result | None:
        """Hold at a scripted pause step until resume or cancel."""
        step = run.completed + 1
        self.get_logger().info(
            f"Pause step {step}/{run.total}: entering paused state"
        )
        self._handle_pause(
            run.feedback, run.handle, run.completed, run.total, active_step=step
        )

        if run.handle.is_cancel_requested:
            self.get_logger().warning(
                f"Task cancelled while paused at step {step}/{run.total}"
            )
            return self._finish_canceled(run, "Task was cancelled while paused")

        if step < run.total:
            self.get_logger().info(
                f"Resumed from pause, continuing with step {step + 1}/{run.total}"
            )
        else:
            self.get_logger().info(
                f"Resumed from final pause step {step}/{run.total}"
            )
        return None

    # Goal termination helpers.

    def _finish_aborted(
        self, run: _GoalRun, message: str, completed: int | None = None
    ) -> BeambotExecution.Result:
        run.result.error_message = message
        run.result.completed_steps = run.completed if completed is None else completed
        run.handle.abort()
        return run.result

    def _finish_canceled(
        self, run: _GoalRun, message: str, completed: int | None = None
    ) -> BeambotExecution.Result:
        run.result.error_message = message
        run.result.completed_steps = run.completed if completed is None else completed
        run.handle.canceled()
        return run.result

    def _stop_if_cancel_requested(
        self, run: _GoalRun, finished: int = 0
    ) -> BeambotExecution.Result | None:
        """Cancel after `finished` more steps if the client requested it."""
        if not run.handle.is_cancel_requested:
            return None
        done = run.completed + finished
        self.get_logger().warning(f"Task cancelled after step {done}/{run.total}")
        return self._finish_canceled(run, "Task was canceled", done)

    def _mark_faulted_if_unsafe(self):
        """Fault when a failed batch may have left motion active."""
        if self._last_error.startswith(("TIMEOUT:", "REPLAY_TIMEOUT:")):
            self._faulted = True

    def _execute_step(
        self, task_type: str, step: dict[str, Any], poses_json: str
    ) -> bool:
        """Execute a single step by dispatching to the appropriate action server."""
        # UR launch infrastructure owns controller activation.

        self.get_logger().info(f"Executing step: {task_type}")

        if task_type in VISION_TASK_TYPES:
            return self._call_vision_task(task_type, step, poses_json)

        call_by_task_type = {
            "moveto": self._call_moveto,
            "end_effector": self._call_endeffector,
            "tool_exchange": self._handle_tool_exchange,
            "vision_scan": self._call_vision_scan,
            "pipettor": self._call_pipettor,
        }
        call = call_by_task_type.get(task_type)
        if call is None:
            self._last_error = f"Unknown task type: '{task_type}'"
            self.get_logger().error(self._last_error)
            return False
        return call(step, poses_json)

    # Batch planning and shared goal builders.

    def _execute_batch(
        self,
        batch_tasks: list[dict[str, Any]],
        poses_json: str,
        dry_run: bool = False,
        cached_trajectory=None,  # Serialized MTC solution to replay, or None.
    ) -> bool:
        """Plan or execute one batch; cached plans bypass planning."""
        if not batch_tasks:
            return True

        task_types = [t.get("task_type", "?") for t in batch_tasks]
        self.get_logger().info(
            f"{'Replaying cached plan for' if cached_trajectory else 'Executing'} "
            f"batch of {len(batch_tasks)} tasks: {task_types}"
        )

        # Task builders share the module-level MTC node.
        moveto_task = MoveToTask(self, self._arm_group)
        try:
            endeffector_task = EndEffectorTask(self, self._arm_group)

            # Replan after terminal replay failures. A timeout may leave motion active,
            # so never dispatch a second trajectory after REPLAY_TIMEOUT.
            if cached_trajectory is not None:
                error = moveto_task.execute_solution_msg(
                    cached_trajectory, is_replay=True
                )
                if error is None:
                    return True
                if error.startswith("REPLAY_TIMEOUT"):
                    self._last_error = error
                    return False
                self.get_logger().warning(
                    f"Cached replay failed ({error}); re-planning fresh"
                )
            task = moveto_task.create_task_template(f"Batch ({len(batch_tasks)} tasks)")

            for i, batch_task in enumerate(batch_tasks):
                task_type = batch_task.get("task_type", "")
                error = None

                if task_type == "moveto":
                    goal = self._create_moveto_goal(batch_task, poses_json)
                    error = moveto_task.add_to_task(task, goal)

                elif task_type == "end_effector":
                    goal = self._create_endeffector_goal(batch_task)
                    error = endeffector_task.add_to_task(task, goal)

                else:
                    self._last_error = f"Unknown batchable type: {task_type}"
                    self.get_logger().error(self._last_error)
                    return False

                if error is not None:
                    self._last_error = f"Batch task {i} ({task_type}): {error}"
                    self.get_logger().error(self._last_error)
                    return False

            if dry_run:
                error = moveto_task.init_and_plan(task, dry_run=True)
                if error is not None:
                    self._last_error = error
                    return False
                # Cache the serialized solution; live Tasks cannot survive relaunches.
                self._last_planned_sol_msg = moveto_task.last_sol_msg
                return True

            error = moveto_task.load_plan_execute(task)
            if error is not None:
                self._last_error = error
                return False
            # Save the fresh solution for later replay.
            self._last_planned_sol_msg = moveto_task.last_sol_msg
            return True
        finally:
            moveto_task.close()

    def _create_moveto_goal(
        self, step: dict[str, Any], poses_json: str
    ) -> MoveToAction.Goal:
        """Create a MoveToAction.Goal from task dict."""
        goal = MoveToAction.Goal()
        goal.target = step.get("target", "")
        goal.planning_type = step.get("planning_type", "")
        goal.direction = step.get("direction", "")
        goal.distance = float(step.get("distance", 0.0))
        goal.cartesian_target = [float(v) for v in step.get("cartesian_target", [])]
        goal.frame_id = step.get("frame_id", "base_link")
        goal.poses_json = poses_json
        goal.constraints_json = (
            json.dumps(step["constraints"]) if "constraints" in step else ""
        )
        return goal

    def _create_endeffector_goal(self, step: dict[str, Any]) -> EndEffectorAction.Goal:
        """Create an EndEffectorAction.Goal from task dict."""
        gripper_type = step.get("end_effector_type", self._current_gripper)
        gripper_config = self._grippers.get(gripper_type, {})

        goal = EndEffectorAction.Goal()
        goal.gripper_group = gripper_config.get("gripper_group", "")
        goal.end_effector_action = step.get("end_effector_action", "")
        return goal

    # Child-action communication and motion/tool handlers.

    def _send_and_wait(
        self, client: ActionClient, goal, name: str, timeout: float
    ) -> bool:
        """Send a child goal and poll without re-entering the executor."""
        self._last_error = ""

        if hasattr(goal, "robot_model_revision"):
            revision = self._moveit_manager.model_revision
            if not revision:
                self._last_error = f"Cannot send {name}: no active RobotModel revision"
                self.get_logger().error(self._last_error)
                return False
            goal.robot_model_revision = revision

        if not client.wait_for_server(timeout_sec=5.0):
            self._last_error = f"TIMEOUT: {name} action server unavailable (waited 5s)"
            self.get_logger().error(self._last_error)
            return False

        send_future = client.send_goal_async(goal)
        if not wait_for_future(send_future, timeout=10.0):
            self._faulted = True
            self._last_error = f"TIMEOUT: {name} goal acceptance timed out (10s)"
            self.get_logger().error(self._last_error)
            return False

        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self._last_error = f"{name} goal was rejected by action server"
            self.get_logger().error(self._last_error)
            return False

        result_future = goal_handle.get_result_async()
        if not wait_for_future(result_future, timeout=timeout):
            self._faulted = True
            self._last_error = f"TIMEOUT: {name} timed out after {timeout}s"
            self.get_logger().error(self._last_error)
            goal_handle.cancel_goal_async()
            return False

        result = result_future.result()
        self._last_result = result.result
        if not result.result.success:
            self._last_error = (
                getattr(result.result, "error_message", "")
                or f"{name} failed (no details)"
            )
            self.get_logger().error(f"{name} failed: {self._last_error}")
        return result.result.success

    def _call_moveto(self, step: dict[str, Any], poses_json: str) -> bool:
        """Call the MoveTo action server."""
        goal = self._create_moveto_goal(step, poses_json)
        return self._send_and_wait(
            self._moveto_client, goal, "moveto", self._timeouts["moveto"]
        )

    def _call_endeffector(self, step: dict[str, Any], poses_json: str) -> bool:
        """Call the EndEffector action server."""
        goal = self._create_endeffector_goal(step)
        return self._send_and_wait(
            self._endeffector_client,
            goal,
            "end_effector",
            self._timeouts["end_effector"],
        )

    def _handle_tool_exchange(self, step: dict[str, Any], poses_json: str) -> bool:
        success = self._tool_exchange_manager.exchange(
            step,
            poses_json,
            self._current_gripper,
            self._timeouts["tool_exchange"],
        )
        if not success and self._tool_exchange_manager.last_error:
            self._last_error = self._tool_exchange_manager.last_error
        return success

    def _call_pipettor(self, step: dict[str, Any], poses_json: str) -> bool:
        """Call the Pipettor action server."""
        goal = PipettorAction.Goal()
        goal.operation = step.get("operation", "")
        goal.volume_pct = float(step.get("volume_pct", 0.0))
        goal.poses_json = poses_json

        if "led_color" in step:
            led = step["led_color"]
            goal.led_color = ColorRGBA()
            goal.led_color.r = float(led.get("r", 0.0))
            goal.led_color.g = float(led.get("g", 0.0))
            goal.led_color.b = float(led.get("b", 0.0))
            goal.led_color.a = 1.0

        return self._send_and_wait(
            self._pipettor_client, goal, "pipettor", self._timeouts["pipettor"]
        )

    # Vision and sample handlers.

    def _call_vision_scan(self, step: dict[str, Any], poses_json: str) -> bool:
        """Scan configured poses and cache detected marker poses."""
        goal = VisionScanAction.Goal()
        goal.scans_per_position = int(step.get("scans_per_position", 3))
        goal.timeout = float(step.get("timeout", 10.0))
        goal.poses_json = poses_json

        scan_position_keys = step.get("scan_positions", [])
        if not scan_position_keys:
            self._last_error = "vision_scan requires 'scan_positions' list"
            self.get_logger().error(self._last_error)
            return False

        scan_positions_flat, valid_positions = flatten_scan_positions(
            json.loads(poses_json), scan_position_keys, self.get_logger().warning
        )

        if valid_positions == 0:
            self._last_error = "No valid scan positions found"
            self.get_logger().error(self._last_error)
            return False

        goal.scan_positions_flat = scan_positions_flat
        goal.num_scan_positions = valid_positions

        self.get_logger().info(
            f"VisionScan: {valid_positions} positions × "
            f"{goal.scans_per_position} scans per position"
        )

        return self._send_and_wait(
            self._vision_scan_client, goal, "vision_scan", self._timeouts["vision_scan"]
        )

    def _call_vision_task(
        self, task_type: str, step: dict[str, Any], poses_json: str
    ) -> bool:
        """Dispatch vision tasks and retain detected poses."""
        # Non-vision pick/place stays on the sample server.
        if task_type in ("pick_sample", "place_sample") and not step.get(
            "use_vision", True
        ):
            return self._call_sample_hardcoded(task_type, step, poses_json)

        cfg = resolve_vision_step(task_type, step)

        # Cap pre-motion settling at 10 seconds.
        settle_time = min(float(cfg.get("settle_time", 1.0)), 10.0)
        if settle_time > 0:
            self.get_logger().info(f"Waiting {settle_time:.1f}s for robot to settle...")
            time.sleep(settle_time)

        goal = build_vision_goal(
            cfg,
            poses_json,
            grippers=self._grippers,
            current_gripper=self._current_gripper,
            ik_frame=self._gripper_ik_frame(),
            on_info=self.get_logger().info,
            on_warning=self.get_logger().warning,
        )
        success = self._send_and_wait(
            self._vision_task_client, goal, task_type, self._timeouts["vision_task"]
        )

        if success and self._last_result is not None:
            pos = list(getattr(self._last_result, "detected_position", []))
            ori = list(getattr(self._last_result, "detected_orientation", []))
            if len(pos) == 3:
                self._last_detected_position = pos
                self._last_detected_orientation = ori if len(ori) == 4 else None
                self.get_logger().info(
                    f"Stored detected pose: [{pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}]"
                )

        # Vacuum watchdog bookkeeping is disabled.
        # watchdog = VISION_PRESETS.get(task_type, {}).get("watchdog", "")
        # if success and watchdog and self._current_gripper == "epick":
        #     if watchdog == "arm" and getattr(self._last_result, "vacuum_ok", True):
        #         self._vacuum.armed = True
        #         self._vacuum.lost = False
        #     elif watchdog == "disarm":
        #         self._vacuum.armed = False
        #         self._vacuum.lost = False

        return success

    def _gripper_ik_frame(self) -> str:
        """Return configured IK tip frame for the current gripper."""
        return gripper_tip_frame(self._current_gripper, default="flange")

    def _call_sample_hardcoded(
        self, task_type: str, step: dict[str, Any], poses_json: str
    ) -> bool:
        """Non-vision pick/place sample servers, joint-pose sequence."""
        if task_type == "pick_sample":
            action_type, client = PickSampleAction, self._pick_sample_client
        else:
            action_type, client = PlaceSampleAction, self._place_sample_client

        gripper_config = self._grippers.get(
            step.get("gripper", self._current_gripper), {}
        )
        goal = action_type.Goal()
        goal.use_vision = False
        goal.approach_pose = step.get("approach_pose", "")
        goal.target_pose = step.get("target_pose", "")
        goal.gripper_group = gripper_config.get("gripper_group", "")
        goal.gripper_states_json = json.dumps(gripper_config.get("states", {}))
        goal.poses_json = poses_json
        goal.constraints_json = (
            json.dumps(step["constraints"]) if "constraints" in step else ""
        )
        success = self._send_and_wait(
            client, goal, task_type, self._timeouts[task_type]
        )
        # if success and task_type == "place_sample" and self._current_gripper == "epick":
        #     self._vacuum.armed = False
        #     self._vacuum.lost = False
        return success

    # State and feedback publishing.

    def _publish_state(self, state: str):
        """Publish execution state."""
        msg = String()
        msg.data = state
        self._state_publisher.publish(msg)

    def _publish_gripper(self, gripper: str):
        """Publish current gripper to latched topic."""
        msg = String()
        msg.data = gripper
        self._gripper_publisher.publish(msg)

    def _set_current_gripper(self, gripper: str):
        self._current_gripper = gripper
        self._publish_gripper(gripper)

    def _update_feedback(
        self,
        feedback: BeambotExecution.Feedback,
        goal_handle: ServerGoalHandle,
        current_step: int,
        total_steps: int,
        task_type: str,
    ):
        """Publish progress feedback."""
        feedback.current_step = current_step
        feedback.current_action = task_type
        feedback.progress_percentage = (
            (current_step / total_steps) * 100.0 if total_steps > 0 else 0.0
        )
        feedback.status_message = (
            "Task completed" if not task_type else f"Executing: {task_type}"
        )
        feedback.current_gripper = self._current_gripper

        goal_handle.publish_feedback(feedback)


def main(args=None):
    """Run the MTC Orchestrator server."""
    run_server(BeambotOrchestratorServer, args)


if __name__ == "__main__":
    main()
