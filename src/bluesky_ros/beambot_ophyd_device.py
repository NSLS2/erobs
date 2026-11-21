"""MTC Python Ophyd Device for Bluesky Integration

This module provides an Ophyd device wrapper for beambot orchestrator,
enabling Bluesky experiment orchestration with MTC robot tasks.
"""

from ophyd.status import DeviceStatus
from rclpy.node import Node
from rclpy.action import ActionClient
from rosidl_runtime_py.utilities import get_action
from bluesky.protocols import Movable
import rclpy


class ActionStatus(DeviceStatus):
    """Track the status of a ROS Action for Bluesky"""

    def __init__(self, device, **kwargs):
        super().__init__(device, **kwargs)

    def _handle_failure(self):
        self.device.cancel_goal()


class BeambotExecutionDevice(Node, Movable):
    """Ophyd device for MTC task execution via Bluesky"""

    def __init__(self, name="mtc_executor", **kwargs):
        super().__init__(name, **kwargs)

        # Ophyd attributes Bluesky expects on a Movable (merge_cycler reads .parent)
        self.name = name
        self.parent = None

        self.action_type = get_action('beambot_interfaces/BeambotExecution')

        self._action_client = ActionClient(self, self.action_type, 'beambot_execution')
        self._goal_handle = None
        self._bluesky_status = None
        self._send_goal_future = None
        self._get_result_future = None

    def construct_goal_message(self, json_path_or_string):
        """Construct goal from JSON file path or JSON string

        Args:
            json_path_or_string: Either a path to a .json file or a JSON string

        Returns:
            BeambotExecution.Goal with full_json populated
        """
        goal = self.action_type.Goal()

        # If it's a file path, read the file
        if isinstance(json_path_or_string, str) and json_path_or_string.endswith('.json'):
            with open(json_path_or_string, 'r') as f:
                goal.full_json = f.read()
        else:
            # Assume it's already a JSON string
            goal.full_json = json_path_or_string

        return goal

    def set(self, json_path_or_string):
        """Execute MTC task"""
        self._bluesky_status = ActionStatus(self)

        # Construct and send goal
        goal_msg = self.construct_goal_message(json_path_or_string)

        self.get_logger().info("Waiting for action server...")
        self._action_client.wait_for_server(timeout_sec=10.0)

        self.get_logger().info("Sending goal...")
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback
        )
        self._send_goal_future.add_done_callback(self._goal_response_callback)

        # Spin until complete
        while not self._bluesky_status.done:
            rclpy.spin_once(self, timeout_sec=0.1)

        return self._bluesky_status

    def _feedback_callback(self, feedback_msg):
        """Handle execution feedback"""
        fb = feedback_msg.feedback
        self.get_logger().info(
            f"Step {fb.current_step}: {fb.current_action} "
            f"({fb.progress_percentage:.1f}%) - {fb.status_message}"
        )

    def _goal_response_callback(self, future):
        """Handle goal acceptance/rejection"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            self._bluesky_status.set_exception(Exception("Goal rejected"))
            return

        self.get_logger().info("Goal accepted")
        self._goal_handle = goal_handle
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future):
        """Handle final result"""
        result = future.result()

        if result.status == 4:  # SUCCEEDED
            self.get_logger().info(
                f"Task completed: {result.result.completed_steps}/{result.result.total_steps} steps"
            )
            self._bluesky_status.set_finished()
        elif result.status == 5:  # ABORTED
            self.get_logger().error(f"Task aborted: {result.result.error_message}")
            self._bluesky_status.set_exception(Exception(result.result.error_message))
        elif result.status == 6:  # CANCELED
            self.get_logger().warning("Task canceled")
            self._bluesky_status.set_exception(Exception("Canceled"))
        else:
            self.get_logger().error(f"Unknown result status: {result.status}")
            self._bluesky_status.set_exception(Exception("Unknown status"))

    def cancel_goal(self):
        """Cancel the current goal"""
        if self._goal_handle is not None:
            self.get_logger().info("Canceling goal...")
            cancel_future = self._goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=5.0)
