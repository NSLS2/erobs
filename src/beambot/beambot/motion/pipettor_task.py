"""Pipettor driver commands without robot-motion planning."""

from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import ColorRGBA

from pipette_driver.action import PipettorOperation

from beambot.motion.task_builder import wait_for_future


class PipettorTask:
    PIPETTOR_ACTION = "pipettor_operation"
    DEFAULT_TIMEOUT = 60.0  # Seconds.

    def __init__(self, rclpy_node: Node):
        self.rclpy_node = rclpy_node
        self.logger = rclpy_node.get_logger()

        self._action_client = ActionClient(
            rclpy_node,
            PipettorOperation,
            self.PIPETTOR_ACTION
        )

        self.logger.info("PipettorTask initialized")

    def run(self, goal) -> str | None:
        """Execute a pipettor goal; return None on success or an error string."""
        if goal.operation == "SET_LED":
            op_desc = (
                f"SET_LED ({int(goal.led_color.r * 255)}, "
                f"{int(goal.led_color.g * 255)}, "
                f"{int(goal.led_color.b * 255)})"
            )
        else:
            op_desc = goal.operation

        self.logger.info(f"Pipettor: {op_desc}")

        return self._execute_pipettor_action(
            goal.operation,
            goal.volume_pct,
            goal.led_color
        )

    def _execute_pipettor_action(
        self,
        operation: str,
        volume_pct: float,
        led_color: ColorRGBA
    ) -> str | None:
        """Send a driver goal and wait for its result."""
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            return f"Pipettor action server '{self.PIPETTOR_ACTION}' not available (timeout 5s)"

        action_goal = PipettorOperation.Goal()
        action_goal.operation = operation
        action_goal.volume_pct = volume_pct  # Currently ignored by the driver.
        action_goal.led_color = led_color

        send_goal_future = self._action_client.send_goal_async(action_goal)
        if not wait_for_future(send_goal_future, timeout=5.0):
            return f"Pipettor goal send timeout for operation '{operation}'"

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            return f"Pipettor goal rejected for operation '{operation}'"

        result_future = goal_handle.get_result_async()
        if not wait_for_future(result_future, timeout=self.DEFAULT_TIMEOUT):
            self.logger.error("Pipettor operation timeout")
            goal_handle.cancel_goal_async()
            return f"TIMEOUT: Pipettor operation '{operation}' timed out after {self.DEFAULT_TIMEOUT}s"

        result = result_future.result()
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            return f"Pipettor action failed with status {result.status} for operation '{operation}'"

        if not result.result.success:
            return f"Pipettor operation '{operation}' failed: {result.result.message}"

        self.logger.info(f"Pipettor operation succeeded: {result.result.message}")
        return None
