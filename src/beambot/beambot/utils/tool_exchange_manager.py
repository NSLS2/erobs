"""Coordinate physical tool exchange and dependent runtime state."""

import time
from collections.abc import Callable, Mapping
from typing import Any

from rclpy.action import ActionClient
from rclpy.node import Node
from std_srvs.srv import Trigger

from beambot_interfaces.action import ToolExchangeAction


class ToolExchangeManager:
    """Run tool exchange, then synchronize MoveIt and vision state."""

    def __init__(
        self,
        node: Node,
        grippers: Mapping[str, Any],
        moveit_manager,
        callback_group,
        *,
        use_mock_hardware: bool,
        send_and_wait: Callable[..., bool],
        on_gripper_changed: Callable[[str], None],
    ):
        self._node = node
        self._logger = node.get_logger()
        self._grippers = grippers
        self._moveit_manager = moveit_manager
        self._callback_group = callback_group
        self._use_mock_hardware = use_mock_hardware
        self._send_and_wait = send_and_wait
        self._on_gripper_changed = on_gripper_changed
        self.last_error = ""

        self._action_client = ActionClient(
            node,
            ToolExchangeAction,
            "beambot_toolexchange",
            callback_group=callback_group,
        )
        # Only the vision task server detects the mounted gripper from TF;
        # the scan server's camera chain is gripper-independent.
        self._vision_task_reset_tf = node.create_client(
            Trigger, "beambot_vision_task_reset_tf", callback_group=callback_group
        )

    def exchange(
        self,
        step: dict[str, Any],
        poses_json: str,
        current_gripper: str,
        timeout: float,
    ) -> bool:
        """Execute one validated tool exchange."""
        self.last_error = ""
        operation = step.get("operation", "")
        if operation not in {"dock", "load"}:
            self.last_error = (
                f"Unknown tool exchange operation: '{operation}' "
                "(expected 'load' or 'dock')"
            )
            self._logger.error(self.last_error)
            return False

        new_gripper = "none"
        if operation == "load":
            new_gripper = step.get("gripper", "")
            if new_gripper not in self._grippers:
                self.last_error = (
                    f"Unknown gripper '{new_gripper}' in tool_exchange "
                    f"(available: {', '.join(self._grippers)})"
                )
                self._logger.error(self.last_error)
                return False

        if operation == "dock" and current_gripper != step.get("gripper", ""):
            self.last_error = (
                f"Cannot dock {step.get('gripper', '')}: "
                f"{current_gripper} is attached"
            )
            self._logger.error(self.last_error)
            return False

        if operation == "dock" and not self._use_mock_hardware:
            self._logger.info("Setting tool voltage to 0V for dock (QC release)")
            if not self._set_tool_voltage_via_io(0):
                self.last_error = "Tool voltage change to 0V was not confirmed"
                return False
            self._moveit_manager.notify_voltage_change(0)
            time.sleep(0.5)

        goal = ToolExchangeAction.Goal()
        goal.operation = operation
        goal.gripper = step.get("gripper", "")
        goal.current_attached_gripper = current_gripper
        goal.dock_number = int(step.get("dock_number", 0))
        goal.approach_pose = step.get("approach_pose", "")
        goal.poses_json = poses_json

        if not self._send_and_wait(
            self._action_client,
            goal,
            "tool_exchange",
            timeout,
        ):
            return False

        if new_gripper == current_gripper:
            return True

        self._logger.info(
            f"Gripper changed: {current_gripper} → {new_gripper}, restarting MoveIt"
        )
        self._on_gripper_changed(new_gripper)

        if not self._moveit_manager.launch_moveit_with_gripper(new_gripper):
            self.last_error = (
                "Failed to restart MoveIt after tool exchange "
                f"({current_gripper} → {new_gripper})"
            )
            self._logger.error(self.last_error)
            return False

        self._reset_vision_tf()
        return True

    def _set_tool_voltage_via_io(self, voltage: int) -> bool:
        """Set tool voltage without stopping the external-control program."""
        from ur_msgs.srv import SetIO

        client = None
        try:
            client = self._node.create_client(
                SetIO,
                "/io_and_status_controller/set_io",
                callback_group=self._callback_group,
            )
            if not client.wait_for_service(timeout_sec=3.0):
                self._logger.error("set_io service not available")
                return False

            request = SetIO.Request()
            request.fun = SetIO.Request.FUN_SET_TOOL_VOLTAGE
            request.pin = 0
            request.state = float(voltage)

            # Blocks until another executor thread delivers the reply (or 5 s).
            response = client.call(request, timeout_sec=5.0)
            if response is not None and response.success:
                self._logger.info(f"Tool voltage set to {voltage}V via set_io")
                return True
            self._logger.error(f"Failed to set tool voltage to {voltage}V")
            return False
        except Exception as error:
            self._logger.error(f"Failed to set tool voltage to {voltage}V: {error}")
            return False
        finally:
            if client is not None:
                self._node.destroy_client(client)

    def _reset_vision_tf(self):
        """Clear stale gripper transforms after MoveIt changes URDF."""
        client = self._vision_task_reset_tf
        try:
            if not client.wait_for_service(timeout_sec=3.0):
                self._logger.warning(
                    "vision_task TF reset service not available — "
                    "server may detect wrong gripper frame"
                )
                return
            response = client.call(Trigger.Request(), timeout_sec=5.0)
            if response is not None and response.success:
                self._logger.info("vision_task server TF buffer reset")
            else:
                self._logger.warning("vision_task TF reset did not complete")
        except Exception as error:
            self._logger.warning(f"vision_task TF reset failed: {error}")
