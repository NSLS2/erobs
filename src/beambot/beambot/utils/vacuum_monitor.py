"""Optional ePick vacuum-loss tracking between execution units.

The subscription and orchestrator integration are disabled. Both must be
restored for live detection; this monitor does not stop in-flight motion.
"""

from typing import Any

from rclpy.node import Node


# ObjectDetectionStatus.NO_OBJECT_DETECTED; keep ePick imports optional.
_NO_OBJECT = 3


class VacuumMonitor:
    """Track grasp state and latch vacuum loss for the orchestrator."""

    def __init__(self, node: Node, grippers_config: dict[str, Any], callback_group=None):
        self._logger = node.get_logger()
        self._grippers = grippers_config

        self.armed = False
        self.lost = False
        self.status: 'int | None' = None

        # Disabled to avoid callback overhead while loss checks are inactive.
        # Re-enable the subscription and orchestrator integration together.
        _SUBSCRIBE = False

        if not _SUBSCRIBE:
            self._sub = None
            return

        try:
            # Beamlines without ePick do not require its message package.
            from epick_msgs.msg import ObjectDetectionStatus
        except ImportError:
            self._sub = None
            self._logger.info(
                "epick_msgs not available — vacuum monitor inactive "
                "(non-ePick beamline)"
            )
            return

        self._sub = node.create_subscription(
            ObjectDetectionStatus, '/object_detection_status',
            self._on_status, 10,
            callback_group=callback_group,
        )

    def reset(self):
        """Clear arming and loss flags for a new goal."""
        self.armed = False
        self.lost = False

    def _on_status(self, msg):
        """Cache status and latch a no-object report while armed."""
        self.status = int(msg.status)
        if self.armed and not self.lost and self.status == _NO_OBJECT:
            self.lost = True
            self._logger.warning(
                "VACUUM_LOST: object detection status changed to NO_OBJECT_DETECTED "
                "while vacuum is active"
            )

    def update_after_tasks(
        self, executed_tasks: list[dict[str, Any]], current_gripper: str
    ):
        """Update arming after executed end-effector actions.

        grippers.epick.states.grasp/release must match SRDF state names.
        Missing names are logged and leave the existing monitor state unchanged.
        """
        if current_gripper != "epick":
            return

        states = self._grippers.get("epick", {}).get("states", {})
        grasp_state = states.get("grasp")
        release_state = states.get("release")
        if not grasp_state or not release_state:
            self._logger.error(
                "Vacuum monitor disabled: grippers.epick.states.grasp/release "
                "must be declared in the active beamline YAML and match the "
                "SRDF group_state names. Drop detection is OFF until fixed."
            )
            return

        for task in executed_tasks:
            if task.get("task_type") != "end_effector":
                continue
            action = task.get("end_effector_action", "")
            if action == grasp_state:
                self.lost = False
                self.armed = True
                self._logger.info(f"Vacuum monitor ARMED ({grasp_state} detected)")
                # TODO: Use fresh post-grasp status; cached readings can falsely report a loss.
                if self.status == _NO_OBJECT:
                    self.lost = True
                    self._logger.warning(
                        f"VACUUM_LOST: no seal detected immediately after {grasp_state}"
                    )
            elif action == release_state:
                self.armed = False
                self.lost = False
                self._logger.info(f"Vacuum monitor DISARMED ({release_state} detected)")

    def check_lost(self) -> str | None:
        """Return a pending loss error once and disarm; otherwise return None."""
        if not self.armed or not self.lost:
            return None
        self.armed = False
        return (
            "VACUUM_LOST: object dropped — ePick reports NO_OBJECT_DETECTED "
            "while vacuum was active. Send vacuum_off then vacuum_on to retry."
        )
