#!/usr/bin/env python3
"""Serve vision-guided movement, sample handling and spincoater tasks."""

import threading

from std_srvs.srv import Trigger

from beambot.action_servers.base_action_server import BaseActionServer, run_server
from beambot.motion.vision_task import VisionTask
from beambot_interfaces.action import VisionTaskAction


class VisionTaskActionServer(BaseActionServer):
    def __init__(self):
        super().__init__(
            node_name="beambot_vision_task_server",
            action_name="beambot_vision_task",
            action_type=VisionTaskAction,
        )

        self._reset_tf_service = self.create_service(
            Trigger, "beambot_vision_task_reset_tf", self._reset_tf_callback
        )
        self.get_logger().info("TF reset service: beambot_vision_task_reset_tf")

        self._warmup_spincoater_model()

    def _warmup_spincoater_model(self):
        """Warm the spincoater model in a background thread."""

        def _warmup():
            try:
                import numpy as np
                from beambot.vision.perception.spincoater_detection import _get_sample_model

                self.get_logger().info(
                    "Warming up spincoater sample model (background)..."
                )
                model = _get_sample_model()
                # Run one prediction to initialize inference resources.
                dummy = np.zeros((640, 640, 3), dtype=np.uint8)
                model(dummy, conf=0.5, verbose=False)
                self.get_logger().info("Spincoater sample model ready")
            except Exception as e:  # noqa: BLE001 — warmup is best-effort
                self.get_logger().warning(f"Spincoater model warmup skipped: {e}")

        threading.Thread(target=_warmup, daemon=True).start()

    def create_task(self):
        """Create the vision task from the beamline camera settings."""
        from beambot.config_loader import load_beamline_config

        config, _ = load_beamline_config()
        camera_config = config.get("camera", {})
        self.get_logger().info(
            f"Camera config: type={camera_config.get('type')}, "
            f"frame={camera_config.get('frame')}"
        )
        return VisionTask(
            self,
            camera_type=camera_config.get("type"),
            camera_frame=camera_config.get("frame"),
            marker_dictionary=camera_config.get("marker_dictionary"),
        )

    def _execute(self, goal_handle):
        """Run the task and populate the action result."""
        goal = goal_handle.request
        error = self._task.run(goal)

        result = VisionTaskAction.Result()
        if error is not None:
            result.success = False
            result.error_message = error
            return result

        result.success = True
        result.vacuum_ok = self._task.vacuum_ok
        result.motion_kind = (
            "NONE" if self._task.last_detected_pose else "CARTESIAN_POSE"
        )
        if self._task.last_detected_pose is not None:
            pose = self._task.last_detected_pose.pose
            result.detected_position = [
                pose.position.x,
                pose.position.y,
                pose.position.z,
            ]
            result.detected_orientation = [
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ]
        return result

    def _reset_tf_callback(self, request, response):
        self._task.reset_tf()
        response.success = True
        response.message = "TF buffer cleared and listener re-created"
        return response


def main(args=None):
    run_server(VisionTaskActionServer, args)


if __name__ == "__main__":
    main()
