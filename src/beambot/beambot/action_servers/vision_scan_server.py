#!/usr/bin/env python3
"""Serve multi-position marker scans with a server-local pose cache."""

from beambot.action_servers.base_action_server import BaseActionServer, run_server
from beambot.vision.target_localizer import TargetLocalizer
from beambot_interfaces.action import VisionScanAction


class VisionActionServer(BaseActionServer):
    def __init__(self):
        super().__init__(
            node_name="beambot_vision_server",
            action_name="beambot_vision_scan",
            action_type=VisionScanAction,
        )

    def create_task(self):
        """Create the vision engine from the beamline camera settings."""
        from beambot.config_loader import load_beamline_config

        config, _ = load_beamline_config()
        camera_config = config.get("camera", {})
        self.get_logger().info(
            f"Camera config: type={camera_config.get('type')}, "
            f"frame={camera_config.get('frame')}"
        )

        return TargetLocalizer(
            self,
            camera_type=camera_config.get("type"),
            camera_frame=camera_config.get("frame"),
            marker_dictionary=camera_config.get("marker_dictionary"),
        )

    def _execute(self, goal_handle):
        """Scan markers; BaseActionServer finalizes the goal from result.success."""
        goal = goal_handle.request

        # Six joint angles per scan position (radians).
        num_positions = goal.num_scan_positions
        flat = list(goal.scan_positions_flat)

        if len(flat) != num_positions * 6:
            self.get_logger().error(
                f"Invalid scan_positions_flat length: {len(flat)}, "
                f"expected {num_positions * 6}"
            )
            result = VisionScanAction.Result()
            result.success = False
            result.error_message = "Invalid scan positions"
            result.tags_detected = 0
            return result

        scan_positions = [flat[i * 6 : (i + 1) * 6] for i in range(num_positions)]

        scans_per_position = (
            goal.scans_per_position if goal.scans_per_position > 0 else 3
        )
        timeout = goal.timeout if goal.timeout > 0 else 10.0

        self.get_logger().info(
            f"VisionScan: {num_positions} positions × {scans_per_position} scans"
        )

        tags_detected = self._task.scan_all_tags(
            scan_positions=scan_positions,
            scans_per_position=scans_per_position,
            timeout=timeout,
            settle_time=0.3,  # Seconds after each move.
        )

        result = VisionScanAction.Result()
        result.success = tags_detected > 0
        result.tags_detected = tags_detected
        result.error_message = "" if result.success else "No tags detected"
        return result

def main(args=None):
    run_server(VisionActionServer, args)


if __name__ == "__main__":
    main()
