#!/usr/bin/env python3
"""Serve named-pose pick and place sample actions."""

from rclpy.action import ActionServer

from beambot.action_servers.base_action_server import BaseActionServer, run_server
from beambot.motion.pick_sample_task import PickSampleTask
from beambot.motion.place_sample_task import PlaceSampleTask
from beambot_interfaces.action import PickSampleAction, PlaceSampleAction


class SampleActionServer(BaseActionServer):
    """Host PickSampleAction and PlaceSampleAction."""

    def __init__(self):
        super().__init__(
            node_name="beambot_sample_server",
            action_name="beambot_pick_sample",
            action_type=PickSampleAction,
        )

        # Base class owns pick; register place separately.
        self._place_action_server = ActionServer(
            self,
            PlaceSampleAction,
            "beambot_place_sample",
            execute_callback=self._execute_place,
            goal_callback=self._goal_callback,
        )
        self.get_logger().info(
            "PlaceSample action server started: beambot_place_sample"
        )

    def create_task(self):
        """Create the pick task; place is held separately."""
        self._place_task = PlaceSampleTask(self)
        return PickSampleTask(self)

    def _execute(self, goal_handle):
        """Run the pick task."""
        goal = goal_handle.request
        error = self._task.run(goal)

        result = PickSampleAction.Result()
        if error is not None:
            result.success = False
            result.error_message = error
        else:
            result.success = True
            result.vacuum_ok = self._task.vacuum_ok

        return result

    def _execute_place(self, goal_handle):
        """Run the place task and finalize its directly registered goal."""
        try:
            goal = goal_handle.request
            error = self._place_task.run(goal)

            result = PlaceSampleAction.Result()
            if error is not None:
                result.success = False
                result.error_message = error
                goal_handle.abort()
            else:
                result.success = True
                goal_handle.succeed()

            return result
        finally:
            self._executing = False


def main(args=None):
    run_server(SampleActionServer, args)


if __name__ == "__main__":
    main()
