#!/usr/bin/env python3
"""EndEffectorAction server - handles gripper commands via MTC."""

from beambot.action_servers.base_action_server import BaseActionServer, run_server
from beambot.motion.end_effector_task import EndEffectorTask
from beambot_interfaces.action import EndEffectorAction


class EndEffectorActionServer(BaseActionServer):
    """Action server for EndEffector (gripper) operations.

    Handles gripper open/close commands by moving to SRDF-defined states.
    """

    def __init__(self):
        """Initialize the EndEffector action server."""
        super().__init__(
            node_name="beambot_endeffector_server",
            action_name="beambot_endeffector",
            action_type=EndEffectorAction,
        )

    def create_task(self):
        return EndEffectorTask(self)


def main(args=None):
    run_server(EndEffectorActionServer, args)


if __name__ == '__main__':
    main()
