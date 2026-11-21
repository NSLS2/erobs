#!/usr/bin/env python3
"""Serve MoveTo actions through MoveIt Task Constructor."""

from beambot.action_servers.base_action_server import BaseActionServer, run_server
from beambot.motion.move_to_task import MoveToTask
from beambot_interfaces.action import MoveToAction


class MoveToActionServer(BaseActionServer):
    """Handle relative moves, Cartesian targets, joint poses, and SRDF states."""

    def __init__(self):
        super().__init__(
            node_name="beambot_moveto_server",
            action_name="beambot_moveto",
            action_type=MoveToAction,
        )

    def create_task(self):
        return MoveToTask(self)


def main(args=None):
    run_server(MoveToActionServer, args)


if __name__ == '__main__':
    main()
