#!/usr/bin/env python3
"""MoveTo action server - moves robot using MTC."""

from hello_orchestrator_py.base_action_server import BaseActionServer, run_server
from hello_orchestrator_py.stages.move_stages import MoveStages
from hello_orchestrator_py_interfaces.action import MoveToNamedState


class MoveActionServer(BaseActionServer):
    """Action server for MoveTo operations using MTC."""

    def __init__(self):
        super().__init__(
            node_name="move_server_py",
            action_name="move_to_named_state_py",
            action_type=MoveToNamedState,
        )

    def initialize_stages(self):
        self._stages = MoveStages(self)


def main(args=None):
    run_server(MoveActionServer, args)


if __name__ == '__main__':
    main()
