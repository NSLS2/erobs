#!/usr/bin/env python3
"""Print action server"""

from hello_orchestrator_py.base_action_server import BaseActionServer, run_server
from hello_orchestrator_py.stages.print_stages import PrintStages
from hello_orchestrator_py_interfaces.action import PrintMessage


class PrintActionServer(BaseActionServer):
    """Action server for printing messages."""

    def __init__(self):
        super().__init__(
            node_name="print_server_py",
            action_name="print_message_py",
            action_type=PrintMessage,
        )

    def initialize_stages(self):
        self._stages = PrintStages(self)


def main(args=None):
    run_server(PrintActionServer, args)


if __name__ == '__main__':
    main()
