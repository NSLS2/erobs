"""Print stages - simple message printing."""


class PrintStages:
    """Handles print action."""

    def __init__(self, node):
        self.logger = node.get_logger()

    def run(self, goal) -> bool:
        self.logger.info(f"MESSAGE: {goal.message}")
        return True
