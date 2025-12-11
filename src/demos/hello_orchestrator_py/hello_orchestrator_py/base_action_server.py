"""Base class for action servers - handles goal lifecycle and error handling."""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse
from rclpy.action.server import ServerGoalHandle


class BaseActionServer(Node):
    """Base class for action servers. Subclasses implement initialize_stages()."""

    def __init__(self, node_name: str, action_name: str, action_type):
        """Initialize action server."""
        super().__init__(node_name)

        self._executing = False
        self._action_type = action_type
        self._stages = None

        self.initialize_stages()

        if self._stages is None:
            raise RuntimeError(f"{self.__class__.__name__} must set self._stages")

        self._action_server = ActionServer(
            self,
            action_type,
            action_name,
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
        )

        self.get_logger().info(f"{node_name} started on '{action_name}'")

    def initialize_stages(self):
        """Create and assign the stages instance. Override in subclass."""
        raise NotImplementedError("Subclass must implement initialize_stages()")

    def _goal_callback(self, goal_request) -> GoalResponse:
        """Accept goal if not already executing."""
        if self._executing:
            self.get_logger().warning("Rejecting goal: server busy")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT
    
    """    Note: cancel_callback is omitted - defaults to REJECT. Individual action servers
    cannot yet cancel mid-execution (while performing tasks). Cancellation is handled at the
    orchestrator level (between tasks).  """

    def _execute_callback(self, goal_handle: ServerGoalHandle):
        """Execute goal with error handling."""
        self._executing = True

        try:
            success = self._stages.run(goal_handle.request)
            result = self._action_type.Result()
            result.success = success
            if not success:
                result.error_message = "Stage execution failed"

            goal_handle.succeed() if success else goal_handle.abort()
            return result

        except Exception as e:
            self.get_logger().error(f"Exception: {e}")
            goal_handle.abort()
            return self._action_type.Result(success=False, error_message=str(e))

        finally:
            self._executing = False


def run_server(server_class, args=None):
    """Run action server with standard ROS 2 lifecycle."""
    rclpy.init(args=args)
    node = server_class()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
