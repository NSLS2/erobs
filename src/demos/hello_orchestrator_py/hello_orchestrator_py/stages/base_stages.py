"""Base MTC stages - common functionality for MTC tasks."""

import rclcpp
from moveit.task_constructor import core, stages
from moveit_msgs.msg import MoveItErrorCodes

# rclcpp MTC node initialization 
rclcpp.init()
_options = rclcpp.NodeOptions()
_options.automatically_declare_parameters_from_overrides = True
_options.allow_undeclared_parameters = True
_mtc_node = rclcpp.Node("mtc_task_node", _options)


class BaseStages:
    """Base class for MTC stage implementations."""

    def __init__(self, node):
        """Initialize with ROS 2 node for logging."""
        self.node = node
        self.logger = node.get_logger()
        self._mtc_node = _mtc_node
        self.arm_group = "ur_arm"  # from moveit SRDF 

    def create_task_template(self, task_name: str) -> core.Task:
        """Create MTC task with CurrentState stage (Every MTC task needs CurrentState as first stage)."""
        task = core.Task()
        task.name = task_name
        task.loadRobotModel(self._mtc_node)

        current_state = stages.CurrentState("current state")
        task.add(current_state)

        return task

    def make_pipeline_planner(self):
        """Create OMPL pipeline planner."""
        planner = core.PipelinePlanner(self._mtc_node, "ompl")
        planner.max_velocity_scaling_factor = 0.2
        planner.max_acceleration_scaling_factor = 0.2
        return planner

    def load_plan_execute(self, task: core.Task) -> bool:
        """Plan and execute MTC task. Returns True on success."""
        try:
            task.init()

            if not task.plan(max_solutions=1):
                self.logger.error(f"Planning failed: {task.name}")
                return False

            if not task.solutions:
                self.logger.error(f"No solutions found: {task.name}")
                return False

            self.logger.info(f"Planning succeeded: {task.name}")

            result = task.execute(task.solutions[0])
            if result.val != MoveItErrorCodes.SUCCESS:
                self.logger.error(f"Execution failed: {task.name} (error: {result.val})")
                return False

            self.logger.info(f"Task completed: {task.name}")
            return True

        except Exception as e:
            self.logger.error(f"Task failed: {e}")
            return False
