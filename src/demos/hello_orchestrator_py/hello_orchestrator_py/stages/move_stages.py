"""MoveTo stages - MTC-based motion planning."""

from moveit.task_constructor import stages
from hello_orchestrator_py.stages.base_stages import BaseStages


class MoveStages(BaseStages):
    """Handles MoveTo action using MTC."""

    def run(self, goal) -> bool:
        """Execute MoveTo action.

        MoveTo.setGoal() accepts: str (SRDF named pose), joint map,
        PoseStamped, PointStamped, or RobotState. Demo only uses SRDF named poses.
        """
        task = self.create_task_template("MoveTo Task")
        planner = self.make_pipeline_planner()

        move_stage = stages.MoveTo(f"move_to_{goal.target_pose}", planner)
        move_stage.group = self.arm_group
        move_stage.setGoal(goal.target_pose)  # SRDF named pose

        self.logger.info(f"Moving to named state: {goal.target_pose}")

        task.add(move_stage)
        return self.load_plan_execute(task)
