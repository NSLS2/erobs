"""Gripper motion tasks using SRDF-defined group states."""

from moveit.task_constructor import core
from beambot.motion.task_builder import MtcTaskBuilder


class EndEffectorTask(MtcTaskBuilder):
    def add_to_task(self, task: core.Task, goal, planner=None) -> str | None:
        """Append a gripper stage without executing; return None or an error."""
        if not goal.gripper_group:
            self.logger.info("No gripper group - treating as no-op success")
            return None

        if not goal.end_effector_action:
            error = "No end_effector_action specified"
            self.logger.error(error)
            return error

        if planner is None:
            planner = self.make_joint_interpolation_planner()

        stage = self.make_gripper_stage(
            f"gripper_{goal.end_effector_action}",
            planner,
            goal.gripper_group,
            goal.end_effector_action,
        )

        task.add(stage)

        self.logger.info(
            f"Planning gripper action: {goal.end_effector_action} "
            f"(group: {goal.gripper_group})"
        )
        return None

    def run(self, goal) -> str | None:
        """Build, plan, and execute a gripper task; return None or an error."""
        if not goal.gripper_group:
            self.logger.info("No gripper group - treating as no-op success")
            return None

        task = self.create_task_template("EndEffector Task")

        error = self.add_to_task(task, goal)
        if error is not None:
            return error

        return self.load_plan_execute(task)
