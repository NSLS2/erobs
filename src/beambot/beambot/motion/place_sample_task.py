"""Sample placement using named joint poses."""

import json

from beambot.utils.task_parser import flange_offset_error
from beambot.motion.task_builder import MtcTaskBuilder, parse_constraints


class PlaceSampleTask(MtcTaskBuilder):
    def __init__(self, rclpy_node, arm_group: str = "", ik_frame: str = ""):
        super().__init__(rclpy_node, arm_group, ik_frame=ik_frame)
        self.logger.info("PlaceSampleTask initialized")

    def run(self, goal) -> str | None:
        """Execute a place; return None on completion or an error string."""
        error = flange_offset_error(
            direction=goal.offset_direction,
            distance=goal.offset_distance,
        )
        if error:
            return error

        poses = self.parse_poses(goal.poses_json)
        if poses is None:
            return "Failed to parse poses_json for place_sample"

        try:
            gripper_states = (
                json.loads(goal.gripper_states_json) if goal.gripper_states_json else {}
            )
        except json.JSONDecodeError as e:
            return f"Invalid gripper_states_json: {e}"

        try:
            constraints = parse_constraints(
                json.loads(goal.constraints_json) if goal.constraints_json else None
            )
        except json.JSONDecodeError as e:
            return f"Invalid constraints_json: {e}"

        error = self._run_hardcoded(goal, poses, gripper_states, constraints)
        if error is not None:
            return error

        self.logger.info("Place complete")
        return None

    def _run_hardcoded(
        self, goal, poses: dict, gripper_states: dict, constraints
    ) -> str | None:
        """Place using named approach/target poses, then retreat to approach."""
        self.logger.info(
            f"Hardcoded place: approach={goal.approach_pose}, target={goal.target_pose}"
        )

        task = self.create_task_template("Place Sample")
        gripper_planner = self.make_joint_interpolation_planner()

        stage = self.make_move_to_named_stage(
            "place approach",
            goal.approach_pose,
            poses,
            constraints=constraints,
        )
        if not stage:
            return f"Pose '{goal.approach_pose}' not found or invalid (approach)"
        task.add(stage)

        stage = self.make_move_to_named_stage(
            "place target",
            goal.target_pose,
            poses,
            constraints=constraints,
        )
        if not stage:
            return f"Pose '{goal.target_pose}' not found or invalid (target)"
        task.add(stage)

        release_stage = self.make_gripper_stage(
            "open gripper",
            gripper_planner,
            goal.gripper_group,
            gripper_states.get("release", ""),
        )
        if release_stage:
            task.add(release_stage)

        stage = self.make_move_to_named_stage(
            "place retreat",
            goal.approach_pose,
            poses,
            constraints=constraints,
        )
        if not stage:
            return f"Pose '{goal.approach_pose}' not found or invalid (retreat)"
        task.add(stage)

        error = self.load_plan_execute(task)
        if error:
            return f"Place execution failed: {error}"

        return None
