"""Sample picking using named joint poses."""

import json
# import threading

from beambot.utils.task_parser import flange_offset_error
from beambot.motion.task_builder import MtcTaskBuilder, parse_constraints


class PickSampleTask(MtcTaskBuilder):
    def __init__(self, rclpy_node, arm_group: str = "", ik_frame: str = ""):
        super().__init__(rclpy_node, arm_group, ik_frame=ik_frame)
        self.vacuum_ok: bool = True
        self.logger.info("PickSampleTask initialized")

    def run(self, goal) -> str | None:
        """Execute a pick; return None on completion or an error string."""
        self.vacuum_ok = True

        error = flange_offset_error(
            direction=goal.offset_direction,
            distance=goal.offset_distance,
        )
        if error:
            return error

        poses = self.parse_poses(goal.poses_json)
        if poses is None:
            return "Failed to parse poses_json for pick_sample"

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

        # Vacuum check disabled; vacuum_ok remains an unchecked True.
        # self.vacuum_ok = self._check_vacuum()
        self.logger.info("Pick complete (vacuum check disabled)")
        return None

    def _run_hardcoded(
        self, goal, poses: dict, gripper_states: dict, constraints
    ) -> str | None:
        """Pick using named approach/target poses, then retreat to approach."""
        self.logger.info(
            f"Hardcoded pick: approach={goal.approach_pose}, target={goal.target_pose}"
        )

        task = self.create_task_template("Pick Sample")
        gripper_planner = self.make_joint_interpolation_planner()

        release_stage = self.make_gripper_stage(
            "open gripper",
            gripper_planner,
            goal.gripper_group,
            gripper_states.get("release", ""),
        )
        if release_stage:
            task.add(release_stage)

        stage = self.make_move_to_named_stage(
            "pick approach",
            goal.approach_pose,
            poses,
            constraints=constraints,
        )
        if not stage:
            return f"Pose '{goal.approach_pose}' not found or invalid (approach)"
        task.add(stage)

        stage = self.make_move_to_named_stage(
            "pick target",
            goal.target_pose,
            poses,
            constraints=constraints,
        )
        if not stage:
            return f"Pose '{goal.target_pose}' not found or invalid (target)"
        task.add(stage)

        grasp_stage = self.make_gripper_stage(
            "close gripper",
            gripper_planner,
            goal.gripper_group,
            gripper_states.get("grasp", ""),
        )
        if grasp_stage:
            task.add(grasp_stage)

        stage = self.make_move_to_named_stage(
            "pick retreat",
            goal.approach_pose,
            poses,
            constraints=constraints,
        )
        if not stage:
            return f"Pose '{goal.approach_pose}' not found or invalid (retreat)"
        task.add(stage)

        error = self.load_plan_execute(task)
        if error:
            return f"Pick execution failed: {error}"

        return None

    # def _check_vacuum(self) -> bool:
    #     """Read ePick status; missing messages or support return True."""
    #     try:
    #         from epick_msgs.msg import ObjectDetectionStatus
    #     except ImportError:
    #         return True
    #
    #     msg_holder = [None]
    #     event = threading.Event()
    #
    #     def _on_status(msg):
    #         msg_holder[0] = msg
    #         event.set()
    #
    #     sub = self.rclpy_node.create_subscription(
    #         ObjectDetectionStatus,
    #         "/object_detection_status",
    #         _on_status,
    #         10,
    #     )
    #     event.wait(timeout=1.0)
    #     self.rclpy_node.destroy_subscription(sub)
    #
    #     if msg_holder[0] is None:
    #         return True  # No status received; grasp is unverified.
    #
    #     NO_OBJECT = 3
    #     detected = msg_holder[0].status != NO_OBJECT
    #     if not detected:
    #         self.logger.warning(
    #             "VACUUM_LOST: ePick reports NO_OBJECT_DETECTED after pick"
    #         )
    #     return detected
