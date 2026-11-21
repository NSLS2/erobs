"""Load and dock tools using beamline-configured motion sequences."""

from moveit.task_constructor import stages
from beambot.config_loader import load_beamline_config
from beambot.motion.task_builder import MtcTaskBuilder, joints_from_degrees


def _load_tool_exchange_config() -> dict:
    """Load beamline dock geometry and sequences; reject missing required keys."""
    cfg, _ = load_beamline_config()
    te = cfg.get("tool_exchange") or {}
    required = ("dock_spacing_m", "reference_dock", "load_sequence", "dock_sequence")
    missing = [k for k in required if te.get(k) in (None, "")]
    if missing:
        raise ValueError(
            "Active beamline YAML is missing required tool_exchange keys: "
            f"{missing}. Add a `tool_exchange:` block to "
            "$BEAMBOT_BEAMLINE_CONFIG declaring all of: "
            f"{required}."
        )
    return te


class ToolExchangeTask(MtcTaskBuilder):
    def run(self, goal) -> str | None:
        """Plan and execute a tool exchange; return None or an error string."""
        self.logger.info(
            f"Executing tool exchange: operation={goal.operation}, "
            f"gripper={goal.gripper}, dock={goal.dock_number}"
        )

        try:
            te_cfg = _load_tool_exchange_config()
        except ValueError as e:
            self.logger.error(str(e))
            return str(e)
        dock_spacing = float(te_cfg["dock_spacing_m"])
        reference_dock = int(te_cfg["reference_dock"])
        load_sequence = te_cfg["load_sequence"]
        dock_sequence = te_cfg["dock_sequence"]

        if goal.operation == "load" and goal.current_attached_gripper != "none":
            error = (
                f"Cannot load {goal.gripper}: "
                f"{goal.current_attached_gripper} already attached"
            )
            self.logger.error(error)
            return error

        poses = self.parse_poses(goal.poses_json)
        if poses is None:
            return "Failed to parse poses_json for tool_exchange"

        task_name = "Load Tool" if goal.operation == "load" else "Dock Tool"
        task = self.create_task_template(task_name)
        sampling = self.make_pipeline_planner()
        cartesian = self.make_cartesian_planner()
        cartesian.min_fraction = 1.0  # Tool exchange requires a complete path.

        # Approach the reference dock.
        joint_pose = self.get_joint_pose(poses, goal.approach_pose)
        if joint_pose is None:
            return f"Pose '{goal.approach_pose}' not found or invalid (tool exchange approach)"

        approach = stages.MoveTo("approach", sampling)
        approach.group = self.arm_group
        self._set_ik_frame(approach)
        approach.setGoal(joints_from_degrees(joint_pose))
        task.add(approach)

        # Dock spacing is measured along the IK frame's lateral axis.
        offset_y = dock_spacing * (reference_dock - goal.dock_number)
        if abs(offset_y) >= 1e-4:
            direction = "right" if offset_y >= 0 else "left"
            shift = self.create_relative_move_stage(
                "shift to dock", direction, abs(offset_y), cartesian
            )
            task.add(shift)

        if goal.operation == "load":
            sequence = load_sequence
            stage_prefix = "load"
        elif goal.operation == "dock":
            sequence = dock_sequence
            stage_prefix = "dock"
        else:
            return f"Unknown tool exchange operation: '{goal.operation}' (expected 'load' or 'dock')"

        for idx, move in enumerate(sequence):
            direction = move.get("direction", "")
            distance = float(move.get("distance", 0.0))
            if not direction or distance <= 0:
                return f"Invalid {goal.operation}_sequence step {idx}: {move}"
            stage = self.create_relative_move_stage(
                f"{stage_prefix} step {idx + 1}: {direction} {distance:.3f}m",
                direction, distance, cartesian,
            )
            task.add(stage)

        return self.load_plan_execute(task)
