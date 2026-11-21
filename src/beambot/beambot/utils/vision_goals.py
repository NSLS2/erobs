"""Expand vision task steps into VisionTask goals."""

import json
import math
from collections.abc import Callable, Mapping, Sequence
from typing import Any


def _ignore(_message: str) -> None:
    pass


# Legacy task names map to VisionTask defaults; explicit fields override them.
# Vacuum watchdog bookkeeping is disabled.
VISION_PRESETS = {
    "vision_moveto": {"vision_method": "marker", "approach_strategy": "approach_pose"},
    "pick_sample": {
        "vision_method": "marker",
        "approach_strategy": "approach_pose",
        "terminal_action": "grasp",
        "pre_open": True,
        "retreat_from_scan": True,
        # "watchdog": "arm",
    },
    "place_sample": {
        "vision_method": "marker",
        "approach_strategy": "approach_pose",
        "terminal_action": "release",
        "retreat_from_scan": True,
        # "watchdog": "disarm",
    },
    "pick_spincoater": {
        "vision_method": "spincoater_sample",
        "approach_strategy": "j6_snap",
        "terminal_action": "vacuum_on",
        "scan_pose": "spincoater_scan",
        "default_target_pose": "spincoater_place",
        "forward_distance": 0.003,
    },
    "place_spincoater": {
        "vision_method": "spincoater_pocket",
        "approach_strategy": "j6_snap",
        "terminal_action": "vacuum_off",
        "scan_pose": "spincoater_scan",
        "default_target_pose": "spincoater_place",
        "forward_distance": 0.003,
    },
}

# Deprecated task JSON keys, checked in order: (canonical, legacy).
_LEGACY_KEYS = (
    ("vision_method", "detector"),
    ("vision_method", "detection_type"),
    ("approach_strategy", "goal_computer"),
)

# Route canonical and preset names through VisionTask.
VISION_TASK_TYPES = frozenset({"vision_task", *VISION_PRESETS})


def resolve_vision_step(task_type: str, step: Mapping[str, Any]) -> dict[str, Any]:
    """Merge preset defaults under explicit step fields."""
    cfg = {**VISION_PRESETS.get(task_type, {}), **step}
    # Canonical keys override the deprecated aliases.
    aliased = set()
    for canonical, legacy in _LEGACY_KEYS:
        if canonical not in step and canonical not in aliased and legacy in step:
            cfg[canonical] = step[legacy]
            aliased.add(canonical)
    return cfg


def flatten_scan_positions(
    poses: Mapping[str, Sequence[float]],
    keys: Sequence[str],
    on_warning: Callable[[str], None] = _ignore,
) -> tuple[list[float], int]:
    """Flatten named joint poses (degrees) into radians, skipping unknown keys."""
    flat, count = [], 0
    for key in keys:
        if key in poses:
            flat.extend(math.radians(j) for j in poses[key])
            count += 1
        else:
            on_warning(f"Scan position '{key}' not found in poses, skipping")
    return flat, count


def build_vision_goal(
    cfg: Mapping[str, Any],
    poses_json: str,
    *,
    grippers: Mapping[str, Any],
    current_gripper: str,
    ik_frame: str,
    on_info: Callable[[str], None] = _ignore,
    on_warning: Callable[[str], None] = _ignore,
):
    """Build a VisionTask goal from a step resolved by resolve_vision_step."""
    # Keep module import ROS-free for the task parser.
    from beambot_interfaces.action import VisionTaskAction

    gripper_config = grippers.get(cfg.get("gripper", current_gripper), {})
    default_z_offset = grippers.get(current_gripper, {}).get("z_offset", 0.0)

    goal = VisionTaskAction.Goal()
    # Selectors.
    goal.vision_method = cfg.get("vision_method", "marker")
    goal.approach_strategy = cfg.get("approach_strategy", "approach_pose")
    # Detection inputs.
    goal.tag_id = int(cfg.get("tag_id", 0))
    goal.sample_index = int(cfg.get("sample_index", 1))
    goal.timeout = float(cfg.get("timeout", 10.0))
    goal.strategy = cfg.get("strategy", "")
    goal.edge_inset_mm = float(cfg.get("edge_inset_mm", 0.0))
    # Goal-computation inputs.
    goal.z_offset = float(cfg.get("z_offset", default_z_offset))
    goal.marker_offset_x = float(cfg.get("marker_offset_x", 0.0))
    goal.marker_offset_y = float(cfg.get("marker_offset_y", 0.0))
    goal.marker_offset_z = float(cfg.get("marker_offset_z", 0.0))
    goal.offset_direction = cfg.get("offset_direction", "")
    goal.offset_distance = float(cfg.get("offset_distance", 0.0))
    # Preserve legacy spincoater target fields.
    goal.target_pose = (
        cfg.get("target_pose")
        or cfg.get("place_pose")
        or cfg.get("pickup_pose")
        or cfg.get("default_target_pose", "")
    )
    goal.k_offset = float(cfg.get("k_offset", 0.0))
    goal.forward_distance = float(cfg.get("forward_distance", 0.0))
    # Execution tail.
    goal.scan_pose = cfg.get("scan_pose", "")
    goal.terminal_action = cfg.get("terminal_action", "")
    goal.pre_open = bool(cfg.get("pre_open", False))
    # Pick/place may retreat to the scan pose.
    if cfg.get("retreat_from_scan"):
        goal.retreat_pose = cfg.get("scan_pose", "")
    else:
        goal.retreat_pose = cfg.get("retreat_pose", "")
    goal.gripper_group = gripper_config.get("gripper_group", "")
    goal.gripper_states_json = json.dumps(gripper_config.get("states", {}))
    # Common.
    goal.ik_frame = ik_frame
    goal.detect_only = bool(cfg.get("detect_only", False))
    goal.poses_json = poses_json
    goal.constraints_json = (
        json.dumps(cfg["constraints"]) if "constraints" in cfg else ""
    )

    # Multi-position marker scans.
    scan_keys = cfg.get("scan_positions", [])
    if scan_keys:
        poses = json.loads(poses_json) if poses_json else {}
        flat, count = flatten_scan_positions(poses, scan_keys, on_warning)
        if count > 0:
            goal.scan_positions_flat = flat
            goal.num_scan_positions = count
            on_info(f"Multi-position mode: {count} scan positions")

    return goal
