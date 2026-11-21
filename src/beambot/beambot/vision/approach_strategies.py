"""Motion targets and their construction from vision detections."""

import json
from dataclasses import dataclass

from geometry_msgs.msg import PoseStamped


@dataclass(frozen=True)
class CartesianTarget:
    """Approach pose in base_link with optional gripper action and retreat."""

    pose: PoseStamped
    ik_frame: str = ""
    grasp_state: str = ""  # SRDF state; empty skips gripper actuation.
    gripper_group: str = ""
    retreat_pose_key: str = ""  # Named joint pose; empty skips retreat.


@dataclass(frozen=True)
class JointTarget:
    """Joint angles in degrees with optional forward motion and gripper action."""

    joints_deg: list
    forward_distance: float = (
        0.0  # Metres; <= 0 skips forward motion.
    )
    terminal_state: str = ""  # SRDF state; empty skips gripper actuation.
    gripper_group: str = ""


def snap_j6(base_j6_deg: float, angle_deg: float, k_offset_deg: float = 0.0) -> float:
    """Return joint 6 adjusted by the calibrated square-feature angle (degrees)."""
    # Square symmetry limits the correction to (-45, 45] degrees.
    correction = (angle_deg + k_offset_deg) % 90
    if correction > 45:
        correction -= 90
    return base_j6_deg + correction


def compute_approach_pose(detection, ctx):
    """Build an approach target or store its pose for detect-only requests."""
    vision = ctx.vision
    goal = ctx.goal

    approach, active_ik_frame = vision.compute_approach_pose(
        detection,
        goal.z_offset,
        marker_offset_x=goal.marker_offset_x,
        marker_offset_y=goal.marker_offset_y,
        marker_offset_z=goal.marker_offset_z,
        ik_frame_override=goal.ik_frame or "",
    )

    if goal.offset_direction and goal.offset_distance > 0:
        approach = vision._apply_flange_offset(
            approach, goal.offset_direction, goal.offset_distance
        )

    if goal.detect_only:
        ctx.detect_only_pose = approach
        pos = approach.pose.position
        vision.logger.info(
            f"Detect-only: returning approach pose "
            f"[{pos.x:.4f}, {pos.y:.4f}, {pos.z:.4f}] in {approach.header.frame_id}"
        )
        return None

    grasp_state = ""
    terminal = getattr(goal, "terminal_action", "") or ""
    if terminal:
        states = (
            json.loads(goal.gripper_states_json) if goal.gripper_states_json else {}
        )
        # Accept a configured key or raw SRDF state.
        grasp_state = states.get(terminal, terminal)

    return CartesianTarget(
        pose=approach,
        ik_frame=active_ik_frame,
        grasp_state=grasp_state,
        gripper_group=getattr(goal, "gripper_group", "") or "",
        retreat_pose_key=getattr(goal, "retreat_pose", "") or "",
    )


def compute_j6_snap(detection, ctx):
    """Build a joint target by correcting a named pose's joint 6 (degrees)."""
    goal = ctx.goal
    vision = ctx.vision

    base_pose_key = goal.target_pose
    poses = json.loads(goal.poses_json) if goal.poses_json else {}
    base_joints = poses.get(base_pose_key)
    if base_joints is None:
        ctx.error = (
            f"j6_snap: base pose '{base_pose_key}' not found in poses "
            f"(available: {sorted(poses)})"
        )
        return None

    angle = detection["angle_mod90"]
    base_j6 = base_joints[5]
    corrected_j6 = snap_j6(base_j6, angle, goal.k_offset)

    vision.logger.info(
        f"j6_snap: base={base_j6:.1f}°, angle={angle:.1f}°, "
        f"k={goal.k_offset:.1f}° -> target_j6={corrected_j6:.1f}°"
    )

    corrected = list(base_joints)
    corrected[5] = corrected_j6

    terminal = getattr(goal, "terminal_action", "") or ""
    terminal_state = ""
    if terminal:
        states = (
            json.loads(goal.gripper_states_json) if goal.gripper_states_json else {}
        )
        # Accept a configured key or raw SRDF state.
        terminal_state = states.get(terminal, terminal)

    return JointTarget(
        joints_deg=corrected,
        forward_distance=getattr(goal, "forward_distance", 0.0),
        terminal_state=terminal_state,
        gripper_group=getattr(goal, "gripper_group", "") or "",
    )


APPROACH_STRATEGIES = {
    "approach_pose": compute_approach_pose,
    "j6_snap": compute_j6_snap,
}


def get_approach_strategy(name: str):
    """Return a approach strategy or raise KeyError listing the available names."""
    try:
        return APPROACH_STRATEGIES[name]
    except KeyError:
        raise KeyError(
            f"unknown approach_strategy '{name}'. Registered: {sorted(APPROACH_STRATEGIES)}"
        ) from None
