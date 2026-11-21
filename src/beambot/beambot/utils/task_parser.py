"""Parse orchestrator task scripts and resolve named poses."""

import json
from collections.abc import Callable, Mapping, Sequence
from pathlib import Path
from typing import Any

import yaml

from beambot.config_loader import is_finite_number
from beambot.utils.vision_goals import VISION_TASK_TYPES


DRY_RUN_SUPPORTED_TYPES = {"moveto", "end_effector"}
TASK_MACROS = {"pickup_tip": "tip_rack", "pickup_vial": "vial_rack"}
SUPPORTED_TASK_TYPES = {
    "moveto",
    "end_effector",
    "tool_exchange",
    "vision_scan",
    "pipettor",
    "pause",
    *VISION_TASK_TYPES,
}
MOVETO_DIRECTIONS = (
    "forward",
    "backward",
    "left",
    "right",
    "up",
    "down",
    "x",
    "-x",
    "y",
    "-y",
    "z",
    "-z",
)
MOVETO_PLANNERS = ("", "auto", "joint", "cartesian", "pilz", "pilz_ptp")
GRID_DIRECTION_OPPOSITES = {
    "forward": "backward",
    "backward": "forward",
    "left": "right",
    "right": "left",
    "up": "down",
    "down": "up",
}


def moveto_goal_error(
    *,
    target: Any = "",
    direction: Any = "",
    distance: Any = 0.0,
    cartesian_target: Any = (),
    planning_type: Any = "",
) -> str | None:
    """Return why a MoveTo goal is invalid, or None when it is valid."""
    if not isinstance(planning_type, str) or planning_type not in MOVETO_PLANNERS:
        return f"Unknown MoveTo planning_type: {planning_type!r}"

    named = bool(target)
    relative = bool(direction) or distance != 0.0
    cartesian = bool(cartesian_target)
    if sum((named, relative, cartesian)) != 1:
        return (
            "MoveTo goal must specify exactly one of target, "
            "direction/distance, or cartesian_target"
        )

    if named and (not isinstance(target, str) or not target.strip()):
        return "MoveTo target must be a non-empty string"
    if relative:
        if not isinstance(direction, str) or direction not in MOVETO_DIRECTIONS:
            return f"Unknown MoveTo direction: {direction!r}"
        if not is_finite_number(distance) or distance <= 0.0:
            return "MoveTo distance must be a finite number greater than zero"
    if cartesian and (
        not isinstance(cartesian_target, Sequence)
        or len(cartesian_target) not in (3, 6)
        or not all(is_finite_number(value) for value in cartesian_target)
    ):
        return "MoveTo cartesian_target must contain exactly 3 or 6 finite numbers"
    return None


def flange_offset_error(*, direction: Any = "", distance: Any = 0.0) -> str | None:
    """Return why a flange offset is invalid, or None when it is valid/absent."""
    if not isinstance(direction, str):
        return "Flange offset direction must be a string"
    if not is_finite_number(distance):
        return "Flange offset distance must be a finite number greater than zero"
    if not direction and distance == 0.0:
        return None
    if direction not in MOVETO_DIRECTIONS:
        return f"Unknown flange offset direction: {direction!r}"
    if distance <= 0.0:
        return "Flange offset distance must be a finite number greater than zero"
    return None


def _load_poses_registry(
    poses_file: str, on_warning: Callable[[str], None] | None
) -> dict[str, Any]:
    path = Path(poses_file)
    if not path.is_file():
        return {}
    try:
        with path.open() as stream:
            return yaml.safe_load(stream) or {}
    except Exception as error:  # noqa: BLE001 - unavailable registry is non-fatal
        if on_warning:
            on_warning(f"Failed to read poses registry: {error}")
        return {}


def _finite_number(value: Any, path: str, *, positive: bool = False) -> float:
    if not is_finite_number(value) or (positive and value <= 0):
        qualifier = " greater than zero" if positive else ""
        raise ValueError(f"{path} must be a finite number{qualifier}")
    return float(value)


def _positive_int(value: Any, path: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ValueError(f"{path} must be a positive integer")
    return value


def _grid_direction(value: Any, path: str) -> str:
    if not isinstance(value, str) or value not in GRID_DIRECTION_OPPOSITES:
        allowed = ", ".join(GRID_DIRECTION_OPPOSITES)
        raise ValueError(f"{path} must be one of: {allowed}")
    return value


def expand_grid_target(
    target_name: str,
    vision_targets: Mapping[str, Any],
    task: Mapping[str, Any],
    task_path: str = "task",
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    """Expand a grid vision target into scan, marker and row/column move tasks.

    task supplies row/col or element_index, optional config overrides and, for
    pickup_vial, a pipettor_operation. Also return the resolved element and the
    row/column moves so callers can report them. Raise ValueError when invalid.
    """
    target_path = f"vision_targets.{target_name}"
    base_config = vision_targets.get(target_name)
    if not isinstance(base_config, Mapping):
        raise ValueError(f"{task_path}: missing or invalid {target_path} configuration")
    override = task.get("config", {})
    if not isinstance(override, Mapping):
        raise ValueError(f"{task_path}.config must be an object")
    override_grid = override.get("grid", {})
    if not isinstance(override_grid, Mapping):
        raise ValueError(f"{task_path}.config.grid must be an object")
    config = {**base_config, **override}
    if isinstance(base_config.get("grid"), Mapping):
        config["grid"] = {**base_config["grid"], **override_grid}
    if config.get("mode") != "grid":
        raise ValueError(f"{target_path}.mode must be 'grid'")

    marker_id = config.get("marker_id")
    if isinstance(marker_id, bool) or not isinstance(marker_id, int) or marker_id < 0:
        raise ValueError(f"{target_path}.marker_id must be a non-negative integer")
    scan_pose = config.get("scan_pose", "")
    if not isinstance(scan_pose, str):
        raise ValueError(f"{target_path}.scan_pose must be a string")

    grid = config.get("grid")
    if not isinstance(grid, Mapping):
        raise ValueError(f"{target_path}.grid must be an object")
    rows = _positive_int(grid.get("rows"), f"{target_path}.grid.rows")
    cols = _positive_int(grid.get("cols"), f"{target_path}.grid.cols")

    if "row" in task or "col" in task:
        if "row" not in task or "col" not in task:
            raise ValueError(f"{task_path} must provide both row and col")
        row, col = task["row"], task["col"]
        if isinstance(row, bool) or not isinstance(row, int):
            raise ValueError(f"{task_path}.row must be an integer")
        if isinstance(col, bool) or not isinstance(col, int):
            raise ValueError(f"{task_path}.col must be an integer")
    else:
        element_index = task.get("element_index", 0)
        if isinstance(element_index, bool) or not isinstance(element_index, int):
            raise ValueError(f"{task_path}.element_index must be an integer")
        row, col = divmod(element_index, cols)
    if not 0 <= row < rows or not 0 <= col < cols:
        raise ValueError(
            f"{task_path} row={row}, col={col} out of range "
            f"(max: {rows - 1}, {cols - 1})"
        )

    marker_offset = config.get("marker_offset", {})
    if not isinstance(marker_offset, Mapping):
        raise ValueError(f"{target_path}.marker_offset must be an object")
    default_pitch = _finite_number(
        grid.get("pitch", 0.009), f"{target_path}.grid.pitch", positive=True
    )

    # Columns and rows share one rule: start at the A1 offset (default: the
    # marker offset magnitude) and step one pitch per index along the axis.
    axis_moves = {}
    for axis, axis_name, index, default_direction, marker_key in (
        ("col", "column", col, "left", "x"),
        ("row", "row", row, "up", "y"),
    ):
        prefix = f"{target_path}.grid.{axis}"
        pitch = _finite_number(
            grid.get(f"{axis}_pitch", default_pitch), f"{prefix}_pitch", positive=True
        )
        direction = _grid_direction(
            grid.get(f"{axis}_direction", default_direction), f"{prefix}_direction"
        )
        away = GRID_DIRECTION_OPPOSITES[direction]
        increasing = _grid_direction(
            grid.get(f"{axis}_increasing", away), f"{prefix}_increasing"
        )
        if increasing not in (direction, away):
            raise ValueError(f"{prefix}_increasing must follow the {axis_name} axis")
        marker = _finite_number(
            marker_offset.get(marker_key, 0.0), f"{target_path}.marker_offset.{marker_key}"
        )
        offset = _finite_number(grid.get(f"{axis}_offset", abs(marker)), f"{prefix}_offset")
        offset += index * pitch * (1 if increasing == direction else -1)
        axis_moves[axis] = (direction if offset >= 0 else away, abs(offset))

    moves = config.get("moves")
    if not isinstance(moves, Sequence) or isinstance(moves, (str, bytes)) or not moves:
        raise ValueError(f"{target_path}.moves must be a non-empty list")
    move_tasks = []
    for move_index, move in enumerate(moves):
        move_path = f"{target_path}.moves[{move_index}]"
        if move in ("column_offset", "row_offset"):
            direction, distance = axis_moves["col" if move == "column_offset" else "row"]
            if distance <= 1e-6:
                continue
        elif isinstance(move, Mapping):
            direction = _grid_direction(move.get("direction"), f"{move_path}.direction")
            distance = _finite_number(
                move.get("distance"), f"{move_path}.distance", positive=True
            )
        else:
            raise ValueError(
                f"{move_path} must be column_offset, row_offset, or a move object"
            )
        move_tasks.append(
            {
                "task_type": "moveto",
                "target": "",
                "planning_type": "cartesian",
                "direction": direction,
                "distance": round(distance, 6),
            }
        )

    operation = task.get("pipettor_operation")
    if task.get("task_type") == "pickup_vial" and operation:
        if not isinstance(operation, str):
            raise ValueError(f"{task_path}.pipettor_operation must be a string")
        if operation not in {"SUCK", "EXPEL", "RINSE"}:
            raise ValueError(
                f"{task_path}.pipettor_operation must be SUCK, EXPEL, or RINSE"
            )
        if not move_tasks or not isinstance(moves[-1], Mapping):
            raise ValueError(f"{target_path}.moves must end with a retreat move")
        volume = _finite_number(task.get("volume_pct", 0.5), f"{task_path}.volume_pct")
        operations = [operation]
        if operation == "RINSE":
            count = _positive_int(
                task.get("rinse_count", 2), f"{task_path}.rinse_count"
            )
            operations = ["SUCK", "EXPEL"] * count + ["SUCK"]
        move_tasks[-1:-1] = [
            {"task_type": "pipettor", "operation": op, "volume_pct": volume}
            for op in operations
        ]

    expanded = []
    if scan_pose:
        expanded.append({"task_type": "moveto", "target": scan_pose})
    expanded.append({"task_type": "vision_moveto", "tag_id": marker_id})
    expanded.extend(move_tasks)
    element = {"row": row, "col": col, "index": row * cols + col, "moves": axis_moves}
    return expanded, element


def _expand_task_macros(
    tasks: Sequence[Mapping[str, Any]], vision_targets: Mapping[str, Any]
) -> list[dict[str, Any]]:
    expanded = []
    for index, task in enumerate(tasks):
        if task["task_type"] in TASK_MACROS:
            macro_tasks, _ = expand_grid_target(
                TASK_MACROS[task["task_type"]], vision_targets, task, f"tasks[{index}]"
            )
            expanded.extend(macro_tasks)
        else:
            expanded.append(dict(task))
    return expanded


def parse_task_script(
    full_json: str,
    *,
    dry_run: bool,
    grippers: Mapping[str, Any],
    poses_file: str,
    vision_targets: Mapping[str, Any],
    on_info: Callable[[str], None] | None = None,
    on_warning: Callable[[str], None] | None = None,
) -> tuple[str, list[dict[str, Any]], str]:
    """Parse task JSON, expand macros, and resolve missing named poses.

    Return (start_gripper, tasks, poses_json).
    """
    if not full_json:
        raise ValueError("Goal missing required full_json")

    try:
        script = json.loads(full_json)
    except json.JSONDecodeError as error:
        raise ValueError(f"Invalid JSON: {error}") from error

    if not isinstance(script, Mapping):
        raise ValueError("Task script root must be an object")
    if "start_gripper" not in script:
        raise ValueError("Task script missing 'start_gripper'")
    if "tasks" not in script:
        raise ValueError("Task script missing 'tasks'")

    start_gripper = script["start_gripper"]
    if not isinstance(start_gripper, str):
        raise ValueError("start_gripper must be a string")
    if start_gripper not in grippers:
        available = ", ".join(grippers)
        raise ValueError(f"Unknown gripper: {start_gripper} (available: {available})")

    tasks = script["tasks"]
    if not isinstance(tasks, list) or not tasks:
        raise ValueError("tasks must be a non-empty list")
    allowed_input = SUPPORTED_TASK_TYPES | TASK_MACROS.keys()
    allowed = ", ".join(sorted(allowed_input))
    for index, task in enumerate(tasks):
        if not isinstance(task, Mapping):
            raise ValueError(f"tasks[{index}] must be an object")
        task_type = task.get("task_type")
        if not isinstance(task_type, str) or task_type not in allowed_input:
            raise ValueError(f"tasks[{index}].task_type must be one of: {allowed}")

    if not isinstance(vision_targets, Mapping):
        raise ValueError("vision_targets configuration must be an object")
    tasks = _expand_task_macros(tasks, vision_targets)

    for index, task in enumerate(tasks):
        task_type = task["task_type"]
        if task_type == "end_effector" and "end_effector_type" in task:
            gripper = task["end_effector_type"]
            if not isinstance(gripper, str) or gripper not in grippers:
                raise ValueError(
                    f"tasks[{index}].end_effector_type: unknown gripper {gripper!r}"
                )
        error = flange_offset_error(
            direction=task.get("offset_direction", ""),
            distance=task.get("offset_distance", 0.0),
        )
        if error:
            raise ValueError(f"tasks[{index}]: {error}")
        if task_type == "moveto":
            error = moveto_goal_error(
                target=task.get("target", ""),
                direction=task.get("direction", ""),
                distance=task.get("distance", 0.0),
                cartesian_target=task.get("cartesian_target", ()),
                planning_type=task.get("planning_type", ""),
            )
            if error:
                raise ValueError(f"tasks[{index}]: {error}")

    if dry_run:
        unsupported = [
            (index, task.get("task_type", "?"))
            for index, task in enumerate(tasks)
            if task.get("task_type", "") not in DRY_RUN_SUPPORTED_TYPES
        ]
        if unsupported:
            bad = ", ".join(
                f"step {index + 1} ({task_type})" for index, task_type in unsupported
            )
            allowed = ", ".join(sorted(DRY_RUN_SUPPORTED_TYPES))
            raise ValueError(
                f"Dry-run not supported for: {bad}. "
                f"v1 supports only: {allowed}. "
                "Run without dry_run (or with use_mock_hardware) for full task types."
            )

    poses = script.get("poses", {})
    pose_keys_needed = set()
    for task in tasks:
        target = task.get("target", "")
        if target and target not in poses:
            pose_keys_needed.add(target)
        for key in task.get("scan_positions", []):
            if key not in poses:
                pose_keys_needed.add(key)
        for field in (
            "scan_pose",
            "approach_pose",
            "target_pose",
            "place_pose",
            "pickup_pose",
        ):
            value = task.get(field)
            if value and value not in poses:
                pose_keys_needed.add(value)

    if pose_keys_needed:
        registry = _load_poses_registry(poses_file, on_warning)
        resolved = [key for key in pose_keys_needed if key in registry]
        poses.update((key, registry[key]) for key in resolved)
        if resolved and on_info:
            on_info(f"Auto-resolved {len(resolved)} pose(s) from registry: {resolved}")

    return start_gripper, tasks, json.dumps(poses)
