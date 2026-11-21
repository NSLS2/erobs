"""Load and cache the YAML selected by BEAMBOT_BEAMLINE_CONFIG.

The loader requires explicit beamline selection; convenience helpers retain
their documented defaults when configuration is unavailable.
"""

import json
import math
import os
import threading
from numbers import Real
from pathlib import Path

import yaml


_ENV_VAR = "BEAMBOT_BEAMLINE_CONFIG"

# Parse once to keep frequent callers off disk; repeated YAML reads delayed planning.
# The lock prevents duplicate initial loads. Cached data, including nested dicts,
# is shared and must be treated as read-only; runtime overrides belong to callers.
_config_cache: tuple[dict, str] | None = None
_config_cache_lock = threading.Lock()


class BeamlineConfigError(RuntimeError):
    """Raised when the beamline config cannot be located, parsed or validated."""


def get_beamline_config_path() -> str:
    """Return the selected YAML's absolute path; reject an unset or invalid path."""
    raw = os.environ.get(_ENV_VAR, "").strip()
    if not raw:
        raise BeamlineConfigError(
            f"{_ENV_VAR} environment variable is not set.\n"
            f"Set it to the absolute path of your beamline's YAML config, e.g.:\n"
            f"    export {_ENV_VAR}=/path/to/your_beamline.yaml\n"
            f"Start from the template: src/beambot/config/example_cms_beamline.yaml"
        )

    # Preserve unknown '~user' handling and normalize '..' without following symlinks.
    path = Path(os.path.abspath(os.path.expanduser(raw)))
    if not path.is_file():
        raise BeamlineConfigError(
            f"{_ENV_VAR} points at a file that does not exist: {path}"
        )
    return str(path)


def load_beamline_config() -> tuple[dict, str]:
    """Return the shared configuration dict and source path for path resolution.

    Treat the dict as read-only. Successful loads are cached until reset or
    process exit; environment/file changes are not detected. Failures are retried.
    """
    global _config_cache
    if _config_cache is None:
        with _config_cache_lock:
            if _config_cache is None:
                _config_cache = _load_beamline_config_uncached()
    return _config_cache


def _load_beamline_config_uncached() -> tuple[dict, str]:
    """Read the selected YAML and require a mapping at the root."""
    path = get_beamline_config_path()
    try:
        with open(path, "r") as f:
            data = yaml.safe_load(f) or {}
    except yaml.YAMLError as e:
        raise BeamlineConfigError(f"Failed to parse {path}: {e}") from e
    if not isinstance(data, dict):
        raise BeamlineConfigError(f"{path}: expected a YAML mapping at root")
    errors = validate_beamline_config(data)
    if errors:
        raise BeamlineConfigError(
            f"{path}: invalid beamline config:\n  " + "\n  ".join(errors)
        )
    return data, path


ALLOWED_TOOL_VOLTAGES = (0, 12, 24)


def is_finite_number(value) -> bool:
    """True for real, finite numbers; bools and strings are rejected."""
    return not isinstance(value, bool) and isinstance(value, Real) and math.isfinite(value)


def validate_beamline_config(config: dict) -> list[str]:
    """Return every problem found in a parsed beamline config; empty when valid.

    beamline, robot.ip, robot.moveit_config_package, robot.description_package and
    grippers are required; other sections a beamline does not use are absent and
    skipped. Checks run once per process at load, so a bad value stops the stack
    before it moves the robot.
    """
    errors = []
    if "beamline" not in config:
        errors.append("beamline: required")
    robot = config.get("robot")
    if not isinstance(robot, dict):
        robot = {}
    for key in ("ip", "moveit_config_package", "description_package"):
        if not robot.get(key):
            errors.append(f"robot.{key}: required")
    grippers = config.get("grippers")
    if not isinstance(grippers, dict):
        errors.append(f"grippers: expected a mapping, got {grippers!r}")
        grippers = {}
    for name, gripper in grippers.items():
        prefix = f"grippers.{name}"
        if not isinstance(gripper, dict):
            errors.append(f"{prefix}: expected a mapping")
            continue

        mass = gripper.get("payload_mass")
        if not (is_finite_number(mass) and mass > 0):
            errors.append(f"{prefix}.payload_mass: expected a finite number > 0, got {mass!r}")
        cog = gripper.get("payload_cog")
        if not (isinstance(cog, dict) and all(is_finite_number(cog.get(a)) for a in "xyz")):
            errors.append(f"{prefix}.payload_cog: expected finite numbers x, y, z, got {cog!r}")

        # Omitted tool_voltage means 0 V (off), suitable for passive tools.
        voltage = gripper.get("tool_voltage", 0)
        if isinstance(voltage, bool) or voltage not in ALLOWED_TOOL_VOLTAGES:
            errors.append(
                f"{prefix}.tool_voltage: expected one of {ALLOWED_TOOL_VOLTAGES}, got {voltage!r}"
            )
    return errors


def reset_beamline_config_cache() -> None:
    """Clear the loader cache for tests or explicit reloads.

    Existing references and caller-specific caches are not refreshed.
    """
    global _config_cache
    with _config_cache_lock:
        _config_cache = None


_DEFAULT_ARM_JOINTS = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)


def arm_joint_names() -> list[str]:
    """Return joint names in pose-registry order, defaulting to the six UR joints.

    Missing settings and read errors use the default, including isolated imports.
    """
    try:
        config, _ = load_beamline_config()
        joints = config.get("robot", {}).get("arm_joints")
        if joints:
            return list(joints)
    except Exception:
        pass
    return list(_DEFAULT_ARM_JOINTS)


def gripper_tip_frame(gripper: str, default: str = "flange") -> str:
    """Return the gripper's tip frame, or default on missing keys or read errors."""
    try:
        config, _ = load_beamline_config()
        return config.get("grippers", {}).get(gripper, {}).get("tip_frame", default)
    except Exception:
        return default


def z_offset_for_tip_frame(tip_frame: str, default: float = 0.0) -> float:
    """Return the matching tip frame's z_offset, or default if unavailable.

    Vision identifies the active tool by tip frame rather than gripper name.
    """
    try:
        config, _ = load_beamline_config()
        for gconf in config.get("grippers", {}).values():
            if gconf.get("tip_frame") == tip_frame:
                return float(gconf.get("z_offset", default))
        return default
    except Exception:
        return default


def configured_tip_frames() -> list[str]:
    """Return unique tip frames in config order, excluding flange; [] on errors."""
    try:
        config, _ = load_beamline_config()
        frames = []
        for gconf in config.get("grippers", {}).values():
            tf = gconf.get("tip_frame")
            if tf and tf != "flange" and tf not in frames:
                frames.append(tf)
        return frames
    except Exception:
        return []


def moveit_config_package() -> str:
    """Return the beamline's MoveIt config package (robot.moveit_config_package)."""
    config, _ = load_beamline_config()
    return config["robot"]["moveit_config_package"]


def description_package() -> str:
    """Return the beamline's robot description package (robot.description_package)."""
    config, _ = load_beamline_config()
    return config["robot"]["description_package"]


def gripper_urdf_file(gripper: str, default: str = "ur_standalone.urdf") -> str:
    """Return the GUI's gripper URDF filename, or default if unavailable."""
    try:
        config, _ = load_beamline_config()
        return config.get("grippers", {}).get(gripper, {}).get("urdf_file", default)
    except Exception:
        return default


# Use move_group's pipeline files so plugins and adapters, including ValidateSolution, match.
_PLANNING_PIPELINE_FILES = {
    "ompl": "ompl_planning.yaml",
    "pilz_industrial_motion_planner": "pilz_industrial_motion_planner_planning.yaml",
    "stomp": "stomp_planning.yaml",
}


def _emit_param(prefix: str, value, args: list) -> None:
    """Flatten nested mappings into dotted ROS parameter names and -p arguments."""
    if isinstance(value, dict):
        for k, v in value.items():
            _emit_param(f"{prefix}.{k}", v, args)
    else:
        rendered = json.dumps(value).replace('"', "'")
        args += ["-p", f"{prefix}:={rendered}"]


def build_pipeline_param_args() -> list:
    """Build OMPL/Pilz and optional STOMP parameters for MTC and action servers.

    Forward all keys, including planner_configs and group settings, so requested
    planner IDs resolve instead of falling back to the default planner.
    """
    from ament_index_python.packages import get_package_share_path

    cfg_root = get_package_share_path(moveit_config_package()) / "config"
    args: list = []
    for pipeline, filename in _PLANNING_PIPELINE_FILES.items():
        path = cfg_root / filename
        # OMPL/Pilz are required; beamlines opt into STOMP with its config file.
        if pipeline == "stomp" and not path.exists():
            continue
        with path.open() as f:
            data = yaml.safe_load(f) or {}
        for key, value in data.items():
            _emit_param(f"{pipeline}.{key}", value, args)
    return args


def gripper_launch_config(gripper: str) -> dict:
    """Return configured gripper launch values, converting numbers/bools to strings.

    urdf_xacro is the launch Xacro, not the GUI's urdf_file. tool_comm_params
    remains a dict of strings; booleans become "true"/"false". The shared
    ros2_control controller file remains a launch constant.
    """
    config, _ = load_beamline_config()
    g = config.get("grippers", {}).get(gripper)
    if g is None:
        raise BeamlineConfigError(
            f"Gripper '{gripper}' not declared in beamline config grippers: block"
        )

    def _s(v):
        if isinstance(v, bool):
            return str(v).lower()
        return str(v)

    return {
        "urdf_xacro": g["launch_urdf"],
        "moveit_controllers": g["moveit_controllers_file"],
        "tool_voltage": _s(g["tool_voltage"]),
        "use_tool_communication": _s(g["use_tool_communication"]),
        "tool_comm_params": {k: _s(v) for k, v in (g.get("tool_comm_params") or {}).items()},
    }


def resolve_beamline_path(rel_or_abs: str, config_path: str) -> str:
    """Expand '~' and resolve a configured path without requiring it to exist.

    Relative paths use the nearest ancestor containing src/ (up to 10 directories),
    or the config directory if none is found. Absolute paths are normalized
    without following symlinks.
    """
    if not rel_or_abs:
        return ""
    # Unlike Path.expanduser(), expanduser() leaves unknown '~user' names unchanged.
    expanded = Path(os.path.expanduser(rel_or_abs))
    if expanded.is_absolute():
        # Preserve lexical normalization; resolve() would follow symlinks.
        return os.path.abspath(expanded)

    config_dir = Path(config_path).resolve().parent
    candidate = config_dir
    for _ in range(10):
        if (candidate / "src").is_dir():
            return str(candidate / expanded)
        parent = candidate.parent
        if parent == candidate:
            break
        candidate = parent

    return str(config_dir / expanded)
