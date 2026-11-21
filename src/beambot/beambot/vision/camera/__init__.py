"""Shared marker detection results and camera-backend selection."""

from dataclasses import dataclass
from importlib import import_module
from types import ModuleType

from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import Pose


@dataclass
class DetectionResult:
    """Marker IDs and poses with the capture timestamp, when available."""
    markers: list[tuple[int, Pose]]
    capture_stamp: TimeMsg | None


_CAMERA_MODULES = {
    "zivid": "beambot.vision.camera.zivid",
    "zed": "beambot.vision.camera.zed",
}


def get_camera(camera_type: str) -> ModuleType:
    """Load the selected camera module without importing other backends."""
    if camera_type not in _CAMERA_MODULES:
        supported = ", ".join(_CAMERA_MODULES)
        raise ValueError(
            f"Unsupported camera type: '{camera_type}'. "
            f"Supported types: {supported}"
        )

    try:
        return import_module(_CAMERA_MODULES[camera_type])
    except ImportError as e:
        raise ImportError(
            f"Camera backend '{camera_type}' is not available: {e}. "
            f"Install the required packages or select a different camera "
            f"in the beamline config."
        ) from e
