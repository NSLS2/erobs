"""Pose registry file I/O.

Format MUST stay byte-compatible with ``beambot_mcp_server._write_poses_file``
(``yaml.dump`` with ``default_flow_style=None, width=200``) so GUI- and
agent-written poses are indistinguishable on disk. Group comments are NOT
preserved on write; this matches MCP behavior and the current
``cms/poses.yaml`` has none.

Qt-free on purpose so the safety logic is unit-testable without a display.
"""

import os
import tempfile
from pathlib import Path

import yaml


def read_poses(path):
    """Return the pose registry dict, keeping only 6-element list entries."""
    if not path or not Path(path).exists():
        return {}
    with Path(path).open() as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        return {}
    return {k: v for k, v in data.items() if isinstance(v, list) and len(v) == 6}


def write_poses(path, poses):
    """Atomically write ``poses`` to ``path`` (temp file then Path.replace)."""
    dir_path = Path(path).resolve().parent
    dir_path.mkdir(parents=True, exist_ok=True)
    fd, tmp_path = tempfile.mkstemp(dir=dir_path, suffix=".yaml.tmp")
    tmp_path = Path(tmp_path)
    try:
        with os.fdopen(fd, "w") as f:
            yaml.dump(poses, f, default_flow_style=None, width=200)
        tmp_path.replace(path)
    except BaseException:
        if tmp_path.exists():
            tmp_path.unlink()
        raise
