"""Headless GUI-level tests for the PosesPanel registry owner (#94).

Exercises the real PyQt6 widget under the offscreen platform — no display,
no ROS. Verifies that save/delete persist to the YAML file AND refresh the
in-memory view + re-emit poses_loaded (so the Poses tab and Manage Poses
dialog can never diverge), and that the GUI writer's on-disk format matches
the MCP server's one-line flow style (shared single source of truth).
"""

import pytest

pytest.importorskip("PyQt6.QtWidgets", reason="PyQt6 not installed")

from beambot_gui.poses_panel import PosesPanel  # noqa: E402
from beambot_gui.pose_io import read_poses  # noqa: E402

pytestmark = pytest.mark.usefixtures("qapp")


@pytest.fixture
def registry(tmp_path):
    f = tmp_path / "poses.yaml"
    f.write_text("safe_tool_exchange: [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]\n")
    return f


def test_save_pose_persists_refreshes_and_emits(registry):
    f = registry
    panel = PosesPanel()
    panel.load_from_file(f)
    assert panel.get_poses_file() == f

    emitted = []
    panel.poses_loaded.connect(lambda d: emitted.append(dict(d)))

    assert panel.save_pose("hotplate", [10.0, 20.0, 30.0, 40.0, 50.0, 60.0]) is True
    # in-memory mirror updated, existing pose preserved
    assert "hotplate" in panel.get_poses()
    assert panel.get_poses()["safe_tool_exchange"] == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
    # list widget repainted and signal re-emitted (keeps other surfaces in sync)
    assert panel.pose_list.count() == 2
    assert emitted and "hotplate" in emitted[-1]
    # actually on disk
    assert read_poses(f).get("hotplate") == [10.0, 20.0, 30.0, 40.0, 50.0, 60.0]


def test_delete_pose_persists_and_keeps_siblings(registry):
    f = registry
    panel = PosesPanel()
    panel.load_from_file(f)
    panel.save_pose("temp", [0.0] * 6)
    assert panel.delete_pose("temp") is True
    assert "temp" not in panel.get_poses()
    assert "temp" not in read_poses(f)
    assert "safe_tool_exchange" in read_poses(f)


def test_save_without_registry_returns_false():
    panel = PosesPanel()  # never loaded a file
    assert panel.get_poses_file() is None
    assert panel.save_pose("x", [0.0] * 6) is False  # returns False, no crash


def test_writer_matches_mcp_flow_format(registry):
    f = registry
    panel = PosesPanel()
    panel.load_from_file(f)
    panel.save_pose("p", [1.0, 2.0, 3.0, 4.0, 5.0, 6.0])
    # one-line flow style — byte-compatible with beambot_mcp_server._write_poses_file
    assert "p: [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]" in f.read_text()
