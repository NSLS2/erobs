"""Pure-Python tests for the Qt-free pose registry I/O (no display needed)."""

import tempfile
from pathlib import Path
from unittest.mock import patch

from beambot_gui.pose_io import read_poses, write_poses


def test_round_trip():
    with tempfile.TemporaryDirectory() as d:
        path = Path(d) / "poses.yaml"
        poses = {"home": [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]}
        write_poses(path, poses)
        assert read_poses(path) == poses


def test_merge_without_clobber():
    with tempfile.TemporaryDirectory() as d:
        path = Path(d) / "poses.yaml"
        write_poses(path, {"a": [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]})
        existing = read_poses(path)
        existing["b"] = [2.0, 2.0, 2.0, 2.0, 2.0, 2.0]
        write_poses(path, existing)
        result = read_poses(path)
        assert "a" in result and "b" in result


def test_byte_format_flow_style():
    with tempfile.TemporaryDirectory() as d:
        path = Path(d) / "poses.yaml"
        write_poses(path, {"p": [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]})
        with path.open() as f:
            text = f.read()
        assert "p: [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]" in text


def test_delete():
    with tempfile.TemporaryDirectory() as d:
        path = Path(d) / "poses.yaml"
        write_poses(path, {"a": [1.0] * 6, "b": [2.0] * 6})
        poses = read_poses(path)
        poses.pop("a")
        write_poses(path, poses)
        result = read_poses(path)
        assert "a" not in result and "b" in result


def test_no_leftover_tmp_files():
    with tempfile.TemporaryDirectory() as d:
        path = Path(d) / "poses.yaml"
        write_poses(path, {"a": [1.0] * 6})
        leftovers = list(Path(d).glob("*.yaml.tmp"))
        assert leftovers == []


def test_failed_write_preserves_original():
    with tempfile.TemporaryDirectory() as d:
        path = Path(d) / "nested" / "poses.yaml"
        poses = {"home": [1.0] * 6}
        write_poses(str(path), poses)
        with patch("beambot_gui.pose_io.yaml.dump", side_effect=RuntimeError("write failed")):
            try:
                write_poses(path, {"other": [2.0] * 6})
            except RuntimeError:
                pass
            else:
                raise AssertionError("Expected write failure")
        assert read_poses(str(path)) == poses
        assert list(path.parent.glob("*.yaml.tmp")) == []


if __name__ == "__main__":
    test_round_trip()
    test_merge_without_clobber()
    test_byte_format_flow_style()
    test_delete()
    test_no_leftover_tmp_files()
    test_failed_write_preserves_original()
    print("All pose_io tests passed.")
