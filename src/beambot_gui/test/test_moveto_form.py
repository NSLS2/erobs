"""Headless tests for MoveToForm mode selector (#93).

Verifies that the mode selector prevents ambiguous goals by emitting only
the active mode's keys in collect_values().
"""

import pytest

pytest.importorskip("PyQt6.QtWidgets", reason="PyQt6 not installed")

from beambot_gui.task_forms import MoveToForm  # noqa: E402

pytestmark = pytest.mark.usefixtures("qapp")


def _make_form(step=None):
    step = step or {"task_type": "move_to"}
    return MoveToForm(step, 0, {})


def test_cartesian_mode_emits_only_cartesian_keys():
    form = _make_form()
    form.mode_combo.setCurrentText("Cartesian target")
    form.cart_x.setValue(0.5)
    form.cart_y.setValue(0.1)
    form.cart_z.setValue(0.3)
    result = form.collect_values()
    assert "cartesian_target" in result
    assert "frame_id" in result
    assert "planning_type" in result
    assert "direction" not in result
    assert "distance" not in result
    assert "target" not in result


def test_relative_mode_emits_only_relative_keys():
    form = _make_form()
    form.mode_combo.setCurrentText("Relative move")
    form.direction.setCurrentText("up")
    form.distance.setValue(0.1)
    result = form.collect_values()
    assert result["direction"] == "up"
    assert result["distance"] == 0.1
    assert "cartesian_target" not in result
    assert "frame_id" not in result
    assert "target" not in result


def test_named_mode_emits_only_target():
    form = _make_form({"task_type": "move_to", "target": "hotplate"})
    form.mode_combo.setCurrentText("Named target")
    result = form.collect_values()
    assert result["target"] == "hotplate"
    assert "direction" not in result
    assert "distance" not in result
    assert "cartesian_target" not in result
    assert "frame_id" not in result


def test_ambiguous_step_defaults_to_relative():
    """Backend precedence: relative > cartesian > named. A step with both
    direction+distance AND cartesian_target should default to Relative mode."""
    step = {
        "task_type": "move_to",
        "direction": "forward",
        "distance": 0.05,
        "cartesian_target": [0.1, 0.2, 0.3],
        "target": "foo",
    }
    form = _make_form(step)
    assert form.mode_combo.currentText() == "Relative move"
    result = form.collect_values()
    assert "direction" in result
    assert "cartesian_target" not in result


def test_joint_mode_emits_target_and_inline_pose():
    """Selecting Joint values, setting 6 spinboxes → step has target + _inline_joint_pose."""
    form = MoveToForm({"task_type": "move_to"}, 2, {})
    form.mode_combo.setCurrentText("Joint values")
    angles = [10.0, -20.0, 30.0, -40.0, 50.0, -60.0]
    for spin, val in zip(form.joint_spins, angles):
        spin.setValue(val)
    result = form.collect_values()
    assert result["target"] == "moveto_joints_3"
    assert result["_inline_joint_pose"]["name"] == "moveto_joints_3"
    assert result["_inline_joint_pose"]["values"] == angles
    assert "direction" not in result
    assert "distance" not in result
    assert "cartesian_target" not in result


def test_joint_mode_detected_from_poses():
    """Step with target pointing at a 6-element poses entry → defaults to Joint values."""
    poses = {"my_joint_pose": [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]}
    step = {"task_type": "move_to", "target": "my_joint_pose"}
    form = MoveToForm(step, 0, poses)
    assert form.mode_combo.currentText() == "Joint values"
    # Spinboxes should be loaded with the pose values
    for spin, expected in zip(form.joint_spins, poses["my_joint_pose"]):
        assert spin.value() == expected
    # Collect preserves the name
    result = form.collect_values()
    assert result["target"] == "my_joint_pose"
    assert result["_inline_joint_pose"]["values"] == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]


def test_read_current_pose_fills_spinboxes():
    """Read current pose button fills spinboxes from provided current_pose."""
    pose = [11.0, 22.0, 33.0, 44.0, 55.0, 66.0]
    form = MoveToForm({"task_type": "move_to"}, 0, {}, current_pose=pose)
    form.mode_combo.setCurrentText("Joint values")
    form._read_current_pose()
    for spin, expected in zip(form.joint_spins, pose):
        assert spin.value() == expected


def test_read_current_pose_none_no_crash(monkeypatch):
    """Read current pose with no live pose warns and leaves spinboxes untouched."""
    # QMessageBox.warning is modal and blocks even under offscreen Qt, so patch
    # it out; we only care that the no-pose path is handled without filling.
    import beambot_gui.task_forms as tf

    warned = {}
    monkeypatch.setattr(
        tf.QMessageBox, "warning",
        lambda *a, **k: warned.setdefault("called", True),
    )
    form = MoveToForm({"task_type": "move_to"}, 0, {}, current_pose=None)
    form.mode_combo.setCurrentText("Joint values")
    form._read_current_pose()
    assert warned.get("called") is True
    # Spinboxes stay at default 0.0 — no live pose was injected.
    assert all(spin.value() == 0.0 for spin in form.joint_spins)


# --- Live preview callback tests ---


def test_preview_joint_mode_calls_cb():
    """Joint mode spinbox changes call preview_cb with 6 degree values."""
    calls = []
    form = MoveToForm({"task_type": "move_to"}, 0, {},
                      preview_cb=calls.append, end_preview_cb=lambda: None)
    form.mode_combo.setCurrentText("Joint values")
    calls.clear()
    angles = [10.0, -20.0, 30.0, -40.0, 50.0, -60.0]
    for spin, val in zip(form.joint_spins, angles):
        spin.setValue(val)
    assert len(calls) >= 1
    assert calls[-1] == angles


def test_preview_named_mode_known_pose():
    """Named mode with a 6-element pose in poses dict calls preview_cb."""
    calls = []
    end_calls = []
    poses = {"scan_pose": [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]}
    form = MoveToForm({"task_type": "move_to"}, 0, poses,
                      preview_cb=calls.append, end_preview_cb=lambda: end_calls.append(1))
    form.mode_combo.setCurrentText("Named target")
    calls.clear()
    end_calls.clear()
    form.target.setCurrentText("scan_pose")
    assert len(calls) >= 1
    assert calls[-1] == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]


def test_preview_named_mode_unknown_pose():
    """Named mode with an unknown name calls end_preview_cb, not preview_cb."""
    calls = []
    end_calls = []
    form = MoveToForm({"task_type": "move_to"}, 0, {},
                      preview_cb=calls.append, end_preview_cb=lambda: end_calls.append(1))
    form.mode_combo.setCurrentText("Named target")
    calls.clear()
    end_calls.clear()
    form.target.setEditText("some_srdf_state")
    assert len(calls) == 0
    assert len(end_calls) >= 1


def test_preview_cartesian_mode_calls_end():
    """Switching to Cartesian mode calls end_preview_cb."""
    calls = []
    end_calls = []
    form = MoveToForm({"task_type": "move_to"}, 0, {},
                      preview_cb=calls.append, end_preview_cb=lambda: end_calls.append(1))
    form.mode_combo.setCurrentText("Joint values")
    calls.clear()
    end_calls.clear()
    form.mode_combo.setCurrentText("Cartesian target")
    assert len(end_calls) >= 1


def test_preview_done_calls_end():
    """Closing the dialog via done() calls end_preview_cb."""
    end_calls = []
    form = MoveToForm({"task_type": "move_to"}, 0, {},
                      preview_cb=lambda x: None, end_preview_cb=lambda: end_calls.append(1))
    end_calls.clear()
    form.done(0)
    assert len(end_calls) >= 1


# --- Embedded viz widget tests ---


def test_viz_widget_embedded_in_content_row():
    """Passing viz_widget embeds it in the form's _content_row layout."""
    from PyQt6.QtWidgets import QWidget

    dummy_viz = QWidget()
    form = MoveToForm({"task_type": "move_to"}, 0, {}, viz_widget=dummy_viz)
    assert dummy_viz.parent() is not None
    assert form.minimumWidth() == 900


def test_viz_widget_detached_on_done():
    """done() detaches the viz widget (setParent(None)) before dialog teardown."""
    from PyQt6.QtWidgets import QWidget

    dummy_viz = QWidget()
    form = MoveToForm({"task_type": "move_to"}, 0, {}, viz_widget=dummy_viz)
    form.done(0)
    assert dummy_viz.parent() is None


def test_no_viz_widget_no_resize():
    """Without viz_widget, form keeps its normal minimum width."""
    form = MoveToForm({"task_type": "move_to"}, 0, {})
    assert form.minimumWidth() == 480


# --- Named-target dropdown tests ---


def test_named_target_combo_populated():
    """Combo box lists all pose names plus moveit_home sentinel."""
    poses = {"alpha": [1, 2, 3, 4, 5, 6], "beta": [0] * 6, "gamma": [0] * 6}
    form = MoveToForm({"task_type": "move_to"}, 0, poses)
    items = [form.target.itemText(i) for i in range(form.target.count())]
    for name in poses:
        assert name in items
    assert "moveit_home" in items


def test_named_target_dropdown_selection_collects():
    """Selecting from dropdown round-trips through collect_values."""
    poses = {"pose_a": [0] * 6, "pose_b": [0] * 6}
    form = MoveToForm({"task_type": "move_to"}, 0, poses)
    form.mode_combo.setCurrentText("Named target")
    form.target.setCurrentText("pose_b")
    result = form.collect_values()
    assert result["target"] == "pose_b"


def test_named_target_arbitrary_text_collects():
    """Typing a value not in the list still round-trips (editable combo)."""
    form = MoveToForm({"task_type": "move_to"}, 0, {})
    form.mode_combo.setCurrentText("Named target")
    form.target.setEditText("custom_srdf_state")
    result = form.collect_values()
    assert result["target"] == "custom_srdf_state"
