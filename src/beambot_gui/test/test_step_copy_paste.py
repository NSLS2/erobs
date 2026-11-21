"""Tests for step copy/paste."""

import json

import pytest

pytest.importorskip("PyQt6.QtWidgets", reason="PyQt6 not installed")

from PyQt6.QtCore import QMimeData, Qt
from PyQt6.QtTest import QTest
from PyQt6.QtWidgets import QApplication


@pytest.fixture
def set_steps_clipboard(main_window):
    def set_clipboard(data):
        payload = json.dumps(data)
        mime = QMimeData()
        mime.setData(main_window.STEP_CLIPBOARD_MIME, payload.encode("utf-8"))
        mime.setText(payload)
        QApplication.clipboard().setMimeData(mime)

    return set_clipboard


@pytest.fixture
def window(main_window):
    from beambot_gui.ros2_bridge import ROS2Bridge

    ros2 = ROS2Bridge()
    win = main_window.BeambotMainWindow(ros2)
    win.step_list.set_editing_enabled(True)
    yield win
    win.close()


def test_copy_single_step(window, main_window):
    window.config["tasks"] = [{"task_type": "moveto", "target": "home"}]
    window._refresh_tree()
    window.step_list.set_current_row(0)

    window._copy_steps()

    mime = QApplication.clipboard().mimeData()
    assert mime.hasFormat(main_window.STEP_CLIPBOARD_MIME)
    assert json.loads(mime.text()) == [
        {"task_type": "moveto", "target": "home"}
    ]


def test_copy_multiple_steps(window):
    window.config["tasks"] = [
        {"task_type": "moveto", "target": "a"},
        {"task_type": "pick_sample", "tag_id": 1},
        {"task_type": "place_sample", "tag_id": 2},
    ]
    window._refresh_tree()
    window.step_list._list_widget.item(0).setSelected(True)
    window.step_list._list_widget.item(2).setSelected(True)

    window._copy_steps()

    data = json.loads(QApplication.clipboard().text())
    assert [step["task_type"] for step in data] == ["moveto", "place_sample"]


def test_paste_at_end(window, set_steps_clipboard):
    window.config["tasks"] = [{"task_type": "moveto", "target": "home"}]
    window._refresh_tree()
    set_steps_clipboard(
        [{"task_type": "end_effector", "end_effector_type": "hande"}]
    )

    window._paste_steps()

    assert [step["task_type"] for step in window.config["tasks"]] == [
        "moveto",
        "end_effector",
    ]
    assert window.step_list.selected_indices() == [1]


def test_paste_after_selection(window, set_steps_clipboard):
    window.config["tasks"] = [
        {"task_type": "moveto", "target": "a"},
        {"task_type": "moveto", "target": "b"},
    ]
    window._refresh_tree()
    window.step_list.set_current_row(0)
    set_steps_clipboard([{"task_type": "tool_exchange", "gripper": "epick"}])

    window._paste_steps()

    assert [step["task_type"] for step in window.config["tasks"]] == [
        "moveto",
        "tool_exchange",
        "moveto",
    ]


def test_drag_reorder_updates_tasks_and_preserves_group_selection(window):
    window.config["tasks"] = [
        {"task_type": "moveto", "target": name}
        for name in ("a", "b", "c", "d")
    ]
    window._refresh_tree()

    # Simulate Qt moving original rows 1 and 3 ahead of row 0.
    window._reorder_steps([1, 3, 0, 2], [1, 3])

    assert [step["target"] for step in window.config["tasks"]] == ["b", "d", "a", "c"]
    assert window.step_list.selected_indices() == [0, 1]


def test_execute_from_selected_dispatches_remaining_steps(window):
    window.config["tasks"] = [
        {"task_type": "moveto", "target": "a"},
        {"task_type": "moveto", "target": "b"},
        {"task_type": "moveto", "target": "c"},
        {"task_type": "moveto", "target": "d"},
    ]
    window._refresh_tree()
    window.step_list.set_current_row(2)
    sent = {}
    window.ros2.execute_task = lambda config_json, dry_run=False: sent.update(
        config=json.loads(config_json), dry_run=dry_run
    )

    window._execute_from_selected()

    assert [step["target"] for step in sent["config"]["tasks"]] == ["c", "d"]
    assert window.step_list._exec_toolbar._step_label.text() == "Step 3/4"

    window._on_feedback(0, 0, "Initializing MoveIt", "", "")
    assert window.step_list._step_rows[2]._state.name == "PENDING"

    window._on_feedback(50, 1, "moveto", "", "")
    assert window.step_list._step_rows[0]._state.name == "PENDING"
    assert window.step_list._step_rows[2]._state.name == "RUNNING"
    assert window.progress_bar.value() == 75


def test_paste_invalid_clipboard_ignores(window, set_steps_clipboard):
    original = [{"task_type": "moveto"}]
    window.config["tasks"] = original.copy()
    window._refresh_tree()

    QApplication.clipboard().setText("not editor data")
    window._paste_steps()
    set_steps_clipboard({"wrong": "structure"})
    window._paste_steps()
    set_steps_clipboard([{"no_task_type": True}])
    window._paste_steps()
    set_steps_clipboard([{"task_type": "not_a_real_task"}])
    window._paste_steps()

    assert window.config["tasks"] == original


def test_paste_produces_independent_copy(window, set_steps_clipboard):
    window.config["tasks"] = []
    set_steps_clipboard(
        [{"task_type": "moveto", "target": {"pose": [1, 2, 3]}}]
    )

    window._paste_steps()
    window.config["tasks"][0]["target"]["pose"][0] = 99
    window._paste_steps()

    assert window.config["tasks"][0]["target"]["pose"][0] == 99
    assert window.config["tasks"][1]["target"]["pose"][0] == 1


def test_paste_rejects_non_string_task_type(window, set_steps_clipboard):
    original = [{"task_type": "moveto"}]
    window.config["tasks"] = original.copy()
    window._refresh_tree()

    for task_type in ([], 123, None):
        set_steps_clipboard([{"task_type": task_type}])
        window._paste_steps()

    assert window.config["tasks"] == original


def test_paste_rejects_malformed_known_task_atomically(window, set_steps_clipboard):
    original = [{"task_type": "moveto", "target": "home"}]
    window.config["tasks"] = original.copy()
    window._refresh_tree()
    set_steps_clipboard(
        [{"task_type": "pipettor", "operation": "SUCK", "volume_pct": {}}]
    )

    window._paste_steps()

    assert window.config["tasks"] == original


def test_paste_rejects_oversize_payload(window, main_window):
    window.config["tasks"] = [{"task_type": "moveto"}]
    window._refresh_tree()
    mime = QMimeData()
    mime.setData(
        main_window.STEP_CLIPBOARD_MIME,
        b" " * (main_window.STEP_CLIPBOARD_MAX_BYTES + 1),
    )
    QApplication.clipboard().setMimeData(mime)

    window._paste_steps()

    assert window.config["tasks"] == [{"task_type": "moveto"}]


def test_paste_rejects_excessive_step_count(window, main_window, set_steps_clipboard):
    window.config["tasks"] = [{"task_type": "moveto"}]
    window._refresh_tree()
    set_steps_clipboard(
        [{"task_type": "moveto"}] * (main_window.STEP_CLIPBOARD_MAX_STEPS + 1)
    )

    window._paste_steps()

    assert window.config["tasks"] == [{"task_type": "moveto"}]


def test_paste_accepts_core_vision_task(window, set_steps_clipboard):
    window.config["tasks"] = []
    set_steps_clipboard([{"task_type": "vision_task", "target_type": "sample"}])

    window._paste_steps()

    assert window.config["tasks"] == [
        {"task_type": "vision_task", "target_type": "sample"}
    ]


def test_paste_disabled_during_execution(window, set_steps_clipboard):
    window.config["tasks"] = [{"task_type": "moveto"}]
    window._refresh_tree()
    set_steps_clipboard([{"task_type": "end_effector"}])

    window.step_list.set_editing_enabled(False)
    window._paste_steps()
    assert len(window.config["tasks"]) == 1

    window.step_list.set_editing_enabled(True)
    window._paste_steps()
    assert len(window.config["tasks"]) == 2


def test_shortcuts_only_apply_inside_step_list(window, set_steps_clipboard, qapp):
    window.config["tasks"] = [{"task_type": "moveto", "target": "home"}]
    window._refresh_tree()
    set_steps_clipboard([{"task_type": "end_effector"}])
    window.show()
    qapp.processEvents()

    window._save_run_btn.setFocus()
    QTest.keyClick(
        window._save_run_btn,
        Qt.Key.Key_V,
        Qt.KeyboardModifier.ControlModifier,
    )
    assert len(window.config["tasks"]) == 1

    window.step_list._list_widget.setFocus()
    QTest.keyClick(
        window.step_list._list_widget,
        Qt.Key.Key_V,
        Qt.KeyboardModifier.ControlModifier,
    )
    assert len(window.config["tasks"]) == 2
