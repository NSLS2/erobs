"""Execution controls follow the orchestrator's published state."""

from types import SimpleNamespace


class _Control:
    def setEnabled(self, enabled):
        self.enabled = enabled

    def isEnabled(self):
        return self.enabled


class _StepList:
    def set_editing_enabled(self, enabled):
        self.editing_enabled = enabled

    def set_paused(self, paused):
        self.paused = paused

    def selected_indices(self):
        return []


def _window(BeambotMainWindow):
    controls = [_Control() for _ in range(12)]
    window = SimpleNamespace(
        exec_btn=controls[0],
        execute_from_btn=controls[1],
        pause_btn=controls[2],
        resume_btn=controls[3],
        stop_btn=controls[4],
        task_toolbar=controls[5],
        up_step_btn=controls[6],
        down_step_btn=controls[7],
        remove_step_btn=controls[8],
        clear_steps_btn=controls[9],
        gripper_combo=controls[10],
        dry_run_check=controls[11],
        step_list=_StepList(),
        _execution_state=None,
        _goal_pending=False,
        _log=lambda message: None,
    )
    window._project_execution_state = (
        lambda: BeambotMainWindow._project_execution_state(window)
    )
    window._update_execute_from_selected = (
        lambda _=None: BeambotMainWindow._update_execute_from_selected(window, _)
    )
    return window


def test_pause_step_default_summary_and_preview_boundary(main_window):
    assert main_window._build_task_defaults({})["pause"] == {"task_type": "pause"}
    assert main_window.task_summary({"task_type": "pause"}) == "Wait for operator to resume"
    assert "pause" not in main_window.BeambotMainWindow._DRY_RUN_SUPPORTED


def test_execution_state_projects_every_control(main_window):
    expected = {
        "IDLE": (True, False, False, False, True, False),
        "RUNNING": (False, True, False, True, False, False),
        "COMPLETING_TASK": (False, False, False, True, False, False),
        "PAUSED": (False, False, True, True, False, True),
        "UNKNOWN": (False, False, False, False, False, False),
    }
    for state, controls in expected.items():
        assert main_window._execution_controls(state) == controls

    assert main_window._execution_controls("IDLE", goal_pending=True) == (
        False, False, False, True, False, False
    )


def test_active_state_clears_pending_goal_and_updates_widgets(main_window):
    BeambotMainWindow = main_window.BeambotMainWindow
    window = _window(BeambotMainWindow)
    window._goal_pending = True

    BeambotMainWindow._on_execution_state(window, "IDLE")
    assert window._goal_pending

    BeambotMainWindow._on_execution_state(window, "PAUSED")
    assert not window._goal_pending
    assert window.resume_btn.enabled
    assert window.stop_btn.enabled
    assert not window.task_toolbar.enabled
    assert not window.step_list.editing_enabled
    assert window.step_list.paused


def test_bridge_caches_and_forwards_execution_state(qapp):
    from beambot_gui.ros2_bridge import ROS2Bridge

    bridge = ROS2Bridge()
    states = []
    bridge.execution_state_changed.connect(states.append)

    bridge._on_execution_state(SimpleNamespace(data="PAUSED"))

    assert bridge.execution_state == "PAUSED"
    assert states == ["PAUSED"]
