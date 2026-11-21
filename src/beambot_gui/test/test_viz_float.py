"""Exercise production detach/redock methods with an ordinary QWidget viewer."""

import pytest

pytest.importorskip("PyQt6.QtWidgets", reason="PyQt6 not installed")

from PyQt6.QtWidgets import QTabWidget, QWidget  # noqa: E402

from beambot_gui.task_forms import MoveToForm  # noqa: E402


class FakeWindow:
    """Minimal stand-in exposing the attributes the helpers use."""

    def __init__(self):
        self.right_tabs = QTabWidget()
        chat = QWidget()
        self.viz_panel = QWidget()
        self.right_tabs.addTab(chat, "Chat")
        self._viz_tab_title = "3D View"
        self.right_tabs.addTab(self.viz_panel, self._viz_tab_title)
        self._viz_floating = False
        self._viz_home_index = -1


@pytest.fixture
def window(main_window, monkeypatch):
    monkeypatch.setattr(main_window, "WEBENGINE_AVAILABLE", True)
    w = FakeWindow()
    w._detach_viz_for_form = main_window.BeambotMainWindow._detach_viz_for_form.__get__(w)
    w._redock_viz_after_form = main_window.BeambotMainWindow._redock_viz_after_form.__get__(w)
    yield w
    w.viz_panel.close()
    w.right_tabs.close()


def test_detach_removes_from_tab(window):
    w = window
    assert w.right_tabs.indexOf(w.viz_panel) == 1
    w._detach_viz_for_form()
    assert w.right_tabs.indexOf(w.viz_panel) == -1
    assert w._viz_floating is True


def test_double_detach_is_noop(window):
    w = window
    w._detach_viz_for_form()
    w._detach_viz_for_form()
    assert w._viz_floating is True
    assert w.right_tabs.indexOf(w.viz_panel) == -1


def test_redock_without_detach_is_noop(window):
    w = window
    original_idx = w.right_tabs.indexOf(w.viz_panel)
    w._redock_viz_after_form()
    assert w._viz_floating is False
    assert w.right_tabs.indexOf(w.viz_panel) == original_idx


def test_full_cycle_preserves_panel(window):
    """Full detach→embed→done→redock cycle: panel survives and returns home."""
    w = window
    original_idx = w.right_tabs.indexOf(w.viz_panel)
    panel_id = id(w.viz_panel)

    w._detach_viz_for_form()
    dialog = MoveToForm({"task_type": "moveto"}, 0, {}, viz_widget=w.viz_panel)
    assert w.viz_panel.parent() is not None
    dialog.done(0)
    assert w.viz_panel.parent() is None

    w._redock_viz_after_form()
    assert w.right_tabs.indexOf(w.viz_panel) == original_idx
    assert id(w.viz_panel) == panel_id
    assert w._viz_floating is False
