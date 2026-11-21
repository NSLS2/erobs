"""Entrypoint cleanup without constructing Qt windows or ROS clients."""

import sys
from types import ModuleType, SimpleNamespace
from unittest.mock import Mock

import pytest

pytest.importorskip("PyQt6.QtWidgets")

from beambot_gui import main as entrypoint  # noqa: E402


@pytest.mark.parametrize("failure", [None, "init", "window", "show", "event_loop"])
def test_main_shuts_down_once_on_exit_or_failure(failure, monkeypatch):
    app, bridge, window = Mock(), Mock(), Mock()
    app.exec.return_value = 7
    bridge.init_ros2.return_value = False  # Offline startup still opens the editor.
    window_factory = Mock(return_value=window)
    window_module = ModuleType("beambot_gui.main_window")
    window_module.BeambotMainWindow = window_factory
    monkeypatch.setitem(sys.modules, "beambot_gui.main_window", window_module)
    monkeypatch.setattr(entrypoint, "QApplication", Mock(return_value=app))
    monkeypatch.setattr(entrypoint, "ROS2Bridge", Mock(return_value=bridge))
    monkeypatch.setattr(entrypoint.theme, "apply", Mock())

    if failure:
        error = RuntimeError(failure)
        {
            "init": bridge.init_ros2,
            "window": window_factory,
            "show": window.show,
            "event_loop": app.exec,
        }[failure].side_effect = error
        with pytest.raises(RuntimeError) as raised:
            entrypoint.main()
        assert raised.value is error
    else:
        with pytest.raises(SystemExit) as raised:
            entrypoint.main()
        assert raised.value.code == 7
        window_factory.assert_called_once_with(bridge)
        window.show.assert_called_once_with()
        app.exec.assert_called_once_with()

    bridge.init_ros2.assert_called_once_with()
    bridge.shutdown.assert_called_once_with()


def test_window_close_leaves_ros_cleanup_to_entrypoint():
    from beambot_gui.main_window import BeambotMainWindow

    window = SimpleNamespace(agent_bridge=Mock(), ros2=Mock())
    event = Mock()
    BeambotMainWindow.closeEvent(window, event)

    window.agent_bridge.disconnect.assert_called_once_with()
    window.ros2.shutdown.assert_not_called()
    event.accept.assert_called_once_with()
