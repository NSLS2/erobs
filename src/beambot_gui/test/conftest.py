"""Headless Qt fixtures; window tests never start ROS, agents, or WebEngine."""

import importlib.util
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace

import pytest


@pytest.fixture(scope="session")
def qapp():
    with pytest.MonkeyPatch.context() as patch:
        patch.setenv("QT_QPA_PLATFORM", "offscreen")
        widgets = pytest.importorskip("PyQt6.QtWidgets")
        from PyQt6.QtCore import Qt

        widgets.QApplication.setAttribute(Qt.ApplicationAttribute.AA_ShareOpenGLContexts)
        app = widgets.QApplication.instance() or widgets.QApplication([])
        yield app


@pytest.fixture
def main_window(monkeypatch, qapp):
    """Load real window methods without retaining optional ROS import stubs."""
    try:
        from action_msgs.msg import GoalStatus  # noqa: F401
    except ImportError:
        package = ModuleType("action_msgs")
        package.msg = ModuleType("action_msgs.msg")
        package.msg.GoalStatus = SimpleNamespace(STATUS_SUCCEEDED=4, STATUS_CANCELED=5)
        monkeypatch.setitem(sys.modules, "action_msgs", package)
        monkeypatch.setitem(sys.modules, "action_msgs.msg", package.msg)

    spec = importlib.util.spec_from_file_location(
        "beambot_gui._test_main_window",
        Path(__file__).parents[1] / "beambot_gui" / "main_window.py",
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, "WEBENGINE_AVAILABLE", False)
    monkeypatch.setattr(module.AgentBridge, "connect_agent", lambda self: None)
    monkeypatch.setattr(module.ROS2Bridge, "shutdown", lambda self: None)
    monkeypatch.setattr(module.BeambotMainWindow, "_load_beamline_yaml", lambda self: ({}, None))
    return module
