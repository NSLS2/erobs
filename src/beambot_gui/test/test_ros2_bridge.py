import time
from concurrent.futures import Future
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

def test_missing_action_client_emits_one_result(qapp):
    from beambot_gui.ros2_bridge import ROS2Bridge

    bridge = ROS2Bridge()
    results = []
    bridge.action_result_received.connect(lambda *result: results.append(result))

    bridge.execute_task("{}")

    deadline = time.monotonic() + 2
    while not results and time.monotonic() < deadline:
        qapp.processEvents()
        time.sleep(0.01)

    assert results == [(0, "Action client unavailable", 0, 0)]


@pytest.mark.parametrize("before_acceptance", [False, True])
@pytest.mark.parametrize("outcome", ["cancelled", "succeeded", "cancel_error", "cancel_rejected"])
def test_stop_waits_for_actual_result(qapp, monkeypatch, before_acceptance, outcome):
    from beambot_gui import ros2_bridge

    bridge = ros2_bridge.ROS2Bridge()
    results, logs = [], []
    bridge.action_result_received.connect(lambda *result: results.append(result))
    bridge.log.connect(logs.append)
    send_future, result_future, cancel_future = Future(), Future(), Future()
    cancel_future.set_result(SimpleNamespace(return_code=1 if outcome == "cancel_rejected" else 0))
    handle = SimpleNamespace(
        accepted=True,
        get_result_async=Mock(return_value=result_future),
        cancel_goal_async=Mock(return_value=cancel_future),
    )
    if outcome == "cancel_error":
        handle.cancel_goal_async.side_effect = RuntimeError("cancel transport failed")
    if not before_acceptance:
        send_future.set_result(handle)
    bridge._action_client = SimpleNamespace(
        wait_for_server=Mock(return_value=True),
        send_goal_async=Mock(return_value=send_future),
    )
    monkeypatch.setattr(ros2_bridge, "BeambotExecution", SimpleNamespace(Goal=SimpleNamespace), raising=False)
    monkeypatch.setattr(ros2_bridge, "threading", SimpleNamespace(
        Thread=lambda target, daemon: SimpleNamespace(start=target),
    ))
    status = 5 if outcome == "cancelled" else 4
    ticks = 0

    def advance(_seconds):
        nonlocal ticks
        ticks += 1
        assert results == []
        if ticks == 1:
            bridge.stop_execution()
        elif ticks == 2:
            if not send_future.done():
                send_future.set_result(handle)
        elif ticks == 3:
            assert bridge._current_goal_handle is handle
            handle.cancel_goal_async.assert_called_once_with()
            bridge.stop_execution()
        elif ticks == 4:
            handle.cancel_goal_async.assert_called_once_with()
            result_future.set_result(SimpleNamespace(
                status=status,
                result=SimpleNamespace(error_message="backend result", completed_steps=2, total_steps=3),
            ))
        else:
            pytest.fail("Worker did not finish after the terminal result")

    monkeypatch.setattr(ros2_bridge, "time", SimpleNamespace(sleep=advance))
    bridge.execute_task("{}")

    assert results == [(status, "backend result", 2, 3)]
    assert bridge._current_goal_handle is None
    handle.cancel_goal_async.assert_called_once_with()
    if outcome == "cancel_error":
        assert any("cancel transport failed" in message for message in logs)


def test_stop_while_waiting_for_server_does_not_send_goal(qapp, monkeypatch):
    from beambot_gui import ros2_bridge

    bridge = ros2_bridge.ROS2Bridge()
    results = []
    bridge.action_result_received.connect(lambda *result: results.append(result))
    bridge._action_client = SimpleNamespace(
        wait_for_server=lambda **_kwargs: bridge.stop_execution() or True,
        send_goal_async=Mock(),
    )
    monkeypatch.setattr(ros2_bridge, "GoalStatus", SimpleNamespace(STATUS_CANCELED=5), raising=False)
    monkeypatch.setattr(ros2_bridge, "threading", SimpleNamespace(
        Thread=lambda target, daemon: SimpleNamespace(start=target),
    ))

    bridge.execute_task("{}")

    bridge._action_client.send_goal_async.assert_not_called()
    assert results == [(5, "Stopped before sending goal", 0, 0)]
