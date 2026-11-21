"""Headless checks for configurable rack task forms."""

import pytest

pytest.importorskip("PyQt6.QtWidgets", reason="PyQt6 not installed")

import beambot_gui.task_forms as task_forms  # noqa: E402

pytestmark = pytest.mark.usefixtures("qapp")


TIP_CONFIG = {
    "mode": "grid",
    "marker_id": 10,
    "scan_pose": "pipettor_scan",
    "marker_offset": {"x": 0.0, "y": -0.062},
    "grid": {
        "rows": 8,
        "cols": 12,
        "pitch": 0.0091,
        "row_direction": "up",
        "col_direction": "left",
    },
    "moves": [
        {"direction": "forward", "distance": 0.05},
        "column_offset",
        "row_offset",
        {"direction": "forward", "distance": 0.031},
        {"direction": "backward", "distance": 0.158},
    ],
}


def test_tip_grid_is_legible_and_config_is_saved(monkeypatch):
    monkeypatch.setattr(task_forms, "_vision_target_config", lambda _: TIP_CONFIG)
    form = task_forms.PickupTipForm(
        {"task_type": "pickup_tip", "row": 0, "col": 11}, 0, {}
    )

    cell = form._grid_buttons[(0, 11)]
    assert cell.text() == "A12"
    assert cell.width() >= 52
    assert cell.height() >= 36
    assert cell.font().pointSize() >= 10

    form._col_pitch.setValue(0.012)
    form._move_table.cellWidget(0, 2).setValue(0.06)
    result = form.collect_values()
    assert result["config"]["grid"]["col_pitch"] == 0.012
    assert result["config"]["moves"][0] == {
        "direction": "forward",
        "distance": 0.06,
    }


def test_grid_dimensions_rebuild_position_picker(monkeypatch):
    monkeypatch.setattr(task_forms, "_vision_target_config", lambda _: TIP_CONFIG)
    form = task_forms.PickupTipForm(
        {"task_type": "pickup_tip", "row": 7, "col": 11}, 0, {}
    )

    form._row_count.setValue(2)
    form._col_count.setValue(3)

    assert len(form._grid_buttons) == 6
    assert (form._selected_row, form._selected_col) == (1, 2)
    assert form.collect_values()["position"] == "B3"


def test_vial_rinse_settings_are_saved(monkeypatch):
    monkeypatch.setattr(task_forms, "_vision_target_config", lambda _: TIP_CONFIG)
    form = task_forms.PickupVialForm(
        {"task_type": "pickup_vial", "row": 0, "col": 0}, 0, {}
    )

    assert not form._rinse_count.isEnabled()
    form._vial_operation.setCurrentText("RINSE")
    assert form._rinse_count.isEnabled()
    form._rinse_count.setValue(3)
    result = form.collect_values()

    assert result["pipettor_operation"] == "RINSE"
    assert result["rinse_count"] == 3
