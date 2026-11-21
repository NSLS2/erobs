"""Headless check for VisionMoveTo vision_method-specific options."""

import pytest

pytest.importorskip("PyQt6.QtWidgets", reason="PyQt6 not installed")

from beambot_gui.task_forms import VisionMoveToForm  # noqa: E402

pytestmark = pytest.mark.usefixtures("qapp")


def test_sample_roi_options_are_shown_and_saved_exclusively():
    form = VisionMoveToForm(
        {"task_type": "vision_moveto", "vision_method": "marker"}, 0, {}
    )
    assert form.marker_dict.isVisibleTo(form)
    assert not form.sample_roi_group.isVisibleTo(form)

    form.vision_method_combo.setCurrentText("sample_roi")
    form.strategy.setCurrentText("nearest_edge")
    form.edge_inset.setValue(4.25)

    assert not form.marker_dict.isVisibleTo(form)
    assert form.sample_roi_group.isVisibleTo(form)
    result = form.collect_values()
    assert result["strategy"] == "nearest_edge"
    assert result["edge_inset_mm"] == 4.25
    assert "marker_dictionary" not in result

    form.vision_method_combo.setCurrentText("marker")
    result = form.collect_values()
    assert "marker_dictionary" in result
    assert "strategy" not in result
    assert "edge_inset_mm" not in result
