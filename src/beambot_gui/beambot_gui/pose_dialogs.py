"""Pose management dialogs: pose list manager, single pose editor, save current pose."""

import json
from pathlib import Path

from PyQt6.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QFormLayout, QDialogButtonBox,
    QTreeWidget, QTreeWidgetItem, QLabel, QLineEdit, QDoubleSpinBox,
    QPushButton, QGroupBox, QRadioButton, QButtonGroup, QFileDialog,
    QMessageBox,
)


class PoseEditorDialog(QDialog):
    """Edit a single pose (6 joint angles in degrees)."""

    JOINT_NAMES = [
        "Shoulder Pan", "Shoulder Lift", "Elbow",
        "Wrist 1", "Wrist 2", "Wrist 3",
    ]
    PRESETS = {
        "Home": [0.0, -90.0, -90.0, -90.0, 90.0, 0.0],
        "Straight Up": [0.0, -90.0, 0.0, -90.0, 0.0, 0.0],
        "All Zero": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    }

    def __init__(self, name, values, parent=None):
        super().__init__(parent)
        self.setWindowTitle(f"Edit Pose: {name}")
        self.setMinimumWidth(350)
        self.setModal(True)
        self.result = None

        layout = QVBoxLayout(self)

        # Joint value spinboxes
        form = QFormLayout()
        self.spins = []
        for i, jname in enumerate(self.JOINT_NAMES):
            spin = QDoubleSpinBox()
            spin.setRange(-360.0, 360.0)
            spin.setDecimals(2)
            spin.setSuffix(" deg")
            spin.setValue(values[i] if i < len(values) else 0.0)
            form.addRow(f"{jname}:", spin)
            self.spins.append(spin)
        layout.addLayout(form)

        # Presets
        preset_group = QGroupBox("Presets")
        preset_layout = QHBoxLayout(preset_group)
        for pname, pvals in self.PRESETS.items():
            btn = QPushButton(pname)
            btn.clicked.connect(lambda checked, v=pvals: self._apply_preset(v))
            preset_layout.addWidget(btn)
        layout.addWidget(preset_group)

        # Buttons
        buttons = QDialogButtonBox(QDialogButtonBox.StandardButton.Save | QDialogButtonBox.StandardButton.Cancel)
        buttons.accepted.connect(self._on_save)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

    def _apply_preset(self, values):
        for spin, val in zip(self.spins, values):
            spin.setValue(val)

    def _on_save(self):
        self.result = [spin.value() for spin in self.spins]
        self.accept()


class PosesManagerDialog(QDialog):
    """Manage the pose registry: list, add, edit, delete, import.

    Every action PERSISTS IMMEDIATELY through the panel (the in-process
    registry owner), so the registry file is the single source of truth and
    the Poses tab + this dialog never diverge. The dialog returns no result.
    """

    def __init__(self, panel, parent=None):
        super().__init__(parent)
        self.panel = panel

        self.setWindowTitle("Manage Poses")
        self.setMinimumSize(600, 450)
        self.setModal(True)

        layout = QVBoxLayout(self)

        # Pose tree
        self.tree = QTreeWidget()
        self.tree.setHeaderLabels(["Pose Name", "Joint Values (degrees)"])
        self.tree.header().setStretchLastSection(True)
        self.tree.setColumnWidth(0, 180)
        self.tree.itemDoubleClicked.connect(self._edit_pose)
        layout.addWidget(self.tree, stretch=1)

        # Buttons
        btn_row = QHBoxLayout()
        for label, slot in [
            ("Add", self._add_pose),
            ("Edit", self._edit_selected),
            ("Delete", self._delete_pose),
            ("Import JSON", self._import_json),
        ]:
            btn = QPushButton(label)
            btn.clicked.connect(slot)
            btn_row.addWidget(btn)
        btn_row.addStretch()
        layout.addLayout(btn_row)

        # Dialog buttons — Close only; all edits already persisted.
        buttons = QDialogButtonBox(QDialogButtonBox.StandardButton.Close)
        buttons.rejected.connect(self.reject)
        buttons.accepted.connect(self.accept)
        layout.addWidget(buttons)

        self._refresh()

    def _refresh(self):
        self.tree.clear()
        for name, values in sorted(self.panel.get_poses().items()):
            vals_str = ", ".join(f"{v:.2f}" for v in values)
            item = QTreeWidgetItem([name, f"[{vals_str}]"])
            self.tree.addTopLevelItem(item)

    def _persist(self, name, values):
        if not self.panel.save_pose(name, values):
            QMessageBox.warning(
                self, "No Registry",
                "No pose registry file is loaded — cannot save."
            )
        self._refresh()

    def _add_pose(self):
        dlg = PoseEditorDialog("new_pose", [0.0] * 6, self)
        if dlg.exec() == QDialog.DialogCode.Accepted:
            # Ask for name
            from PyQt6.QtWidgets import QInputDialog
            name, ok = QInputDialog.getText(self, "Pose Name", "Enter pose name:")
            if ok and name:
                self._persist(name, dlg.result)

    def _edit_selected(self):
        items = self.tree.selectedItems()
        if items:
            self._edit_pose(items[0])

    def _edit_pose(self, item):
        name = item.text(0)
        values = self.panel.get_pose(name) or [0.0] * 6
        dlg = PoseEditorDialog(name, values, self)
        if dlg.exec() == QDialog.DialogCode.Accepted:
            self._persist(name, dlg.result)

    def _delete_pose(self):
        items = self.tree.selectedItems()
        if not items:
            return
        name = items[0].text(0)
        reply = QMessageBox.question(self, "Delete", f"Delete pose '{name}'?")
        if reply == QMessageBox.StandardButton.Yes:
            if not self.panel.delete_pose(name):
                QMessageBox.warning(
                    self, "No Registry",
                    "No pose registry file is loaded — cannot delete."
                )
            self._refresh()

    def _import_json(self):
        path, _ = QFileDialog.getOpenFileName(self, "Import Poses JSON", "", "JSON (*.json)")
        if not path:
            return
        try:
            with open(path) as f:
                data = json.load(f)
            imported = data.get("poses", data) if isinstance(data, dict) else {}
            count = 0
            for name, values in imported.items():
                if isinstance(values, list) and len(values) == 6:
                    if self.panel.save_pose(name, values):
                        count += 1
            self._refresh()
            QMessageBox.information(self, "Imported", f"Imported {count} pose(s)")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Import failed: {e}")


class SavePoseDialog(QDialog):
    """Save the current robot joint pose to the registry or inline (task-only)."""

    def __init__(self, current_pose, registry_file=None, parent=None):
        super().__init__(parent)
        self.current_pose = current_pose
        self.result = None

        self.setWindowTitle("Save Current Pose")
        self.setMinimumWidth(400)
        self.setModal(True)

        layout = QVBoxLayout(self)

        # Display current pose
        pose_group = QGroupBox("Current Robot Pose")
        pose_layout = QFormLayout(pose_group)
        joints = ["Shoulder Pan", "Shoulder Lift", "Elbow", "Wrist 1", "Wrist 2", "Wrist 3"]
        for jname, val in zip(joints, current_pose):
            pose_layout.addRow(f"{jname}:", QLabel(f"{val:.2f} deg"))
        layout.addWidget(pose_group)

        # Pose name
        name_group = QGroupBox("Pose Name")
        name_layout = QFormLayout(name_group)
        self.name_edit = QLineEdit("new_pose")
        name_layout.addRow("Name:", self.name_edit)
        layout.addWidget(name_group)

        # Save options: registry (shared, persisted) vs inline (this task only)
        opts_group = QGroupBox("Save To")
        opts_layout = QVBoxLayout(opts_group)
        self.btn_group = QButtonGroup(self)
        reg_name = Path(registry_file).name if registry_file else "(no registry loaded)"
        self.radio_registry = QRadioButton(f"Pose registry: {reg_name}")
        self.radio_registry.setEnabled(registry_file is not None)
        self.btn_group.addButton(self.radio_registry, 0)
        opts_layout.addWidget(self.radio_registry)

        self.radio_inline = QRadioButton("Inline (this task only)")
        self.btn_group.addButton(self.radio_inline, 1)
        opts_layout.addWidget(self.radio_inline)

        # Default to the registry when available, else inline.
        if registry_file is not None:
            self.radio_registry.setChecked(True)
        else:
            self.radio_inline.setChecked(True)
        layout.addWidget(opts_group)

        # Buttons
        buttons = QDialogButtonBox(QDialogButtonBox.StandardButton.Save | QDialogButtonBox.StandardButton.Cancel)
        buttons.accepted.connect(self._on_save)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

    def _on_save(self):
        name = self.name_edit.text().strip()
        if not name:
            QMessageBox.warning(self, "Error", "Pose name cannot be empty")
            return

        if self.btn_group.checkedId() == 0:
            action = "save_to_registry"
        else:
            action = "save_inline"
        self.result = {"action": action, "pose_name": name, "pose_values": self.current_pose}
        self.accept()
