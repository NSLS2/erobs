"""Task-editing dialogs for Beambot tasks."""

import copy

from PyQt6.QtWidgets import (
    QDialog,
    QVBoxLayout,
    QFormLayout,
    QDialogButtonBox,
    QScrollArea,
    QWidget,
    QLabel,
    QLineEdit,
    QComboBox,
    QCheckBox,
    QDoubleSpinBox,
    QSpinBox,
    QGroupBox,
    QHBoxLayout,
    QTextEdit,
    QMessageBox,
    QGridLayout,
    QPushButton,
    QSizePolicy,
    QTabWidget,
    QTableWidget,
    QAbstractItemView,
    QHeaderView,
    QLayout,
)
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QFont


# --- Beamline-driven enumerations -----------------------------------------
# Use the cached beamline configuration; unavailable settings return empty values.


def _configured_grippers() -> list[str]:
    """All gripper names declared in the active beamline YAML."""
    try:
        from beambot.config_loader import load_beamline_config

        config, _ = load_beamline_config()
        return list(config.get("grippers", {}).keys())
    except Exception:
        return []


def _grippers_with_states() -> list[str]:
    """Grippers whose YAML config declares a non-empty `states:` block.

    Excludes grippers like `none` and `pipettor` that have no end-effector
    actions, so the EndEffectorForm only offers grippers that can act.
    """
    try:
        from beambot.config_loader import load_beamline_config

        config, _ = load_beamline_config()
        return [
            name
            for name, gconf in config.get("grippers", {}).items()
            if gconf.get("states")
        ]
    except Exception:
        return []


def _end_effector_actions() -> list[str]:
    """Union of all `states.grasp` + `states.release` action names across grippers."""
    try:
        from beambot.config_loader import load_beamline_config

        config, _ = load_beamline_config()
        actions: list[str] = []
        for gconf in config.get("grippers", {}).values():
            states = gconf.get("states") or {}
            for key in ("grasp", "release"):
                val = states.get(key)
                if val and val not in actions:
                    actions.append(val)
        return actions
    except Exception:
        return []


def _vision_target_config(target_name: str) -> dict:
    """Load a vision target config."""
    try:
        from beambot.config_loader import load_beamline_config

        config, _ = load_beamline_config()
        targets = config.get("vision_targets", {})
        return targets.get(target_name, {})
    except Exception:
        return {}


# --- Dispatch ---


def open_task_form(
    step,
    step_index,
    poses,
    parent=None,
    current_pose=None,
    preview_cb=None,
    end_preview_cb=None,
    viz_widget=None,
):
    """Open the edit dialog for a task step. Returns edited step dict or None if cancelled."""
    task_type = step.get("task_type", "")
    form_cls = _FORMS.get(task_type)
    if not form_cls:
        QMessageBox.warning(parent, "Unknown", f"No form for task type: {task_type}")
        return None
    if form_cls is MoveToForm:
        dialog = form_cls(
            step,
            step_index,
            poses,
            parent,
            current_pose=current_pose,
            preview_cb=preview_cb,
            end_preview_cb=end_preview_cb,
            viz_widget=viz_widget,
        )
    else:
        dialog = form_cls(step, step_index, poses, parent)
    if dialog.exec() == QDialog.DialogCode.Accepted:
        return dialog.result
    return None


# --- Base Form ---


class BaseTaskForm(QDialog):
    TITLE = "Edit Step"

    def __init__(self, step, step_index, poses, parent=None):
        super().__init__(parent)
        self.step = copy.deepcopy(step)
        self.poses = poses
        self.result = None

        self.setWindowTitle(f"{self.TITLE} (Step {step_index + 1})")
        self.setMinimumWidth(480)
        self.setModal(True)

        outer = QVBoxLayout(self)

        # Title
        title = QLabel(self.TITLE)
        title.setStyleSheet("font-size: 14px; font-weight: bold;")
        outer.addWidget(title)

        # Scrollable form area
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        form_widget = QWidget()
        self.form = QFormLayout(form_widget)
        scroll.setWidget(form_widget)
        content_row = QHBoxLayout()
        content_row.addWidget(scroll, stretch=1)
        outer.addLayout(content_row, stretch=1)
        self._content_row = content_row

        self.build_form()

        # Buttons
        buttons = QDialogButtonBox(
            QDialogButtonBox.StandardButton.Save
            | QDialogButtonBox.StandardButton.Cancel
        )
        buttons.accepted.connect(self._on_save)
        buttons.rejected.connect(self.reject)
        outer.addWidget(buttons)

    def build_form(self):
        pass

    def collect_values(self):
        return self.step

    def _on_save(self):
        try:
            self.result = self.collect_values()
            self.accept()
        except (ValueError, KeyError) as e:
            QMessageBox.critical(self, "Invalid Input", str(e))

    # --- Helpers ---

    def add_combo(self, label, key, options):
        combo = QComboBox()
        combo.addItems(options)
        current = str(self.step.get(key, options[0] if options else ""))
        idx = combo.findText(current)
        if idx >= 0:
            combo.setCurrentIndex(idx)
        self.form.addRow(label, combo)
        return combo

    def add_text(self, label, key, default=""):
        edit = QLineEdit(str(self.step.get(key, default)))
        self.form.addRow(label, edit)
        return edit

    def add_float(
        self, label, key, default=0.0, min_val=-999.0, max_val=999.0, decimals=4
    ):
        spin = QDoubleSpinBox()
        spin.setRange(min_val, max_val)
        spin.setDecimals(decimals)
        spin.setValue(float(self.step.get(key, default)))
        self.form.addRow(label, spin)
        return spin

    def add_int(self, label, key, default=0, min_val=0, max_val=999):
        spin = QSpinBox()
        spin.setRange(min_val, max_val)
        spin.setValue(int(self.step.get(key, default)))
        self.form.addRow(label, spin)
        return spin

    def add_check(self, label, key, default=False):
        check = QCheckBox()
        check.setChecked(bool(self.step.get(key, default)))
        self.form.addRow(label, check)
        return check

    def add_section(self, title):
        group = QGroupBox(title)
        layout = QFormLayout(group)
        self.form.addRow(group)
        return layout

    def add_hint(self, text):
        lbl = QLabel(text)
        lbl.setStyleSheet("color: gray; font-size: 10px;")
        lbl.setWordWrap(True)
        self.form.addRow(lbl)

    def add_offset_row(self, section, prefix, keys):
        """Add X/Y/Z offset fields in a horizontal row."""
        row = QHBoxLayout()
        spins = {}
        for axis, key in zip(["X:", "Y:", "Z:"], keys):
            row.addWidget(QLabel(axis))
            spin = QDoubleSpinBox()
            spin.setRange(-1.0, 1.0)
            spin.setDecimals(4)
            spin.setValue(float(self.step.get(key, 0.0)))
            spin.setMaximumWidth(90)
            row.addWidget(spin)
            spins[key] = spin
        section.addRow(prefix, row)
        return spins


# --- Concrete Forms ---


class MoveToForm(BaseTaskForm):
    TITLE = "MoveTo Configuration"

    _MODES = ["Named target", "Relative move", "Cartesian target", "Joint values"]

    _JOINT_NAMES = [
        "Shoulder Pan",
        "Shoulder Lift",
        "Elbow",
        "Wrist 1",
        "Wrist 2",
        "Wrist 3",
    ]
    _JOINT_PRESETS = {
        "Home": [0.0, -90.0, -90.0, -90.0, 90.0, 0.0],
        "Straight Up": [0.0, -90.0, 0.0, -90.0, 0.0, 0.0],
        "All Zero": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    }

    def __init__(
        self,
        step,
        step_index,
        poses,
        parent=None,
        current_pose=None,
        preview_cb=None,
        end_preview_cb=None,
        viz_widget=None,
    ):
        self._current_pose = current_pose
        self._step_index = step_index
        self._preview_cb = preview_cb
        self._end_preview_cb = end_preview_cb
        self._viz_widget = viz_widget
        super().__init__(step, step_index, poses, parent)

    def _detect_mode(self):
        """Match backend precedence: relative > cartesian > joint > named."""
        if self.step.get("direction") and float(self.step.get("distance", 0)) != 0:
            return "Relative move"
        if self.step.get("cartesian_target"):
            return "Cartesian target"
        target = self.step.get("target", "")
        if target and self.poses.get(target) and len(self.poses[target]) == 6:
            return "Joint values"
        return "Named target"

    def build_form(self):
        # Mode selector
        self.mode_combo = QComboBox()
        self.mode_combo.addItems(self._MODES)
        self.mode_combo.setCurrentText(self._detect_mode())
        self.form.addRow("Move mode:", self.mode_combo)

        self._preview = QLabel("")
        self._preview.setStyleSheet("color: gray; font-size: 10px;")
        self.form.addRow(self._preview)

        # --- Named target section ---
        self._named_group = QGroupBox("Named Target")
        named_layout = QFormLayout(self._named_group)
        self.form.addRow(self._named_group)
        self.target = QComboBox()
        self.target.setEditable(True)
        self.target.setInsertPolicy(QComboBox.InsertPolicy.NoInsert)
        pose_names = sorted(self.poses.keys())
        self.target.addItems(pose_names)
        for sentinel in ("moveit_home",):
            if self.target.findText(sentinel) < 0:
                self.target.addItem(sentinel)
        current = str(self.step.get("target", "moveit_home"))
        i = self.target.findText(current)
        if i >= 0:
            self.target.setCurrentIndex(i)
        else:
            self.target.setEditText(current)
        named_layout.addRow("Target:", self.target)
        hint = QLabel("Pick a pose or type an SRDF named state (e.g. moveit_home)")
        hint.setStyleSheet("color: gray; font-size: 10px;")
        named_layout.addRow(hint)

        # --- Planning type (cartesian mode only) ---
        self._planning_group = QGroupBox()
        self._planning_group.setFlat(True)
        self._planning_group.setStyleSheet("QGroupBox{border:none;}")
        pl = QFormLayout(self._planning_group)
        self.form.addRow(self._planning_group)
        self.planning = QComboBox()
        self.planning.addItems(["joint", "cartesian"])
        current_pt = str(self.step.get("planning_type", "joint"))
        idx = self.planning.findText(current_pt)
        if idx >= 0:
            self.planning.setCurrentIndex(idx)
        pl.addRow("Planning Type:", self.planning)

        # --- Relative move section ---
        self._rel_group = QGroupBox("Relative Move")
        rel = QFormLayout(self._rel_group)
        self.form.addRow(self._rel_group)
        self.direction = QComboBox()
        self.direction.addItems(["forward", "backward", "left", "right", "up", "down"])
        self.direction.setCurrentText(self.step.get("direction", "forward"))
        rel.addRow("Direction:", self.direction)
        self.distance = QDoubleSpinBox()
        self.distance.setRange(0, 2.0)
        self.distance.setDecimals(3)
        self.distance.setValue(float(self.step.get("distance", 0.05)))
        rel.addRow("Distance (m):", self.distance)

        # --- Cartesian target section ---
        self._cart_group = QGroupBox("Cartesian Target")
        cart = QFormLayout(self._cart_group)
        self.form.addRow(self._cart_group)
        existing = self.step.get("cartesian_target", [])
        self.cart_x = QDoubleSpinBox()
        self.cart_x.setRange(-2, 2)
        self.cart_x.setDecimals(4)
        self.cart_y = QDoubleSpinBox()
        self.cart_y.setRange(-2, 2)
        self.cart_y.setDecimals(4)
        self.cart_z = QDoubleSpinBox()
        self.cart_z.setRange(-2, 2)
        self.cart_z.setDecimals(4)
        pos_row = QHBoxLayout()
        for lbl, spin, i in [
            ("X:", self.cart_x, 0),
            ("Y:", self.cart_y, 1),
            ("Z:", self.cart_z, 2),
        ]:
            pos_row.addWidget(QLabel(lbl))
            if len(existing) > i:
                spin.setValue(existing[i])
            pos_row.addWidget(spin)
        cart.addRow("Position (m):", pos_row)

        self.cart_r = QDoubleSpinBox()
        self.cart_r.setRange(-360, 360)
        self.cart_r.setDecimals(2)
        self.cart_p = QDoubleSpinBox()
        self.cart_p.setRange(-360, 360)
        self.cart_p.setDecimals(2)
        self.cart_yaw = QDoubleSpinBox()
        self.cart_yaw.setRange(-360, 360)
        self.cart_yaw.setDecimals(2)
        rot_row = QHBoxLayout()
        for lbl, spin, i in [
            ("R:", self.cart_r, 3),
            ("P:", self.cart_p, 4),
            ("Y:", self.cart_yaw, 5),
        ]:
            rot_row.addWidget(QLabel(lbl))
            if len(existing) > i:
                spin.setValue(existing[i])
            rot_row.addWidget(spin)
        cart.addRow("Orientation (deg):", rot_row)

        self.frame_id = QComboBox()
        self.frame_id.addItems(["base_link", "flange"])
        self.frame_id.setCurrentText(self.step.get("frame_id", "base_link"))
        cart.addRow("Frame:", self.frame_id)

        # --- Joint values section ---
        self._joint_group = QGroupBox("Joint Values")
        jl = QFormLayout(self._joint_group)
        self.form.addRow(self._joint_group)

        # Load existing joint values if editing a joint-mode step
        existing_jv = []
        target = self.step.get("target", "")
        if target and self.poses.get(target) and len(self.poses[target]) == 6:
            existing_jv = self.poses[target]

        self.joint_spins = []
        for i, jname in enumerate(self._JOINT_NAMES):
            spin = QDoubleSpinBox()
            spin.setRange(-360.0, 360.0)
            spin.setDecimals(2)
            spin.setSuffix(" deg")
            if i < len(existing_jv):
                spin.setValue(existing_jv[i])
            jl.addRow(f"{jname}:", spin)
            self.joint_spins.append(spin)

        # Presets row
        preset_row = QHBoxLayout()
        for pname, pvals in self._JOINT_PRESETS.items():
            btn = QPushButton(pname)
            btn.clicked.connect(lambda checked, v=pvals: self._apply_joint_preset(v))
            preset_row.addWidget(btn)
        jl.addRow("Presets:", preset_row)

        # Read current pose button
        self._read_pose_btn = QPushButton("Read current pose")
        self._read_pose_btn.clicked.connect(self._read_current_pose)
        jl.addRow(self._read_pose_btn)

        # Wire mode switching
        self.mode_combo.currentTextChanged.connect(self._apply_mode)
        self._apply_mode(self.mode_combo.currentText())

        # Wire live preview callbacks
        for spin in self.joint_spins:
            spin.valueChanged.connect(self._emit_preview)
        self.target.currentTextChanged.connect(self._emit_preview)
        self._emit_preview()

        # Embed the 3D viewer to the right of the form
        if self._viz_widget is not None:
            self._viz_widget.setParent(None)
            self._content_row.addWidget(self._viz_widget, stretch=1)
            self._viz_widget.show()
            self.setMinimumWidth(900)
            self.setMinimumHeight(520)

    def _apply_joint_preset(self, values):
        for spin, val in zip(self.joint_spins, values):
            spin.setValue(val)

    def _read_current_pose(self):
        if self._current_pose is None:
            QMessageBox.warning(
                self, "No Pose", "No robot pose available. Is the robot connected?"
            )
            return
        for spin, val in zip(self.joint_spins, self._current_pose):
            spin.setValue(val)

    def _apply_mode(self, mode):
        self._named_group.setVisible(mode == "Named target")
        self._rel_group.setVisible(mode == "Relative move")
        self._cart_group.setVisible(mode == "Cartesian target")
        self._planning_group.setVisible(mode == "Cartesian target")
        self._joint_group.setVisible(mode == "Joint values")
        previews = {
            "Named target": f"Will execute: move to '{self.target.currentText()}'",
            "Relative move": "Will execute: relative move in selected direction",
            "Cartesian target": "Will execute: move to XYZ coordinates",
            "Joint values": "Will execute: move to explicit joint angles",
        }
        self._preview.setText(previews.get(mode, ""))
        self._emit_preview()

    def _emit_preview(self, _=None):
        """Push the current goal pose to the 3D viewer preview callback."""
        if not self._preview_cb:
            return
        mode = self.mode_combo.currentText()
        if mode == "Joint values":
            self._preview_cb([s.value() for s in self.joint_spins])
        elif mode == "Named target":
            pose = self.poses.get(self.target.currentText())
            if isinstance(pose, (list, tuple)) and len(pose) == 6:
                self._preview_cb(list(pose))
            elif self._end_preview_cb:
                self._end_preview_cb()
        else:
            if self._end_preview_cb:
                self._end_preview_cb()

    def done(self, r):
        if self._end_preview_cb:
            self._end_preview_cb()
        if self._viz_widget is not None:
            self._viz_widget.setParent(None)
        super().done(r)

    def collect_values(self):
        s = {**self.step}
        mode = self.mode_combo.currentText()

        # Always strip all branches, then add back only the active one
        for k in (
            "target",
            "direction",
            "distance",
            "cartesian_target",
            "frame_id",
            "planning_type",
            "_inline_joint_pose",
        ):
            s.pop(k, None)

        if mode == "Named target":
            s["target"] = self.target.currentText()
            s["planning_type"] = "joint"
        elif mode == "Relative move":
            s["direction"] = self.direction.currentText()
            s["distance"] = self.distance.value()
        elif mode == "Cartesian target":
            s["planning_type"] = self.planning.currentText()
            x, y, z = self.cart_x.value(), self.cart_y.value(), self.cart_z.value()
            cart = [x, y, z]
            r, p, yaw = self.cart_r.value(), self.cart_p.value(), self.cart_yaw.value()
            if r != 0 or p != 0 or yaw != 0:
                cart.extend([r, p, yaw])
            s["cartesian_target"] = cart
            s["frame_id"] = self.frame_id.currentText()
        elif mode == "Joint values":
            # Reuse existing inline name or generate one from step index
            target = self.step.get("target", "")
            if not (target and self.poses.get(target) and len(self.poses[target]) == 6):
                target = f"moveto_joints_{self._step_index + 1}"
            s["target"] = target
            s["planning_type"] = "joint"
            values = [spin.value() for spin in self.joint_spins]
            s["_inline_joint_pose"] = {"name": target, "values": values}
        return s


class SampleForm(BaseTaskForm):
    """Shared form for pick_sample and place_sample."""

    def __init__(self, step, step_index, poses, parent=None):
        self.mode = "pick" if step.get("task_type") == "pick_sample" else "place"
        self.TITLE = "Pick Sample" if self.mode == "pick" else "Place Sample"
        super().__init__(step, step_index, poses, parent)

    def build_form(self):
        self.use_vision = self.add_check("Use Vision:", "use_vision", True)

        # Vision fields
        self.vision_group = QGroupBox("Vision Mode")
        vl = QFormLayout(self.vision_group)
        self.form.addRow(self.vision_group)

        self.vision_method_combo = QComboBox()
        self.vision_method_combo.addItems(["marker", "sample_roi"])
        # detection_type is the deprecated key in older task files.
        self.vision_method_combo.setCurrentText(
            self.step.get("vision_method", self.step.get("detection_type", "marker"))
        )
        vl.addRow("Vision Method:", self.vision_method_combo)

        self.tag_id = QSpinBox()
        self.tag_id.setRange(0, 999)
        self.tag_id.setValue(int(self.step.get("tag_id", 0)))
        vl.addRow("Tag ID:", self.tag_id)

        self.scan_pose = QLineEdit(self.step.get("scan_pose", ""))
        vl.addRow("Scan Pose:", self.scan_pose)

        self.z_offset = QDoubleSpinBox()
        self.z_offset.setRange(-0.1, 0.1)
        self.z_offset.setDecimals(4)
        self.z_offset.setValue(float(self.step.get("z_offset", 0.0)))
        vl.addRow("Z Offset (m):", self.z_offset)

        self.marker_offsets = self.add_offset_row(
            vl,
            "Marker Offsets (m):",
            ["marker_offset_x", "marker_offset_y", "marker_offset_z"],
        )

        self.offset_dir = QComboBox()
        self.offset_dir.addItems(
            ["", "forward", "backward", "left", "right", "up", "down"]
        )
        self.offset_dir.setCurrentText(self.step.get("offset_direction", ""))
        vl.addRow("Flange Direction:", self.offset_dir)
        self.offset_dist = QDoubleSpinBox()
        self.offset_dist.setRange(0, 1.0)
        self.offset_dist.setDecimals(4)
        self.offset_dist.setValue(float(self.step.get("offset_distance", 0.0)))
        vl.addRow("Flange Distance (m):", self.offset_dist)

        # Hardcoded fields
        self.hardcoded_group = QGroupBox("Hardcoded Mode")
        hl = QFormLayout(self.hardcoded_group)
        self.form.addRow(self.hardcoded_group)

        self.approach_pose = QLineEdit(self.step.get("approach_pose", ""))
        hl.addRow("Approach Pose:", self.approach_pose)
        self.target_pose = QLineEdit(self.step.get("target_pose", ""))
        hl.addRow("Target Pose:", self.target_pose)

        # Toggle visibility
        def toggle(checked):
            self.vision_group.setVisible(checked)
            self.hardcoded_group.setVisible(not checked)

        self.use_vision.stateChanged.connect(
            lambda state: toggle(state == Qt.CheckState.Checked.value)
        )
        toggle(self.use_vision.isChecked())

    def collect_values(self):
        s = {**self.step}
        s["use_vision"] = self.use_vision.isChecked()
        if s["use_vision"]:
            s["vision_method"] = self.vision_method_combo.currentText()
            s["tag_id"] = self.tag_id.value()
            s["scan_pose"] = self.scan_pose.text()
            s["z_offset"] = self.z_offset.value()
            for key, spin in self.marker_offsets.items():
                v = spin.value()
                if v != 0.0:
                    s[key] = v
                else:
                    s.pop(key, None)
            od = self.offset_dir.currentText()
            if od:
                s["offset_direction"] = od
                s["offset_distance"] = self.offset_dist.value()
            else:
                s.pop("offset_direction", None)
                s.pop("offset_distance", None)
            s.pop("approach_pose", None)
            s.pop("target_pose", None)
        else:
            s["approach_pose"] = self.approach_pose.text()
            s["target_pose"] = self.target_pose.text()
            for key in (
                "vision_method",
                "tag_id",
                "sample_index",
                "scan_pose",
                "z_offset",
                "marker_offset_x",
                "marker_offset_y",
                "marker_offset_z",
                "offset_direction",
                "offset_distance",
            ):
                s.pop(key, None)
        return s


class VisionScanForm(BaseTaskForm):
    TITLE = "Vision Scan Configuration"

    def build_form(self):
        self.add_hint(
            "Scans markers from multiple positions and caches averaged poses "
            "in the scan server."
        )
        self.form.addRow(QLabel("Scan Positions (one pose name per line):"))
        self.positions_text = QTextEdit()
        self.positions_text.setMaximumHeight(120)
        existing = self.step.get("scan_positions", [])
        if existing:
            self.positions_text.setPlainText("\n".join(existing))
        self.form.addRow(self.positions_text)
        self.spp = self.add_int("Scans Per Position:", "scans_per_position", 3, 1, 20)
        self.timeout = self.add_float("Timeout (s):", "timeout", 10.0, 1.0, 120.0, 1)

    def collect_values(self):
        raw = self.positions_text.toPlainText()
        positions = [p.strip() for p in raw.split("\n") if p.strip()]
        return {
            **self.step,
            "scan_positions": positions,
            "scans_per_position": self.spp.value(),
            "timeout": self.timeout.value(),
        }


class ToolExchangeForm(BaseTaskForm):
    TITLE = "Tool Exchange Configuration"

    def build_form(self):
        self.operation = self.add_combo("Operation:", "operation", ["load", "dock"])
        self.gripper = self.add_combo("Gripper:", "gripper", _configured_grippers())
        self.dock_num = self.add_int("Dock Number:", "dock_number", 3, 1, 10)
        self.approach = self.add_text(
            "Approach Pose:", "approach_pose", "load_approach"
        )
        self.add_hint("Auto convention: load -> load_approach, dock -> dock_approach")

        def update_approach(text):
            if text == "load":
                self.approach.setText("load_approach")
            elif text == "dock":
                self.approach.setText("dock_approach")

        self.operation.currentTextChanged.connect(update_approach)

    def collect_values(self):
        return {
            **self.step,
            "operation": self.operation.currentText(),
            "gripper": self.gripper.currentText(),
            "dock_number": self.dock_num.value(),
            "approach_pose": self.approach.text(),
        }


class EndEffectorForm(BaseTaskForm):
    TITLE = "End Effector Configuration"

    def build_form(self):
        self.type_combo = self.add_combo(
            "Type:", "end_effector_type", _grippers_with_states()
        )
        self.action_combo = self.add_combo(
            "Action:", "end_effector_action", _end_effector_actions()
        )

    def collect_values(self):
        return {
            **self.step,
            "end_effector_type": self.type_combo.currentText(),
            "end_effector_action": self.action_combo.currentText(),
        }


class VisionMoveToForm(BaseTaskForm):
    TITLE = "Vision MoveTo Configuration"

    def build_form(self):
        self.add_hint(
            "Detect object using Zivid camera and move gripper to detected location."
        )
        self.vision_method_combo = self.add_combo(
            "Vision Method:", "vision_method", ["marker", "sample_roi"]
        )

        # Tag options (the tag anchors both vision methods)
        marker_sec = self.add_section("ArUco Tag Options")
        self.tag_id = QSpinBox()
        self.tag_id.setRange(0, 999)
        self.tag_id.setValue(int(self.step.get("tag_id", 0)))
        marker_sec.addRow("Marker ID:", self.tag_id)
        self.marker_dict = QComboBox()
        self.marker_dict.addItems(
            [
                "aruco4x4_50",
                "aruco4x4_100",
                "aruco4x4_250",
                "aruco5x5_50",
                "aruco5x5_100",
                "aruco5x5_250",
                "aruco6x6_50",
                "aruco6x6_100",
                "aruco6x6_250",
            ]
        )
        self.marker_dict.setCurrentText(
            self.step.get("marker_dictionary", "aruco4x4_50")
        )
        self.marker_dict_label = QLabel("Dictionary:")
        marker_sec.addRow(self.marker_dict_label, self.marker_dict)

        # Sample ROI options
        self.sample_roi_group = QGroupBox("Sample ROI Options")
        sample_roi = QFormLayout(self.sample_roi_group)
        self.form.addRow(self.sample_roi_group)
        self.strategy = QComboBox()
        self.strategy.addItems(
            [
                "center",
                "farthest_edge",
                "nearest_edge",
                "farthest_corner",
                "nearest_corner",
            ]
        )
        self.strategy.setCurrentText(self.step.get("strategy", "farthest_edge"))
        sample_roi.addRow("Strategy:", self.strategy)
        self.edge_inset = QDoubleSpinBox()
        self.edge_inset.setRange(0.0, 999.0)
        self.edge_inset.setDecimals(2)
        self.edge_inset.setSuffix(" mm")
        self.edge_inset.setValue(float(self.step.get("edge_inset_mm", 6.5)))
        sample_roi.addRow("Edge Inset:", self.edge_inset)

        # Common options
        common = self.add_section("Common Options")
        self.z_offset = QDoubleSpinBox()
        self.z_offset.setRange(-0.1, 0.1)
        self.z_offset.setDecimals(4)
        self.z_offset.setValue(float(self.step.get("z_offset", 0.0)))
        common.addRow("Z Offset (m):", self.z_offset)
        self.timeout = QDoubleSpinBox()
        self.timeout.setRange(1.0, 120.0)
        self.timeout.setDecimals(1)
        self.timeout.setValue(float(self.step.get("timeout", 10.0)))
        common.addRow("Timeout (s):", self.timeout)
        self.detect_only = QCheckBox()
        self.detect_only.setChecked(bool(self.step.get("detect_only", False)))
        common.addRow("Detect Only:", self.detect_only)

        # Offsets
        self.marker_offsets = self.add_offset_row(
            common,
            "Marker Offsets (m):",
            ["marker_offset_x", "marker_offset_y", "marker_offset_z"],
        )
        self.offset_dir = QComboBox()
        self.offset_dir.addItems(
            ["", "forward", "backward", "left", "right", "up", "down"]
        )
        self.offset_dir.setCurrentText(self.step.get("offset_direction", ""))
        common.addRow("Flange Direction:", self.offset_dir)
        self.offset_dist = QDoubleSpinBox()
        self.offset_dist.setRange(0, 1.0)
        self.offset_dist.setDecimals(4)
        self.offset_dist.setValue(float(self.step.get("offset_distance", 0.0)))
        common.addRow("Flange Distance (m):", self.offset_dist)

        self.vision_method_combo.currentTextChanged.connect(self._apply_vision_method)
        self._apply_vision_method(self.vision_method_combo.currentText())

    def _apply_vision_method(self, vision_method):
        sample_roi = vision_method == "sample_roi"
        self.marker_dict_label.setVisible(not sample_roi)
        self.marker_dict.setVisible(not sample_roi)
        self.sample_roi_group.setVisible(sample_roi)

    def collect_values(self):
        s = {**self.step}
        s["vision_method"] = self.vision_method_combo.currentText()
        s["tag_id"] = self.tag_id.value()
        if s["vision_method"] == "sample_roi":
            s["strategy"] = self.strategy.currentText()
            s["edge_inset_mm"] = self.edge_inset.value()
            s.pop("marker_dictionary", None)
        else:
            s["marker_dictionary"] = self.marker_dict.currentText()
            s.pop("strategy", None)
            s.pop("edge_inset_mm", None)
        s["z_offset"] = self.z_offset.value()
        s["timeout"] = self.timeout.value()
        s["detect_only"] = self.detect_only.isChecked()
        for key, spin in self.marker_offsets.items():
            v = spin.value()
            if v != 0.0:
                s[key] = v
            else:
                s.pop(key, None)
        od = self.offset_dir.currentText()
        if od:
            s["offset_direction"] = od
            s["offset_distance"] = self.offset_dist.value()
        else:
            s.pop("offset_direction", None)
            s.pop("offset_distance", None)
        return s


class PipettorForm(BaseTaskForm):
    TITLE = "Pipettor Configuration"

    def build_form(self):
        self.add_hint("SUCK=aspirate, EXPEL=dispense, EJECT_TIP, SET_LED")
        self.operation = self.add_combo(
            "Operation:", "operation", ["SUCK", "EXPEL", "EJECT_TIP", "SET_LED"]
        )
        self.volume = self.add_float("Volume %:", "volume_pct", 0.5, 0.0, 1.0, 2)

        # LED color
        led = self.add_section("LED Color (for SET_LED)")
        led_color = self.step.get("led_color", {"r": 0.0, "g": 1.0, "b": 0.0})
        self.led_r = QDoubleSpinBox()
        self.led_r.setRange(0, 1)
        self.led_r.setDecimals(2)
        self.led_r.setValue(led_color.get("r", 0.0))
        led.addRow("Red:", self.led_r)
        self.led_g = QDoubleSpinBox()
        self.led_g.setRange(0, 1)
        self.led_g.setDecimals(2)
        self.led_g.setValue(led_color.get("g", 1.0))
        led.addRow("Green:", self.led_g)
        self.led_b = QDoubleSpinBox()
        self.led_b.setRange(0, 1)
        self.led_b.setDecimals(2)
        self.led_b.setValue(led_color.get("b", 0.0))
        led.addRow("Blue:", self.led_b)

        # Color preview
        self.color_preview = QLabel()
        self.color_preview.setFixedHeight(25)
        self._update_preview()
        led.addRow("Preview:", self.color_preview)

        self.led_r.valueChanged.connect(self._update_preview)
        self.led_g.valueChanged.connect(self._update_preview)
        self.led_b.valueChanged.connect(self._update_preview)

    def _update_preview(self):
        r = int(self.led_r.value() * 255)
        g = int(self.led_g.value() * 255)
        b = int(self.led_b.value() * 255)
        self.color_preview.setStyleSheet(
            f"background-color: rgb({r},{g},{b}); border: 1px solid gray;"
        )

    def collect_values(self):
        return {
            **self.step,
            "operation": self.operation.currentText(),
            "volume_pct": self.volume.value(),
            "led_color": {
                "r": self.led_r.value(),
                "g": self.led_g.value(),
                "b": self.led_b.value(),
            },
        }


class _GridSelectorForm(BaseTaskForm):
    """Base form with a clickable grid for selecting A1, A2, B1, etc."""

    TARGET_NAME = ""  # Override in subclasses
    _DIRECTIONS = ["forward", "backward", "left", "right", "up", "down"]
    _OPPOSITE = {
        "forward": "backward",
        "backward": "forward",
        "left": "right",
        "right": "left",
        "up": "down",
        "down": "up",
    }

    def build_form(self):
        loaded = _vision_target_config(self.TARGET_NAME)
        target_cfg = copy.deepcopy(loaded) if isinstance(loaded, dict) else {}
        override = self.step.get("config", {})
        if isinstance(override, dict):
            base_grid = target_cfg.get("grid", {})
            override_grid = override.get("grid", {})
            target_cfg.update({k: v for k, v in override.items() if k != "grid"})
            target_cfg["grid"] = {
                **(base_grid if isinstance(base_grid, dict) else {}),
                **(override_grid if isinstance(override_grid, dict) else {}),
            }
        grid_cfg = target_cfg.get("grid", {})
        self._rows = int(grid_cfg.get("rows", 1))
        self._cols = int(grid_cfg.get("cols", 1))

        self._tabs = QTabWidget()
        self._tabs.setMinimumHeight(470)
        self.form.addRow(self._tabs)

        position_tab = QWidget()
        self._position_form = QFormLayout(position_tab)
        self._tabs.addTab(position_tab, "Position")
        self._add_layout_hint(
            self._position_form,
            f"Select a position on the {self._rows}×{self._cols} grid.",
        )

        grid_group = QGroupBox("Position")
        self._grid_layout = QGridLayout(grid_group)
        self._grid_layout.setSpacing(3)
        self._position_form.addRow(grid_group)

        self._grid_buttons: dict[tuple[int, int], QPushButton] = {}
        self._selected_row = int(self.step.get("row", 0))
        self._selected_col = int(self.step.get("col", 0))
        self._build_grid()

        self._sel_label = QLabel(
            self._cell_name(self._selected_row, self._selected_col)
        )
        self._sel_label.setStyleSheet(
            "font-size: 13px; font-weight: bold; color: #5ce68a;"
        )
        self._position_form.addRow("Selected:", self._sel_label)

        config_tab = QScrollArea()
        config_tab.setWidgetResizable(True)
        config_widget = QWidget()
        self._config_form = QFormLayout(config_widget)
        self._config_form.setSizeConstraint(QLayout.SizeConstraint.SetMinAndMaxSize)
        config_tab.setWidget(config_widget)
        self._tabs.addTab(config_tab, "Config")
        self._build_config(target_cfg, grid_cfg)

        self._row_count.valueChanged.connect(self._resize_grid)
        self._col_count.valueChanged.connect(self._resize_grid)
        self.setMinimumWidth(max(800, self._cols * 55 + 140))
        self.setMinimumHeight(600)

    @staticmethod
    def _add_layout_hint(layout, text):
        label = QLabel(text)
        label.setStyleSheet("color: gray; font-size: 10px;")
        label.setWordWrap(True)
        layout.addRow(label)

    def _build_grid(self):
        while self._grid_layout.count():
            item = self._grid_layout.takeAt(0)
            if item.widget():
                item.widget().deleteLater()
        self._grid_buttons.clear()
        self._selected_row = min(max(self._selected_row, 0), self._rows - 1)
        self._selected_col = min(max(self._selected_col, 0), self._cols - 1)

        for c in range(self._cols):
            label = QLabel(str(c + 1))
            label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            label.setFont(QFont("Monospace", 10, QFont.Weight.Bold))
            label.setStyleSheet("color: #aab2c0;")
            self._grid_layout.addWidget(label, 0, c + 1)

        for r in range(self._rows):
            row_label = QLabel(chr(ord("A") + r))
            row_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            row_label.setFont(QFont("Monospace", 10, QFont.Weight.Bold))
            row_label.setStyleSheet("color: #c3cad5;")
            self._grid_layout.addWidget(row_label, r + 1, 0)
            for c in range(self._cols):
                btn = QPushButton(self._cell_name(r, c))
                btn.setFixedSize(52, 36)
                btn.setFont(QFont("Monospace", 10, QFont.Weight.Bold))
                btn.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed)
                btn.setCheckable(True)
                btn.clicked.connect(
                    lambda checked, row=r, col=c: self._select_cell(row, col)
                )
                self._grid_layout.addWidget(btn, r + 1, c + 1)
                self._grid_buttons[(r, c)] = btn
        self._update_selection()

    def _resize_grid(self):
        self._rows = self._row_count.value()
        self._cols = self._col_count.value()
        self._build_grid()
        self._sel_label.setText(self._cell_name(self._selected_row, self._selected_col))

    @staticmethod
    def _distance_spin(value, *, positive=False):
        spin = QDoubleSpinBox()
        spin.setRange(0.000001 if positive else -2.0, 2.0)
        spin.setDecimals(6)
        spin.setSingleStep(0.001)
        spin.setValue(float(value))
        return spin

    def _direction_combo(self, value):
        combo = QComboBox()
        combo.addItems(self._DIRECTIONS)
        combo.setCurrentText(value)
        return combo

    def _build_config(self, target_cfg, grid_cfg):
        detection = QGroupBox("Vision")
        detection_form = QFormLayout(detection)
        self._marker_id = QSpinBox()
        self._marker_id.setRange(0, 999)
        self._marker_id.setValue(int(target_cfg.get("marker_id", 0)))
        detection_form.addRow("Marker ID:", self._marker_id)
        self._scan_pose = QLineEdit(str(target_cfg.get("scan_pose", "")))
        detection_form.addRow("Scan pose:", self._scan_pose)
        self._config_form.addRow(detection)

        grid_group = QGroupBox("Grid")
        grid_form = QFormLayout(grid_group)
        dimensions = QHBoxLayout()
        self._row_count = QSpinBox()
        self._row_count.setRange(1, 26)
        self._row_count.setValue(self._rows)
        self._col_count = QSpinBox()
        self._col_count.setRange(1, 99)
        self._col_count.setValue(self._cols)
        dimensions.addWidget(QLabel("Rows:"))
        dimensions.addWidget(self._row_count)
        dimensions.addWidget(QLabel("Columns:"))
        dimensions.addWidget(self._col_count)
        grid_form.addRow("Dimensions:", dimensions)

        default_pitch = float(grid_cfg.get("pitch", 0.009))
        self._row_pitch = self._distance_spin(
            grid_cfg.get("row_pitch", default_pitch), positive=True
        )
        self._col_pitch = self._distance_spin(
            grid_cfg.get("col_pitch", default_pitch), positive=True
        )
        grid_form.addRow("Row pitch (m):", self._row_pitch)
        grid_form.addRow("Column pitch (m):", self._col_pitch)

        marker_offset = target_cfg.get("marker_offset") or {}
        self._row_offset = self._distance_spin(
            grid_cfg.get("row_offset", abs(float(marker_offset.get("y", 0.0))))
        )
        self._col_offset = self._distance_spin(
            grid_cfg.get("col_offset", abs(float(marker_offset.get("x", 0.0))))
        )
        grid_form.addRow("A1 row offset (m):", self._row_offset)
        grid_form.addRow("A1 column offset (m):", self._col_offset)

        row_direction = grid_cfg.get("row_direction", "up")
        col_direction = grid_cfg.get("col_direction", "left")
        self._row_direction = self._direction_combo(row_direction)
        self._col_direction = self._direction_combo(col_direction)
        self._row_increasing = self._direction_combo(
            grid_cfg.get("row_increasing", self._OPPOSITE[row_direction])
        )
        self._col_increasing = self._direction_combo(
            grid_cfg.get("col_increasing", self._OPPOSITE[col_direction])
        )
        grid_form.addRow("A1 row direction:", self._row_direction)
        grid_form.addRow("A1 column direction:", self._col_direction)
        grid_form.addRow("Rows increase:", self._row_increasing)
        grid_form.addRow("Columns increase:", self._col_increasing)
        self._config_form.addRow(grid_group)

        moves_group = QGroupBox("Movement Steps")
        moves_layout = QVBoxLayout(moves_group)
        self._move_table = QTableWidget(0, 3)
        self._move_table.setHorizontalHeaderLabels(
            ["Step", "Direction", "Distance (m)"]
        )
        self._move_table.setSelectionBehavior(
            QAbstractItemView.SelectionBehavior.SelectRows
        )
        self._move_table.setSelectionMode(
            QAbstractItemView.SelectionMode.SingleSelection
        )
        header = self._move_table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.ResizeMode.Stretch)
        header.setSectionResizeMode(1, QHeaderView.ResizeMode.Stretch)
        header.setSectionResizeMode(2, QHeaderView.ResizeMode.ResizeToContents)
        self._move_table.verticalHeader().setDefaultSectionSize(38)
        self._move_table.setMinimumHeight(180)
        moves_layout.addWidget(self._move_table)

        controls = QHBoxLayout()
        for label, callback in (
            ("Add", self._add_move),
            ("Remove", self._remove_move),
            ("Up", lambda: self._move_selected(-1)),
            ("Down", lambda: self._move_selected(1)),
        ):
            button = QPushButton(label)
            button.clicked.connect(callback)
            controls.addWidget(button)
        controls.addStretch()
        moves_layout.addLayout(controls)
        self._config_form.addRow(moves_group)

        for move in target_cfg.get("moves", []):
            self._add_move(move)
        if not self._move_table.rowCount():
            self._add_move()
        self._move_table.setCurrentCell(0, 0)

    def _add_move(self, move=None):
        row = self._move_table.rowCount()
        self._move_table.insertRow(row)
        kind = QComboBox()
        kind.addItems(["Fixed move", "Column offset", "Row offset"])
        direction = self._direction_combo("forward")
        distance = self._distance_spin(0.01, positive=True)
        if move == "column_offset":
            kind.setCurrentText("Column offset")
        elif move == "row_offset":
            kind.setCurrentText("Row offset")
        elif isinstance(move, dict):
            direction.setCurrentText(move.get("direction", "forward"))
            distance.setValue(float(move.get("distance", 0.01)))
        self._move_table.setCellWidget(row, 0, kind)
        self._move_table.setCellWidget(row, 1, direction)
        self._move_table.setCellWidget(row, 2, distance)

        def toggle_fixed(text):
            fixed = text == "Fixed move"
            direction.setEnabled(fixed)
            distance.setEnabled(fixed)

        kind.currentTextChanged.connect(toggle_fixed)
        toggle_fixed(kind.currentText())
        self._move_table.setCurrentCell(row, 0)

    def _move_value(self, row):
        kind = self._move_table.cellWidget(row, 0).currentText()
        if kind == "Column offset":
            return "column_offset"
        if kind == "Row offset":
            return "row_offset"
        return {
            "direction": self._move_table.cellWidget(row, 1).currentText(),
            "distance": self._move_table.cellWidget(row, 2).value(),
        }

    def _set_moves(self, moves):
        self._move_table.setRowCount(0)
        for move in moves:
            self._add_move(move)

    def _remove_move(self):
        if self._move_table.rowCount() <= 1:
            return
        row = max(self._move_table.currentRow(), 0)
        self._move_table.removeRow(row)
        self._move_table.setCurrentCell(min(row, self._move_table.rowCount() - 1), 0)

    def _move_selected(self, delta):
        row = self._move_table.currentRow()
        destination = row + delta
        if row < 0 or not 0 <= destination < self._move_table.rowCount():
            return
        moves = [
            self._move_value(index) for index in range(self._move_table.rowCount())
        ]
        moves[row], moves[destination] = moves[destination], moves[row]
        self._set_moves(moves)
        self._move_table.setCurrentCell(destination, 0)

    def _cell_name(self, row, col):
        return f"{chr(ord('A') + row)}{col + 1}"

    def _select_cell(self, row, col):
        self._selected_row = row
        self._selected_col = col
        self._update_selection()
        self._sel_label.setText(self._cell_name(row, col))

    def _update_selection(self):
        for (r, c), btn in self._grid_buttons.items():
            if r == self._selected_row and c == self._selected_col:
                btn.setChecked(True)
                btn.setStyleSheet(
                    "background-color: #2a6b3a; color: white;"
                    "border: 1px solid #5ce68a; padding: 0;"
                )
            else:
                btn.setChecked(False)
                btn.setStyleSheet("color: #e6eaf2; padding: 0;")

    def collect_values(self):
        row_direction = self._row_direction.currentText()
        col_direction = self._col_direction.currentText()
        row_increasing = self._row_increasing.currentText()
        col_increasing = self._col_increasing.currentText()
        if row_increasing not in (row_direction, self._OPPOSITE[row_direction]):
            raise ValueError("Rows must increase along the selected row axis")
        if col_increasing not in (col_direction, self._OPPOSITE[col_direction]):
            raise ValueError("Columns must increase along the selected column axis")
        return {
            **self.step,
            "row": self._selected_row,
            "col": self._selected_col,
            "position": self._cell_name(self._selected_row, self._selected_col),
            "config": {
                "mode": "grid",
                "marker_id": self._marker_id.value(),
                "scan_pose": self._scan_pose.text(),
                "grid": {
                    "rows": self._row_count.value(),
                    "cols": self._col_count.value(),
                    "row_pitch": self._row_pitch.value(),
                    "col_pitch": self._col_pitch.value(),
                    "row_offset": self._row_offset.value(),
                    "col_offset": self._col_offset.value(),
                    "row_direction": row_direction,
                    "col_direction": col_direction,
                    "row_increasing": row_increasing,
                    "col_increasing": col_increasing,
                },
                "moves": [
                    self._move_value(row) for row in range(self._move_table.rowCount())
                ],
            },
        }


class PickupTipForm(_GridSelectorForm):
    TITLE = "Pickup Tip"
    TARGET_NAME = "tip_rack"

    def build_form(self):
        super().build_form()
        self._add_layout_hint(
            self._position_form,
            "Uses vision to align with marker, then moves to selected position.",
        )


class PickupVialForm(_GridSelectorForm):
    TITLE = "Vial Rack"
    TARGET_NAME = "vial_rack"

    def build_form(self):
        super().build_form()

        # Pipettor operation after reaching vial
        op_group = QGroupBox("Pipettor Operation (optional)")
        op_layout = QFormLayout(op_group)

        self._vial_operation = QComboBox()
        self._vial_operation.addItems(["None", "SUCK", "EXPEL", "RINSE"])
        self._vial_operation.setCurrentText(self.step.get("pipettor_operation", "None"))
        op_layout.addRow("Operation:", self._vial_operation)

        self._vial_volume = QDoubleSpinBox()
        self._vial_volume.setRange(0.0, 1.0)
        self._vial_volume.setDecimals(2)
        self._vial_volume.setSingleStep(0.05)
        self._vial_volume.setValue(float(self.step.get("volume_pct", 0.5)))
        op_layout.addRow("Volume %:", self._vial_volume)

        self._rinse_count = QSpinBox()
        self._rinse_count.setRange(1, 100)
        self._rinse_count.setValue(int(self.step.get("rinse_count", 2)))
        self._rinse_count.setToolTip(
            "Each rinse sucks and expels once; the final suck keeps the requested volume."
        )
        op_layout.addRow("Rinse cycles:", self._rinse_count)
        self._add_layout_hint(
            op_layout,
            "RINSE repeats SUCK/EXPEL for each cycle, then SUCKs once more "
            "before leaving the vial.",
        )

        def _toggle_volume(text):
            self._vial_volume.setEnabled(text != "None")
            self._rinse_count.setEnabled(text == "RINSE")

        self._vial_operation.currentTextChanged.connect(_toggle_volume)
        _toggle_volume(self._vial_operation.currentText())

        self._config_form.addRow(op_group)
        self._add_layout_hint(
            self._position_form,
            "Configure the pipettor action and movement in the Config tab.",
        )

    def collect_values(self):
        s = super().collect_values()
        op = self._vial_operation.currentText()
        if op != "None":
            s["pipettor_operation"] = op
            s["volume_pct"] = self._vial_volume.value()
            if op == "RINSE":
                s["rinse_count"] = self._rinse_count.value()
            else:
                s.pop("rinse_count", None)
        else:
            s.pop("pipettor_operation", None)
            s.pop("volume_pct", None)
            s.pop("rinse_count", None)
        return s


class SpincoaterForm(BaseTaskForm):
    """Form for place_spincoater and pick_spincoater tasks."""

    @property
    def _is_pick(self):
        return self.step.get("task_type") == "pick_spincoater"

    @property
    def TITLE(self):
        return "Pick from Spincoater" if self._is_pick else "Place on Spincoater"

    def build_form(self):
        mode = "pickup" if self._is_pick else "placement"
        self.add_hint(
            f"Vision-guided {mode}: detects orientation via 2D capture, "
            f"corrects wrist angle, then {'picks' if self._is_pick else 'places'}."
        )
        self.scan_pose = self.add_text("Scan pose:", "scan_pose", "spincoater_scan")
        pose_key = "pickup_pose" if self._is_pick else "place_pose"
        self.target_pose = self.add_text(
            f"{'Pickup' if self._is_pick else 'Place'} pose:",
            pose_key,
            "spincoater_place",
        )
        self.forward_dist = self.add_float(
            "Forward distance (m):", "forward_distance", 0.003, 0.0, 0.05, 4
        )
        self.k_offset = self.add_float(
            "K offset (deg):", "k_offset", 0.0, -90.0, 90.0, 1
        )
        if not self._is_pick:
            self.release = self.add_check("Release vacuum:", "release", True)

    def collect_values(self):
        pose_key = "pickup_pose" if self._is_pick else "place_pose"
        result = {
            **self.step,
            "scan_pose": self.scan_pose.text(),
            pose_key: self.target_pose.text(),
            "forward_distance": self.forward_dist.value(),
            "k_offset": self.k_offset.value(),
        }
        if not self._is_pick:
            result["release"] = self.release.isChecked()
        return result


# --- Form dispatch map ---

_FORMS = {
    "moveto": MoveToForm,
    "pick_sample": SampleForm,
    "place_sample": SampleForm,
    "vision_scan": VisionScanForm,
    "tool_exchange": ToolExchangeForm,
    "end_effector": EndEffectorForm,
    "vision_moveto": VisionMoveToForm,
    "pipettor": PipettorForm,
    "pickup_tip": PickupTipForm,
    "pickup_vial": PickupVialForm,
    "place_spincoater": SpincoaterForm,
    "pick_spincoater": SpincoaterForm,
}
