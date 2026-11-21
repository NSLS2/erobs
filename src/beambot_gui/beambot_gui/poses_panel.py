"""Poses panel: filterable list of named robot poses loaded from the beamline YAML."""

import yaml
from pathlib import Path

from .pose_io import read_poses, write_poses

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QLineEdit, QListWidget, QListWidgetItem,
    QLabel, QHBoxLayout, QPushButton, QGroupBox, QFormLayout,
    QAbstractItemView,
)
from PyQt6.QtCore import Qt, pyqtSignal, QMimeData
from PyQt6.QtGui import QFont

POSE_MIME_TYPE = "application/x-mtc-pose-name"


class PoseListItem(QListWidgetItem):
    """Custom list item showing pose name and description."""

    def __init__(self, name: str, values: list, group: str = ""):
        super().__init__()
        self.pose_name = name
        self.pose_values = values
        self.group = group
        self.setText(name)
        self.setData(Qt.ItemDataRole.UserRole, {"name": name, "values": values, "group": group})
        self.setToolTip(f"[{', '.join(f'{v:.2f}' for v in values)}]")


class _PoseDragListWidget(QListWidget):
    """QListWidget that exports the dragged pose's name as a custom MIME type."""

    def mimeData(self, items):
        md = QMimeData()
        # We only ever drag a single pose at a time; take the first selected item.
        if items:
            it = items[0]
            if isinstance(it, PoseListItem):
                md.setData(POSE_MIME_TYPE, it.pose_name.encode("utf-8"))
                md.setText(it.pose_name)  # plain-text fallback for other drop targets
        return md


class PosesPanel(QWidget):
    """Panel displaying poses loaded from the beamline poses YAML file."""

    pose_clicked = pyqtSignal(str, list)  # name, joint values — single-click, info only
    pose_activated = pyqtSignal(str, list)  # double-click — append as Move To
    poses_loaded = pyqtSignal(dict)  # full poses dict

    def __init__(self, parent=None):
        super().__init__(parent)
        self._poses: dict[str, list] = {}
        self._groups: dict[str, str] = {}  # pose_name -> group label
        self._poses_file: Path | None = None
        self._build_ui()

    def _build_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(4)

        # Filter bar
        filter_row = QHBoxLayout()
        self.filter_edit = QLineEdit()
        self.filter_edit.setPlaceholderText("Filter poses")
        self.filter_edit.setClearButtonEnabled(True)
        self.filter_edit.textChanged.connect(self._apply_filter)
        filter_row.addWidget(self.filter_edit)

        self.reload_btn = QPushButton("↻")
        self.reload_btn.setFixedWidth(30)
        self.reload_btn.setToolTip("Reload poses from file")
        self.reload_btn.clicked.connect(self._reload)
        filter_row.addWidget(self.reload_btn)
        layout.addLayout(filter_row)

        # Pose count label
        self.count_label = QLabel("No poses loaded")
        self.count_label.setStyleSheet("color: gray; font-size: 10px;")
        layout.addWidget(self.count_label)

        # Pose list (drag-enabled so users can drop a pose onto the step list)
        self.pose_list = _PoseDragListWidget()
        self.pose_list.setAlternatingRowColors(True)
        self.pose_list.setDragEnabled(True)
        self.pose_list.setDragDropMode(QAbstractItemView.DragDropMode.DragOnly)
        self.pose_list.itemDoubleClicked.connect(self._on_double_click)
        self.pose_list.itemClicked.connect(self._on_click)
        layout.addWidget(self.pose_list, stretch=1)

        # Detail panel — shows joint values on selection
        detail_box = QGroupBox("Joint Values")
        detail_box.setMaximumHeight(160)
        detail_layout = QFormLayout(detail_box)
        detail_layout.setContentsMargins(6, 6, 6, 6)
        detail_layout.setSpacing(2)
        self.joint_labels = []
        joint_names = ["Shoulder Pan", "Shoulder Lift", "Elbow",
                       "Wrist 1", "Wrist 2", "Wrist 3"]
        for jname in joint_names:
            lbl = QLabel("—")
            lbl.setFont(QFont("Monospace", 9))
            detail_layout.addRow(f"{jname}:", lbl)
            self.joint_labels.append(lbl)
        layout.addWidget(detail_box)

        # Source file label
        self.source_label = QLabel("")
        self.source_label.setStyleSheet("color: gray; font-size: 9px;")
        self.source_label.setWordWrap(True)
        layout.addWidget(self.source_label)

    def load_from_beamline_config(self, config_path: str | Path):
        """Load poses by reading poses_file from the beamline config YAML."""
        config_path = Path(config_path)
        if not config_path.exists():
            return

        with open(config_path) as f:
            config = yaml.safe_load(f)

        poses_file_rel = config.get("poses_file", "")
        if not poses_file_rel:
            return

        # poses_file is relative to workspace root — walk up from config
        # to find it (the active beamline YAML is named via $BEAMBOT_BEAMLINE_CONFIG)
        workspace_root = config_path.parent
        while workspace_root != workspace_root.parent:
            if (workspace_root / poses_file_rel).exists():
                break
            workspace_root = workspace_root.parent
        else:
            return

        poses_path = workspace_root / poses_file_rel
        self.load_from_file(poses_path)

    def load_from_file(self, path: str | Path):
        """Load poses directly from a YAML file, parsing group comments."""
        path = Path(path)
        if not path.exists():
            self.count_label.setText(f"File not found: {path.name}")
            return

        self._poses_file = path
        self._groups = {}
        self._poses = read_poses(path)

        # Parse comments to extract group labels (e.g. "# --- Pipettor ---").
        # Read-only display pass; pose values themselves come from read_poses.
        current_group = ""
        with open(path) as f:
            for line in f:
                stripped = line.strip()
                if stripped.startswith("# ---") and stripped.endswith("---"):
                    current_group = stripped.strip("# -").strip()
                elif ":" in stripped and not stripped.startswith("#"):
                    pose_name = stripped.split(":")[0].strip()
                    if pose_name in self._poses:
                        self._groups[pose_name] = current_group
        self._refresh_list()
        self.source_label.setText(f"Source: {path.name}")
        self.poses_loaded.emit(dict(self._poses))

    def _refresh_list(self):
        """Rebuild the list widget from loaded poses."""
        self.pose_list.clear()
        filter_text = self.filter_edit.text().lower()

        for name, values in sorted(self._poses.items()):
            group = self._groups.get(name, "")
            if filter_text and filter_text not in name.lower() and filter_text not in group.lower():
                continue
            item = PoseListItem(name, values, group)
            self.pose_list.addItem(item)

        visible = self.pose_list.count()
        total = len(self._poses)
        if filter_text:
            self.count_label.setText(f"{visible}/{total} poses")
        else:
            self.count_label.setText(f"{total} poses")

    def _apply_filter(self):
        self._refresh_list()

    def _on_click(self, item: PoseListItem):
        for lbl, val in zip(self.joint_labels, item.pose_values):
            lbl.setText(f"{val:.2f}°")
        self.pose_clicked.emit(item.pose_name, item.pose_values)

    def _on_double_click(self, item: PoseListItem):
        self.pose_activated.emit(item.pose_name, item.pose_values)

    def _reload(self):
        if self._poses_file:
            self.load_from_file(self._poses_file)

    def get_poses(self) -> dict[str, list]:
        """Return all loaded poses as a dict."""
        return dict(self._poses)

    def get_pose(self, name: str) -> list | None:
        """Get joint values for a specific pose name."""
        return self._poses.get(name)

    def get_poses_file(self) -> Path | None:
        """Return the registry file path, or None if no file is loaded."""
        return self._poses_file

    def save_pose(self, name: str, values: list) -> bool:
        """Persist a registry pose, then reload to refresh + re-emit.

        Reads the current file (cross-process truth), merges the entry, and
        writes it back atomically. Returns False if no registry file is set.
        """
        if self._poses_file is None:
            return False
        poses = read_poses(self._poses_file)
        poses[name] = values
        write_poses(self._poses_file, poses)
        self.load_from_file(self._poses_file)
        return True

    def delete_pose(self, name: str) -> bool:
        """Remove a registry pose, then reload. Returns False if no file set."""
        if self._poses_file is None:
            return False
        poses = read_poses(self._poses_file)
        poses.pop(name, None)
        write_poses(self._poses_file, poses)
        self.load_from_file(self._poses_file)
        return True
