"""Main window: layout, menus, step list, execution controls, status log."""

import json
import time

from pathlib import Path

from PyQt6.QtWidgets import (
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QSplitter,
    QLabel,
    QCheckBox,
    QComboBox,
    QLineEdit,
    QTextEdit,
    QPushButton,
    QProgressBar,
    QFileDialog,
    QMessageBox,
    QDialog,
    QTabWidget,
    QListWidget,
    QListWidgetItem,
    QFrame,
    QSizePolicy,
)
from PyQt6.QtCore import Qt, QSize, QMimeData
from PyQt6.QtGui import QFont, QKeySequence
from action_msgs.msg import GoalStatus

from .ros2_bridge import ROS2Bridge, ROS2_AVAILABLE
from .camera_panel import CameraPanel
from .pose_dialogs import PosesManagerDialog, SavePoseDialog
from .poses_panel import PosesPanel
from .chat_panel import ChatPanel
from .agent_bridge import AgentBridge
from .step_list_panel import StepListPanel, TASK_TYPE_CONFIG
from . import theme

STEP_CLIPBOARD_MIME = "application/x-erobs-task-steps+json"
STEP_CLIPBOARD_MAX_BYTES = 1_000_000
STEP_CLIPBOARD_MAX_STEPS = 1_000

try:
    from .visualization_panel import VisualizationPanel, WEBENGINE_AVAILABLE
except ImportError:
    WEBENGINE_AVAILABLE = False

# Use the active beamline's tools and states for new task defaults.
def _build_task_defaults(beamline_config: dict) -> dict:
    grippers = beamline_config.get("grippers", {}) if beamline_config else {}
    swappable = [g for g in grippers if g != "none"] or list(grippers.keys())
    actuated = [g for g, c in grippers.items() if c.get("states")]

    default_tool_gripper = swappable[0] if swappable else ""
    default_dock = beamline_config.get("tool_exchange", {}).get("reference_dock", 1)
    default_ee_gripper = actuated[0] if actuated else ""
    default_ee_action = (
        grippers.get(default_ee_gripper, {}).get("states", {}).get("grasp", "")
        if default_ee_gripper else ""
    )
    default_marker_dict = (
        beamline_config.get("camera", {}).get("marker_dictionary", "aruco4x4_50")
        if beamline_config else "aruco4x4_50"
    )

    return {
        "moveto": {
            "task_type": "moveto",
            "target": "moveit_home",
            "planning_type": "joint",
        },
        "pick_sample": {
            "task_type": "pick_sample",
            "use_vision": True,
            "vision_method": "marker",
            "tag_id": 0,
            "scan_pose": "",
            "z_offset": 0.0,
        },
        "place_sample": {
            "task_type": "place_sample",
            "use_vision": True,
            "vision_method": "marker",
            "tag_id": 0,
            "scan_pose": "",
            "z_offset": 0.0,
        },
        "vision_scan": {
            "task_type": "vision_scan",
            "scan_positions": [],
            "scans_per_position": 3,
            "timeout": 10.0,
        },
        "tool_exchange": {
            "task_type": "tool_exchange",
            "operation": "load",
            "gripper": default_tool_gripper,
            "dock_number": default_dock,
            "approach_pose": "load_approach",
        },
        "end_effector": {
            "task_type": "end_effector",
            "end_effector_type": default_ee_gripper,
            "end_effector_action": default_ee_action,
        },
        "vision_moveto": {
            "task_type": "vision_moveto",
            "vision_method": "marker",
            "tag_id": 0,
            "timeout": 10.0,
            "z_offset": 0.0,
            "marker_dictionary": default_marker_dict,
        },
        "pipettor": {"task_type": "pipettor", "operation": "SUCK", "volume_pct": 0.5},
        "pickup_tip": {"task_type": "pickup_tip", "row": 0, "col": 0},
        "pickup_vial": {"task_type": "pickup_vial", "row": 0, "col": 0},
        "place_spincoater": {
            "task_type": "place_spincoater",
            "scan_pose": "spincoater_scan",
            "place_pose": "spincoater_place",
            "forward_distance": 0.003,
            "k_offset": 0.0,
        },
        "pick_spincoater": {
            "task_type": "pick_spincoater",
            "scan_pose": "spincoater_scan",
            "pickup_pose": "spincoater_place",
            "forward_distance": 0.003,
            "k_offset": 0.0,
        },
        "pause": {"task_type": "pause"},
    }


# Fallback defaults without a beamline configuration.
TASK_DEFAULTS = _build_task_defaults({})


def task_summary(step):
    """One-line summary for the task tree."""
    t = step.get("task_type", "?")
    if t == "moveto":
        d = step.get("direction", "")
        if d:
            return f"Relative {d} {step.get('distance', 0)}m"
        cart = step.get("cartesian_target")
        if cart:
            return f"Cartesian [{', '.join(f'{v:.3f}' for v in cart)}]"
        return f"Move to {step.get('target', '?')}"
    elif t == "pick_sample":
        if step.get("use_vision", True):
            return f"Vision pick ({step.get('vision_method', 'marker')}, tag {step.get('tag_id', 0)})"
        return f"Hardcoded pick -> {step.get('target_pose', '?')}"
    elif t == "place_sample":
        if step.get("use_vision", True):
            return f"Vision place ({step.get('vision_method', 'marker')}, tag {step.get('tag_id', 0)})"
        return f"Hardcoded place -> {step.get('target_pose', '?')}"
    elif t == "vision_scan":
        n = len(step.get("scan_positions", []))
        return f"Scan {n} positions ({step.get('scans_per_position', 3)}x each)"
    elif t == "tool_exchange":
        return f"{step.get('operation', '?')} {step.get('gripper', '?')} at dock {step.get('dock_number', '?')}"
    elif t == "end_effector":
        return f"{step.get('end_effector_type', '?')} {step.get('end_effector_action', '?')}"
    elif t == "vision_moveto":
        prefix = "[detect only] " if step.get("detect_only") else ""
        s = f"{prefix}Detect ArUco {step.get('tag_id', 0)}"
        od = step.get("offset_direction", "")
        if od:
            s += f" +{od} {step.get('offset_distance', 0)}m"
        return s
    elif t == "pipettor":
        op = step.get("operation", "SUCK")
        if op in ("SUCK", "EXPEL"):
            return f"{op} {step.get('volume_pct', 0) * 100:.0f}%"
        elif op == "SET_LED":
            c = step.get("led_color", {})
            return f"LED ({c.get('r', 0):.1f},{c.get('g', 0):.1f},{c.get('b', 0):.1f})"
        return op
    elif t == "pickup_tip":
        pos = step.get("position", f"{chr(65 + step.get('row', 0))}{step.get('col', 0) + 1}")
        return f"Tip @ {pos}"
    elif t == "pickup_vial":
        pos = step.get("position", f"{chr(65 + step.get('row', 0))}{step.get('col', 0) + 1}")
        op = step.get("pipettor_operation", "")
        if op == "RINSE":
            op = f"RINSE ×{step.get('rinse_count', 2)}"
        suffix = f" → {op} {step.get('volume_pct', 0) * 100:.0f}%" if op else ""
        return f"Vial @ {pos}{suffix}"
    elif t == "pause":
        return "Wait for operator to resume"
    return t


def _execution_controls(state, goal_pending=False):
    """Return Execute, Pause, Resume, Stop, editing, and paused states."""
    if goal_pending:
        return False, False, False, True, False, False
    return {
        "IDLE": (True, False, False, False, True, False),
        "RUNNING": (False, True, False, True, False, False),
        "COMPLETING_TASK": (False, False, False, True, False, False),
        "PAUSED": (False, False, True, True, False, True),
    }.get(state, (False, False, False, False, False, False))


class BeambotMainWindow(QMainWindow):
    def __init__(self, ros2: ROS2Bridge):
        super().__init__()
        self.ros2 = ros2
        self.current_json_file = None
        self.current_robot_pose = None
        # Agent-initiated runs must report their result to the agent bridge.
        self._execution_initiator = "human"
        self._last_goal_was_dry_run = False
        self._execution_state = None
        self._goal_pending = False
        self._execution_start_index = 0

        # Load configuration before building the UI; missing YAML leaves fields empty.
        self._beamline_config, self._beamline_config_path = self._load_beamline_yaml()

        # Prefer "none", then the first configured gripper, or empty if unconfigured.
        grippers = self._beamline_config.get("grippers", {})
        if "none" in grippers:
            default_start_gripper = "none"
        elif grippers:
            default_start_gripper = next(iter(grippers))
        else:
            default_start_gripper = ""

        self.config = {
            "start_gripper": default_start_gripper,
            "poses": {
                "home": [0.0, -90.0, -90.0, -90.0, 90.0, 0.0],
            },
            "tasks": [],
        }
        self._task_defaults = _build_task_defaults(self._beamline_config)

        self.setWindowTitle("MTC GUI Client (beambot)")
        self.resize(1400, 900)

        self._build_menu()
        self._build_central()
        self._connect_signals()
        self._load_beamline_poses()

    # --- Menu ---

    def _build_menu(self):
        mb = self.menuBar()
        file_menu = mb.addMenu("File")
        load_action = file_menu.addAction("Load JSON", self._load_json)
        load_action.setShortcut("Ctrl+O")
        save_action = file_menu.addAction("Save JSON", self._save_json)
        save_action.setShortcut("Ctrl+S")
        file_menu.addSeparator()
        exit_action = file_menu.addAction("Exit", self.close)
        exit_action.setShortcut("Ctrl+Q")

        edit_menu = mb.addMenu("Edit")
        self.copy_action = edit_menu.addAction("Copy", self._copy_steps)
        self.copy_action.setShortcuts(QKeySequence.StandardKey.Copy)
        self.copy_action.setShortcutContext(
            Qt.ShortcutContext.WidgetWithChildrenShortcut
        )
        self.paste_action = edit_menu.addAction("Paste", self._paste_steps)
        self.paste_action.setShortcuts(QKeySequence.StandardKey.Paste)
        self.paste_action.setShortcutContext(
            Qt.ShortcutContext.WidgetWithChildrenShortcut
        )

        view_menu = mb.addMenu("View")
        self.dark_mode_action = view_menu.addAction("Dark Mode")
        self.dark_mode_action.setCheckable(True)
        self.dark_mode_action.setChecked(True)
        self.dark_mode_action.toggled.connect(self._toggle_dark_mode)

        help_menu = mb.addMenu("Help")
        help_menu.addAction("About", self._show_about)

    # --- Central Layout ---

    def _build_central(self):
        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)
        layout.setContentsMargins(8, 8, 8, 8)

        # Robot configuration
        config_header = QLabel("ROBOT CONFIGURATION")
        config_header.setProperty("role", "section")
        layout.addWidget(config_header)

        config_box = QFrame()
        config_box.setObjectName("configStrip")
        config_box.setFrameShape(QFrame.Shape.NoFrame)
        config_layout = QHBoxLayout(config_box)
        config_layout.setContentsMargins(0, 0, 0, 8)
        config_layout.setSpacing(8)
        config_layout.addWidget(QLabel("Robot IP"))
        default_ip = self._beamline_config.get("robot", {}).get("ip", "")
        # Display the configured IP without overriding the orchestrator's connection.
        self.robot_ip_edit = QLineEdit(default_ip)
        self.robot_ip_edit.setMaximumWidth(150)
        self.robot_ip_edit.setReadOnly(True)
        self.robot_ip_edit.setPlaceholderText("not configured")
        self.robot_ip_edit.setToolTip(
            "Sourced from $BEAMBOT_BEAMLINE_CONFIG (robot.ip). "
            "Edit the YAML and restart to change."
        )
        config_layout.addWidget(self.robot_ip_edit)
        config_layout.addSpacing(12)
        config_layout.addWidget(QLabel("Start Gripper"))
        self.gripper_combo = QComboBox()
        self.gripper_combo.addItems(list(self._beamline_config.get("grippers", {}).keys()))
        # A different gripper requires a different robot model and preview.
        self.gripper_combo.currentTextChanged.connect(
            lambda _: self._set_trajectory_cached(False)
        )
        config_layout.addWidget(self.gripper_combo)
        test_btn = QPushButton("Test Server")
        test_btn.clicked.connect(self.ros2.test_server)
        config_layout.addWidget(test_btn)
        poses_btn = QPushButton("Manage Poses")
        poses_btn.clicked.connect(self._manage_poses)
        config_layout.addWidget(poses_btn)
        if ROS2_AVAILABLE:
            save_pose_btn = QPushButton("Save Current Pose")
            save_pose_btn.clicked.connect(self._save_current_pose)
            config_layout.addWidget(save_pose_btn)
        config_layout.addStretch()
        layout.addWidget(config_box)

        # Sidebar, task sequence and monitoring panels
        self.main_splitter = QSplitter(Qt.Orientation.Horizontal)
        layout.addWidget(self.main_splitter, stretch=1)

        self.main_splitter.addWidget(self._build_sidebar())
        self.main_splitter.addWidget(self._build_center_pane())
        self.main_splitter.addWidget(self._build_right_tabs())

        # Give the task sequence most of the available width.
        self.main_splitter.setStretchFactor(0, 0)
        self.main_splitter.setStretchFactor(1, 3)
        self.main_splitter.setStretchFactor(2, 2)
        self.main_splitter.setSizes([240, 720, 440])

    # --- Layout sub-builders ---

    def _build_sidebar(self) -> QWidget:
        """Narrow left sidebar with TASKS | POSES | RUNS tabs."""
        self.left_tabs = QTabWidget()
        self.left_tabs.setMinimumWidth(220)
        self.left_tabs.setMaximumWidth(320)

        # Task palette
        tasks_tab = QWidget()
        tasks_layout = QVBoxLayout(tasks_tab)
        tasks_layout.setContentsMargins(6, 6, 6, 6)
        tasks_layout.setSpacing(6)

        add_label = QLabel("ADD TASK")
        add_label.setProperty("role", "section")
        tasks_layout.addWidget(add_label)

        self.task_toolbar = QListWidget()
        self.task_toolbar.setSelectionMode(QListWidget.SelectionMode.NoSelection)
        self.task_toolbar.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.task_toolbar.setFrameShape(QFrame.Shape.NoFrame)
        self.task_toolbar.setSpacing(2)
        self.task_toolbar.setObjectName("taskPalette")
        self.task_toolbar.setStyleSheet(
            "QListWidget#taskPalette {"
            "    background-color: transparent;"
            "    border: none;"
            "    padding: 0;"
            "}"
            "QListWidget#taskPalette::item {"
            "    padding: 8px 10px;"
            "    border-radius: 6px;"
            "    color: #E6EAF2;"
            "}"
            "QListWidget#taskPalette::item:hover {"
            "    background-color: #1F2C45;"
            "}"
        )
        palette_items = [
            ("Move To", "moveto"),
            ("Pick Sample", "pick_sample"),
            ("Place Sample", "place_sample"),
            ("Place Spincoater", "place_spincoater"),
            ("Pick Spincoater", "pick_spincoater"),
            ("Tool Exchange", "tool_exchange"),
            ("End Effector", "end_effector"),
            ("Vision MoveTo", "vision_moveto"),
            ("Vision Scan", "vision_scan"),
            ("Pipettor", "pipettor"),
            ("Pickup Tip", "pickup_tip"),
            ("Vial Rack", "pickup_vial"),
            ("Pause", "pause"),
        ]
        item_height = 32
        for label, task_type in palette_items:
            cfg = TASK_TYPE_CONFIG.get(task_type, {})
            icon = cfg.get("icon", "+")
            item = QListWidgetItem(f"  {icon}   {label}")
            item.setData(Qt.ItemDataRole.UserRole, task_type)
            item.setSizeHint(QSize(0, item_height))
            self.task_toolbar.addItem(item)
        self.task_toolbar.itemClicked.connect(
            lambda it: self._add_task(it.data(Qt.ItemDataRole.UserRole))
        )
        # Keep every task type visible without scrolling.
        palette_h = (
            item_height * len(palette_items)
            + 2 * self.task_toolbar.spacing() * len(palette_items)
            + 4
        )
        self.task_toolbar.setFixedHeight(palette_h)
        self.task_toolbar.setVerticalScrollBarPolicy(
            Qt.ScrollBarPolicy.ScrollBarAlwaysOff
        )
        tasks_layout.addWidget(self.task_toolbar)

        templates_label = QLabel("TEMPLATES")
        templates_label.setProperty("role", "section")
        tasks_layout.addSpacing(8)
        tasks_layout.addWidget(templates_label)

        templates_placeholder = QLabel(
            "No saved templates yet — save a sequence of steps to reuse it."
        )
        templates_placeholder.setProperty("role", "hint")
        templates_placeholder.setWordWrap(True)
        templates_placeholder.setStyleSheet(
            "QLabel { color: #6B7385; font-size: 11px; padding: 6px 4px;"
            " background: transparent; }"
        )
        tasks_layout.addWidget(templates_placeholder)

        tasks_layout.addStretch(1)

        self.left_tabs.addTab(tasks_tab, "Tasks")

        # Poses
        self.poses_panel = PosesPanel()
        self.poses_panel.poses_loaded.connect(self._on_poses_loaded)
        self.poses_panel.pose_activated.connect(
            lambda name, _values: self._add_moveto_for_pose(name)
        )
        self.left_tabs.addTab(self.poses_panel, "Poses")

        # Saved runs
        runs_tab = QWidget()
        runs_layout = QVBoxLayout(runs_tab)
        runs_layout.setContentsMargins(6, 6, 6, 6)
        runs_layout.setSpacing(6)

        runs_btn_row = QHBoxLayout()
        self._save_run_btn = QPushButton("Save")
        self._save_run_btn.setToolTip("Save the current task sequence as a named run")
        self._save_run_btn.clicked.connect(self._save_run)
        runs_btn_row.addWidget(self._save_run_btn)

        self._rename_run_btn = QPushButton("Rename")
        self._rename_run_btn.setToolTip("Rename the selected run")
        self._rename_run_btn.clicked.connect(self._rename_run)
        runs_btn_row.addWidget(self._rename_run_btn)

        self._delete_run_btn = QPushButton("Delete")
        self._delete_run_btn.setToolTip("Delete the selected run")
        self._delete_run_btn.clicked.connect(self._delete_run)
        runs_btn_row.addWidget(self._delete_run_btn)

        self._refresh_runs_btn = QPushButton("↻")
        self._refresh_runs_btn.setFixedWidth(28)
        self._refresh_runs_btn.setToolTip("Reload the runs list from disk")
        self._refresh_runs_btn.clicked.connect(self._refresh_runs_list)
        runs_btn_row.addWidget(self._refresh_runs_btn)
        runs_layout.addLayout(runs_btn_row)

        self._runs_list = QListWidget()
        self._runs_list.setStyleSheet(
            "QListWidget { background-color: #161B24; border: 1px solid #2C3448;"
            " border-radius: 6px; outline: none; padding: 4px; }"
            "QListWidget::item { padding: 4px 8px; }"
            "QListWidget::item:selected { background-color: #1F2C45; }"
            "QListWidget::item:hover { background-color: #1A2538; }"
        )
        self._runs_list.itemDoubleClicked.connect(self._load_run)
        runs_layout.addWidget(self._runs_list, stretch=1)

        hint = QLabel("Double-click a run to load it into the active sequence.")
        hint.setProperty("role", "hint")
        hint.setWordWrap(True)
        hint.setStyleSheet("color: #6B7385; font-size: 11px;")
        runs_layout.addWidget(hint)

        self.left_tabs.addTab(runs_tab, "Runs")
        self._refresh_runs_list()

        return self.left_tabs

    def _build_center_pane(self) -> QWidget:
        """Center pane: execution toolbar (top), step list (middle), status log (bottom)."""
        center = QWidget()
        center_layout = QVBoxLayout(center)
        center_layout.setContentsMargins(4, 4, 4, 4)
        center_layout.setSpacing(4)

        # Execution controls
        exec_bar = QFrame()
        exec_bar.setFrameShape(QFrame.Shape.NoFrame)
        exec_layout = QHBoxLayout(exec_bar)
        exec_layout.setContentsMargins(4, 4, 4, 4)
        exec_layout.setSpacing(8)

        run_bar = QFrame()
        run_bar.setObjectName("runBar")
        run_bar.setFrameShape(QFrame.Shape.NoFrame)
        run_layout = QHBoxLayout(run_bar)
        run_layout.setContentsMargins(0, 0, 0, 0)
        run_layout.setSpacing(0)

        self.exec_btn = QPushButton("Execute")
        self.exec_btn.setObjectName("segFirst")
        self.exec_btn.setIcon(theme.icon("mdi6.play", color=theme.ON_PRIMARY))
        self.exec_btn.setProperty("class", "primary")
        self.exec_btn.clicked.connect(self._execute)
        run_layout.addWidget(self.exec_btn)
        self.execute_from_btn = QPushButton("From Selected")
        self.execute_from_btn.setIcon(
            theme.icon("mdi6.play-skip-forward", color=theme.ON_SURFACE_DIM)
        )
        self.execute_from_btn.setEnabled(False)
        self.execute_from_btn.setToolTip(
            "Run the selected step and all following steps. "
            "Set Start Gripper to the tool currently on the robot first."
        )
        self.execute_from_btn.clicked.connect(self._execute_from_selected)
        run_layout.addWidget(self.execute_from_btn)
        self.pause_btn = QPushButton("Pause")
        self.pause_btn.setIcon(theme.icon("mdi6.pause", color=theme.ON_SURFACE_DIM))
        self.pause_btn.setEnabled(False)
        self.pause_btn.clicked.connect(self.ros2.pause_task)
        run_layout.addWidget(self.pause_btn)
        self.resume_btn = QPushButton("Resume")
        self.resume_btn.setIcon(theme.icon("mdi6.play-outline", color=theme.ON_SURFACE_DIM))
        self.resume_btn.setEnabled(False)
        self.resume_btn.clicked.connect(self.ros2.resume_task)
        run_layout.addWidget(self.resume_btn)
        self.stop_btn = QPushButton("Stop")
        self.stop_btn.setObjectName("segLast")
        self.stop_btn.setIcon(theme.icon("mdi6.stop", color=theme.ON_SURFACE_DIM))
        self.stop_btn.setProperty("class", "danger")
        self.stop_btn.setEnabled(False)
        self.stop_btn.clicked.connect(self.ros2.stop_execution)
        run_layout.addWidget(self.stop_btn)
        exec_layout.addWidget(run_bar)
        exec_layout.addSpacing(12)

        self.dry_run_check = QCheckBox("Dry Run")
        self.dry_run_check.setToolTip(
            "Plan-only preview: animate the planned motion in the 3D view "
            "without moving the robot. v1 supports moveto + end_effector only.\n\n"
            "After a successful Dry Run, Execute will replay the previewed "
            "plan exactly — no re-planning."
        )
        exec_layout.addWidget(self.dry_run_check)

        self.trajectory_cached_label = QLabel("")
        self.trajectory_cached_label.setProperty("status", "success")
        self.trajectory_cached_label.setVisible(False)
        exec_layout.addWidget(self.trajectory_cached_label)

        exec_layout.addSpacing(8)
        self.progress_bar = QProgressBar()
        self.progress_bar.setVisible(False)
        exec_layout.addWidget(self.progress_bar, stretch=1)
        exec_layout.addStretch(1)
        center_layout.addWidget(exec_bar)

        # Step list
        self.step_list = StepListPanel()
        self.step_list.addActions([self.copy_action, self.paste_action])
        self.step_list.item_double_clicked.connect(self._edit_task_by_index)
        self.step_list.selection_changed.connect(self._update_execute_from_selected)
        self.step_list.pose_dropped.connect(self._add_moveto_for_pose)
        self.step_list.steps_reordered.connect(self._reorder_steps)
        self.step_list.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding
        )
        center_layout.addWidget(self.step_list, stretch=1)

        # Step editing controls
        step_ops_bar = QFrame()
        step_ops_bar.setFrameShape(QFrame.Shape.NoFrame)
        step_ops = QHBoxLayout(step_ops_bar)
        step_ops.setContentsMargins(4, 0, 4, 0)
        step_ops.setSpacing(4)
        self.up_step_btn = QPushButton("Up")
        self.up_step_btn.setIcon(theme.icon("mdi6.arrow-up", color=theme.ON_SURFACE_MUTED))
        self.up_step_btn.clicked.connect(self._move_up)
        self.down_step_btn = QPushButton("Down")
        self.down_step_btn.setIcon(theme.icon("mdi6.arrow-down", color=theme.ON_SURFACE_MUTED))
        self.down_step_btn.clicked.connect(self._move_down)
        self.remove_step_btn = QPushButton("Remove")
        self.remove_step_btn.setIcon(theme.icon("mdi6.close", color=theme.ON_SURFACE_MUTED))
        self.remove_step_btn.clicked.connect(self._remove_task)
        self.clear_steps_btn = QPushButton("Clear")
        self.clear_steps_btn.setIcon(theme.icon("mdi6.broom", color=theme.ON_SURFACE_MUTED))
        self.clear_steps_btn.setToolTip("Remove all steps from the sequence")
        self.clear_steps_btn.clicked.connect(self._clear_tasks)
        for b in (
            self.up_step_btn,
            self.down_step_btn,
            self.remove_step_btn,
            self.clear_steps_btn,
        ):
            b.setCursor(Qt.CursorShape.PointingHandCursor)
            step_ops.addWidget(b)
        step_ops.addStretch(1)
        center_layout.addWidget(step_ops_bar)

        # Activity log
        log_label = QLabel("ACTIVITY LOG")
        log_label.setProperty("role", "section")
        center_layout.addWidget(log_label)
        self.status_log = QTextEdit()
        self.status_log.setReadOnly(True)
        self.status_log.setMaximumHeight(140)
        self.status_log.setObjectName("activityLog")
        mono = QFont("JetBrains Mono")
        if not mono.exactMatch():
            mono = QFont("DejaVu Sans Mono")
        mono.setPointSize(9)
        self.status_log.setFont(mono)
        self.status_log.setStyleSheet(
            f"QTextEdit#activityLog {{ background-color: {theme.SURFACE};"
            f" border: 1px solid {theme.OUTLINE}; border-radius: 6px;"
            f" padding: 8px 10px; color: {theme.ON_SURFACE_MUTED}; }}"
        )
        center_layout.addWidget(self.status_log)

        return center

    def _build_right_tabs(self) -> QWidget:
        """Build the chat, camera and optional 3D-view tabs."""
        self.right_tabs = QTabWidget()

        self.chat_panel = ChatPanel()
        self.agent_bridge = AgentBridge()
        self.right_tabs.addTab(self.chat_panel, "Chat")

        if ROS2_AVAILABLE:
            self.camera = CameraPanel(self.ros2)
            self.right_tabs.addTab(self.camera, "Camera")

        if WEBENGINE_AVAILABLE:
            self.viz_panel = VisualizationPanel(self.ros2)
            self._viz_tab_title = "3D View"
            self.right_tabs.addTab(self.viz_panel, self._viz_tab_title)

        self._viz_floating = False
        return self.right_tabs

    # --- Signal Connections ---

    def _connect_signals(self):
        self.ros2.log.connect(self._log)
        self.ros2.joint_state_received.connect(self._on_joint_state)
        self.ros2.action_feedback_received.connect(self._on_feedback)
        self.ros2.action_result_received.connect(self._on_result)
        self.ros2.execution_state_changed.connect(self._on_execution_state)
        self.ros2.gripper_changed.connect(self._on_gripper_changed)
        if self.ros2.execution_state is not None:
            self._on_execution_state(self.ros2.execution_state)
        else:
            self._project_execution_state()

        # 3D visualization panel
        if WEBENGINE_AVAILABLE and hasattr(self, "viz_panel"):
            self.ros2.joint_state_received.connect(self.viz_panel._on_joint_state)
            self.ros2.gripper_changed.connect(self.viz_panel.set_gripper)
            self.ros2.preview_trajectory_received.connect(
                self.viz_panel.play_trajectory
            )

        # Agent messages and connection status
        self.chat_panel.message_submitted.connect(self._on_chat_message)
        self.agent_bridge.response_received.connect(self.chat_panel.append_assistant)
        self.agent_bridge.tool_called.connect(self.chat_panel.append_tool_call)
        self.agent_bridge.thinking_changed.connect(self.chat_panel.set_thinking)
        self.agent_bridge.error_occurred.connect(self.chat_panel.append_error)
        self.agent_bridge.connected.connect(
            lambda n: self._log(f"Agent connected: {n} tools")
        )
        self.agent_bridge.connected.connect(
            lambda n: self.chat_panel.set_status(f"Connected ({n} tools)")
        )

        # Agent task queue
        self.agent_bridge.tasks_proposed.connect(self._on_tasks_proposed)
        self.agent_bridge.tasks_cleared.connect(self._on_agent_tasks_cleared)
        self.agent_bridge.execution_requested.connect(self._on_agent_execute_requested)

        # Agent mode
        self.chat_panel.mode_change_requested.connect(self.agent_bridge.set_mode)
        self.agent_bridge.mode_changed.connect(self._on_agent_mode_changed)

        # Show agent-initiated execution results in chat.
        self.agent_bridge.execution_outcome.connect(
            self.chat_panel.append_execution_outcome
        )

        # Connect the agent after wiring its signals.
        self.agent_bridge.set_config_getter(lambda: self.config)
        self.agent_bridge.connect_agent()

    # --- Task Management ---

    def _add_task(self, task_type):
        import copy

        step = copy.deepcopy(
            self._task_defaults.get(task_type, {"task_type": task_type})
        )
        self.config["tasks"].append(step)
        self._refresh_tree()
        self._log(f"Added {task_type}")

    def _add_moveto_for_pose(self, pose_name: str):
        """Append a Move To step targeting the named pose (joint planning)."""
        if not pose_name:
            return
        import copy

        step = copy.deepcopy(self._task_defaults["moveto"])
        step["target"] = pose_name
        self.config["tasks"].append(step)
        self._refresh_tree()
        self._log(f"Added Move To '{pose_name}'")

    def _remove_task(self):
        indices = sorted(self.step_list.selected_indices(), reverse=True)
        if not indices:
            return
        for i in indices:
            if 0 <= i < len(self.config["tasks"]):
                self.config["tasks"].pop(i)
        self._refresh_tree()

    def _clear_tasks(self):
        n = len(self.config["tasks"])
        if n == 0:
            return
        reply = QMessageBox.question(
            self,
            "Clear sequence",
            f"Remove all {n} step{'s' if n != 1 else ''} from the sequence?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.Cancel,
            QMessageBox.StandardButton.Cancel,
        )
        if reply != QMessageBox.StandardButton.Yes:
            return
        self.config["tasks"].clear()
        self._refresh_tree()
        self._log(f"Cleared {n} step{'s' if n != 1 else ''}")

    def _copy_steps(self):
        """Copy selected steps to clipboard as JSON."""
        indices = self.step_list.selected_indices()
        if not indices:
            return
        selected = [self.config["tasks"][i] for i in indices]
        payload = json.dumps(selected)
        encoded = payload.encode("utf-8")
        if len(encoded) > STEP_CLIPBOARD_MAX_BYTES:
            self._log("Selection is too large to copy")
            return
        from PyQt6.QtWidgets import QApplication
        mime = QMimeData()
        mime.setData(STEP_CLIPBOARD_MIME, encoded)
        mime.setText(payload)
        QApplication.clipboard().setMimeData(mime)
        self._log(f"Copied {len(selected)} step{'s' if len(selected) != 1 else ''}")

    def _paste_steps(self):
        """Paste steps copied from this editor."""
        if not self.step_list.editing_enabled:
            return
        from PyQt6.QtWidgets import QApplication
        mime = QApplication.clipboard().mimeData()
        if mime is None or not mime.hasFormat(STEP_CLIPBOARD_MIME):
            return
        clip = mime.data(STEP_CLIPBOARD_MIME)
        if clip.isEmpty() or clip.size() > STEP_CLIPBOARD_MAX_BYTES:
            return
        try:
            data = json.loads(bytes(clip))
        except (ValueError, RecursionError):
            return
        if (
            not isinstance(data, list)
            or not data
            or len(data) > STEP_CLIPBOARD_MAX_STEPS
        ):
            return
        # Validate before mutating; task_summary is also the tree render path.
        for item in data:
            if not isinstance(item, dict):
                return
            tt = item.get("task_type")
            if not isinstance(tt, str) or (
                tt not in self._task_defaults and tt != "vision_task"
            ):
                return
            try:
                task_summary(item)
            except (KeyError, TypeError, ValueError, IndexError, OverflowError):
                return
        indices = self.step_list.selected_indices()
        if indices:
            insert_at = max(indices) + 1
        else:
            insert_at = len(self.config["tasks"])
        self.config["tasks"][insert_at:insert_at] = data
        self._refresh_tree()
        self.step_list.select_range(insert_at, len(data))
        self._log(f"Pasted {len(data)} step{'s' if len(data) != 1 else ''}")

    # --- Agent-driven queue updates (Plan/Run mode) ---

    def _on_tasks_proposed(self, tasks: list, options: dict):
        """Apply the agent's proposed tasks to the visible queue."""
        replace = options.get("replace", True)
        if replace:
            self.config["tasks"] = list(tasks)
        else:
            self.config["tasks"].extend(tasks)
        # Execution reads the combobox, so update it along with the proposed tasks.
        sg = options.get("start_gripper")
        if sg:
            idx = self.gripper_combo.findText(sg)
            if idx >= 0:
                self.gripper_combo.setCurrentIndex(idx)
        if options.get("poses"):
            self.config.setdefault("poses", {}).update(options["poses"])
        self._refresh_tree()
        self._log(
            f"Agent proposed {len(tasks)} task(s) "
            f"({'replace' if replace else 'append'})"
        )

    def _on_agent_tasks_cleared(self):
        """Clear the task queue at the agent's request."""
        n = len(self.config["tasks"])
        self.config["tasks"] = []
        self._refresh_tree()
        self._log(f"Agent cleared the task queue ({n} step(s) removed)")

    def _on_agent_execute_requested(self):
        """Execute the agent's queue through the normal execution path."""
        if self.ros2._current_goal_handle is not None:
            self.agent_bridge.notify_execution_complete(
                False, "Another goal is already executing", 0, 0
            )
            return
        if not self.config["tasks"]:
            self.agent_bridge.notify_execution_complete(
                False, "Task queue is empty", 0, 0
            )
            return
        self._execution_initiator = "agent"
        self._log(f"Agent dispatched execution ({len(self.config['tasks'])} task(s))")
        self._execute()

    def _on_agent_mode_changed(self, mode: str):
        """Bridge confirmed the new mode. Reset chat and update the panel."""
        self.chat_panel.clear_chat()
        self.chat_panel.set_mode_label(mode)
        self.chat_panel.set_status(f"Mode: {mode}")
        self.chat_panel.append_assistant(
            f"Switched to {mode.upper()} mode. Conversation reset."
        )
        self._log(f"Agent mode → {mode}")

    def _move_up(self):
        indices = self.step_list.selected_indices()
        if len(indices) != 1:
            return
        idx = indices[0]
        if idx <= 0:
            return
        tasks = self.config["tasks"]
        tasks[idx], tasks[idx - 1] = tasks[idx - 1], tasks[idx]
        self._refresh_tree()
        self.step_list.set_current_row(idx - 1)

    def _move_down(self):
        indices = self.step_list.selected_indices()
        if len(indices) != 1:
            return
        idx = indices[0]
        tasks = self.config["tasks"]
        if idx >= len(tasks) - 1:
            return
        tasks[idx], tasks[idx + 1] = tasks[idx + 1], tasks[idx]
        self._refresh_tree()
        self.step_list.set_current_row(idx + 1)

    def _reorder_steps(self, order: list, moved_rows: list):
        """Apply the list widget's drag order to the task sequence."""
        tasks = self.config["tasks"]
        if (
            not self.step_list.editing_enabled
            or len(order) != len(tasks)
            or set(order) != set(range(len(tasks)))
        ):
            self._refresh_tree()
            return
        self.config["tasks"] = [tasks[i] for i in order]
        self._refresh_tree()
        moved = set(moved_rows)
        self.step_list.select_indices(
            i for i, original_row in enumerate(order) if original_row in moved
        )

    def _detach_viz_for_form(self):
        """Remove the 3D viewer from its tab so the form can embed it."""
        if not (WEBENGINE_AVAILABLE and hasattr(self, "viz_panel")):
            return
        if self._viz_floating:
            return
        self._viz_home_index = self.right_tabs.indexOf(self.viz_panel)
        if self._viz_home_index >= 0:
            self.right_tabs.removeTab(self._viz_home_index)
        self._viz_floating = True

    def _redock_viz_after_form(self):
        """Return the 3D viewer to its tab after the form closes."""
        if not (WEBENGINE_AVAILABLE and hasattr(self, "viz_panel")):
            return
        if not self._viz_floating:
            return
        self.viz_panel.setWindowFlags(Qt.WindowType.Widget)
        idx = min(self._viz_home_index, self.right_tabs.count())
        self.right_tabs.insertTab(idx, self.viz_panel, self._viz_tab_title)
        self.viz_panel.show()
        self._viz_floating = False

    def _edit_task_by_index(self, idx):
        if idx < 0 or idx >= len(self.config["tasks"]):
            return
        step = self.config["tasks"][idx]
        if step.get("task_type") == "pause":
            return

        from .task_forms import open_task_form

        # Include registry poses without storing them inline; inline names take precedence.
        merged_poses = {**self.poses_panel.get_poses(), **self.config.get("poses", {})}
        preview_cb = self.viz_panel.preview_pose if (WEBENGINE_AVAILABLE and hasattr(self, "viz_panel")) else None
        end_preview_cb = self.viz_panel.end_preview if (WEBENGINE_AVAILABLE and hasattr(self, "viz_panel")) else None
        viz_widget = self.viz_panel if (WEBENGINE_AVAILABLE and hasattr(self, "viz_panel")) else None
        self._detach_viz_for_form()
        try:
            result = open_task_form(step, idx, merged_poses, self,
                                    current_pose=self.current_robot_pose,
                                    preview_cb=preview_cb,
                                    end_preview_cb=end_preview_cb,
                                    viz_widget=viz_widget)
        finally:
            self._redock_viz_after_form()
        if result is not None:
            if "_inline_joint_pose" in result:
                ijp = result.pop("_inline_joint_pose")
                self.config.setdefault("poses", {})[ijp["name"]] = ijp["values"]
                self._log(f"Added inline joint pose '{ijp['name']}'")
            self.config["tasks"][idx] = result
            self._refresh_tree()
            self._log(f"Updated step {idx + 1}")

    def _refresh_tree(self):
        self.step_list.refresh(self.config["tasks"], task_summary)
        # Task edits make the displayed preview indicator stale.
        self._set_trajectory_cached(False)

    def _set_trajectory_cached(self, cached: bool, reason: str = ""):
        """Update the 'Trajectory cached' indicator next to the Dry Run checkbox."""
        from . import theme
        if cached:
            self.trajectory_cached_label.setProperty("status", "success")
            self.trajectory_cached_label.setText(
                "✓ Trajectory cached — Execute will replay preview"
            )
            self.trajectory_cached_label.setVisible(True)
        elif reason:
            self.trajectory_cached_label.setProperty("status", "warning")
            self.trajectory_cached_label.setText(f"⚠ {reason}")
            self.trajectory_cached_label.setVisible(True)
            from PyQt6.QtCore import QTimer

            QTimer.singleShot(8000, self._clear_trajectory_cache_warning)
        else:
            self.trajectory_cached_label.setText("")
            self.trajectory_cached_label.setVisible(False)
        theme.restyle(self.trajectory_cached_label)

    def _clear_trajectory_cache_warning(self):
        from . import theme
        self.trajectory_cached_label.setText("")
        self.trajectory_cached_label.setVisible(False)
        self.trajectory_cached_label.setProperty("status", "success")
        theme.restyle(self.trajectory_cached_label)

    # --- Execution ---

    # Match the orchestrator's DRY_RUN_SUPPORTED_TYPES for immediate GUI validation.
    _DRY_RUN_SUPPORTED = {"moveto", "end_effector"}

    def _execute_from_selected(self):
        indices = self.step_list.selected_indices()
        if len(indices) == 1:
            self._execute(indices[0])

    def _execute(self, start_index: int = 0):
        if not self.config["tasks"]:
            QMessageBox.warning(self, "Warning", "No tasks defined")
            return

        if not 0 <= start_index < len(self.config["tasks"]):
            return

        tasks = self.config["tasks"][start_index:]

        dry_run = self.dry_run_check.isChecked()
        if dry_run:
            unsupported = [
                (start_index + i + 1, t.get("task_type", "?"))
                for i, t in enumerate(tasks)
                if t.get("task_type", "") not in self._DRY_RUN_SUPPORTED
            ]
            if unsupported:
                bad = "\n  - ".join(f"step {n}: {tt}" for n, tt in unsupported)
                QMessageBox.warning(
                    self,
                    "Dry Run Unavailable",
                    f"Dry-run preview only supports moveto + end_effector in v1.\n\n"
                    f"Unsupported steps:\n  - {bad}\n\n"
                    f"Uncheck 'Dry Run' to execute, or remove these steps to preview.",
                )
                return

        self.config["start_gripper"] = self.gripper_combo.currentText()
        execution_config = {**self.config, "tasks": tasks}
        self._last_goal_was_dry_run = dry_run
        self._execution_start_index = start_index
        self._goal_pending = True
        self._project_execution_state()
        self.progress_bar.setValue(int(start_index / len(self.config["tasks"]) * 100))
        self.progress_bar.setVisible(True)
        self.step_list.start_execution(len(self.config["tasks"]), start_index)

        self.ros2.execute_task(json.dumps(execution_config), dry_run=dry_run)

    def _on_execution_state(self, state):
        self._execution_state = state
        if state in {"RUNNING", "COMPLETING_TASK", "PAUSED"}:
            self._goal_pending = False
        elif state != "IDLE":
            self._log(f"Unknown execution state: {state}")
        self._project_execution_state()

    def _project_execution_state(self):
        execute, pause, resume, stop, editing, paused = _execution_controls(
            self._execution_state, self._goal_pending
        )
        self.exec_btn.setEnabled(execute)
        self._update_execute_from_selected()
        self.pause_btn.setEnabled(pause)
        self.resume_btn.setEnabled(resume)
        self.stop_btn.setEnabled(stop)
        for control in (
            self.task_toolbar,
            self.up_step_btn,
            self.down_step_btn,
            self.remove_step_btn,
            self.clear_steps_btn,
            self.gripper_combo,
            self.dry_run_check,
        ):
            control.setEnabled(editing)
        self.step_list.set_editing_enabled(editing)
        self.step_list.set_paused(paused)

    def _update_execute_from_selected(self, _=None):
        self.execute_from_btn.setEnabled(
            self.exec_btn.isEnabled() and len(self.step_list.selected_indices()) == 1
        )

    def _on_feedback(self, progress, step, action, gripper, msg):
        total = len(self.config["tasks"])
        remaining = total - self._execution_start_index
        self.progress_bar.setValue(
            int(
                (self._execution_start_index + progress / 100 * remaining)
                / total
                * 100
            )
            if total
            else 0
        )
        self.step_list.update_step(step, progress, action)
        self._log(
            f"[{progress:.0f}%] Step {self._execution_start_index + step}: "
            f"{action} | {gripper} | {msg}"
        )

    def _on_result(self, status, error_msg, completed, total):
        self._goal_pending = False
        was_dry_run = self._last_goal_was_dry_run
        self._last_goal_was_dry_run = False

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.progress_bar.setValue(100)
            self.step_list.finish_execution("success", completed)
            if was_dry_run:
                self._log(
                    f"Dry-run preview complete: {completed}/{total} steps planned"
                )
                self._set_trajectory_cached(True)
            else:
                self._log(f"Task completed: {completed}/{total} steps")
                # Clear the preview indicator; the server may still retain the cached plan.
                self._set_trajectory_cached(False)
        elif status == GoalStatus.STATUS_CANCELED:
            self.step_list.finish_execution("cancelled", completed)
            self._log(f"Task cancelled: {error_msg}")
        else:
            self.step_list.finish_execution("failed", completed)
            if error_msg.startswith("CACHE_"):
                # Extract the message from "CACHE_<REASON>: <message>".
                reason = (
                    error_msg.split(":", 1)[1].strip()
                    if ":" in error_msg
                    else error_msg
                )
                self._log(f"Cached trajectory invalid: {reason}")
                self._set_trajectory_cached(False, "Cached trajectory invalid — run Dry Run again")
                QMessageBox.information(
                    self,
                    "Cached Plan Invalid",
                    f"The previewed plan can no longer be executed:\n\n{reason}\n\n"
                    "Click Dry Run to preview the plan from the current state, "
                    "then Execute.",
                )
            else:
                self._log(f"Task failed: {error_msg} ({completed}/{total} steps)")

        # Resolve the agent's pending execute_queue request.
        if self._execution_initiator == "agent":
            success = status == GoalStatus.STATUS_SUCCEEDED
            self.agent_bridge.notify_execution_complete(
                success, error_msg, completed, total
            )
        self._execution_initiator = "human"
        self._execution_start_index = 0
        self._project_execution_state()

    def _on_joint_state(self, pose):
        self.current_robot_pose = pose

    def _on_gripper_changed(self, gripper: str):
        index = self.gripper_combo.findText(gripper)
        if index >= 0:
            self.gripper_combo.setCurrentIndex(index)

    # --- JSON I/O ---

    def _load_json(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "Load JSON", "", "JSON (*.json);;All (*)"
        )
        if not path:
            return
        try:
            with open(path) as f:
                self.config = json.load(f)
            if "start_gripper" in self.config:
                idx = self.gripper_combo.findText(self.config["start_gripper"])
                if idx >= 0:
                    self.gripper_combo.setCurrentIndex(idx)
            self.current_json_file = path
            self._refresh_tree()
            self._log(f"Loaded {path}")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to load: {e}")

    def _save_json(self):
        path, _ = QFileDialog.getSaveFileName(
            self, "Save JSON", "", "JSON (*.json);;All (*)"
        )
        if not path:
            return
        try:
            self.config["start_gripper"] = self.gripper_combo.currentText()
            with open(path, "w") as f:
                json.dump(self.config, f, indent=2)
            self._log(f"Saved to {path}")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to save: {e}")

    # --- Runs Management ---

    def _runs_dir(self) -> "Path | None":
        """Resolve the runs directory (src/cms/runs/) from the beamline config path."""
        if not self._beamline_config_path:
            return None
        config_path = Path(self._beamline_config_path)
        workspace_root = config_path.parent
        while workspace_root != workspace_root.parent:
            if (workspace_root / "src" / "cms").exists():
                break
            workspace_root = workspace_root.parent
        else:
            return None
        runs_dir = workspace_root / "src" / "cms" / "runs"
        runs_dir.mkdir(parents=True, exist_ok=True)
        return runs_dir

    def _refresh_runs_list(self):
        """Reload the runs list from the cms/runs/ directory."""
        self._runs_list.clear()
        runs_dir = self._runs_dir()
        if not runs_dir:
            return
        for f in sorted(runs_dir.glob("*.json")):
            try:
                with open(f) as fh:
                    data = json.load(fh)
                display_name = data.get("run_name", f.stem)
            except Exception:
                display_name = f.stem
            item = QListWidgetItem(display_name)
            item.setData(Qt.ItemDataRole.UserRole, str(f))
            item.setToolTip(f"{f.name}  ({display_name})")
            self._runs_list.addItem(item)

    def _save_run(self):
        """Save the current task sequence as a named run."""
        runs_dir = self._runs_dir()
        if not runs_dir:
            QMessageBox.warning(
                self, "Cannot Save",
                "BEAMBOT_BEAMLINE_CONFIG not set — cannot locate runs directory."
            )
            return
        if not self.config["tasks"]:
            QMessageBox.warning(self, "Empty", "No tasks to save.")
            return

        from PyQt6.QtWidgets import QInputDialog
        display_name, ok = QInputDialog.getText(
            self, "Save Run", "Display name (shown in the list):",
        )
        if not ok or not display_name.strip():
            return
        display_name = display_name.strip()

        # Suggest a filename from the display name.
        default_filename = display_name.lower().replace(" ", "_").replace("/", "_")
        filename, ok = QInputDialog.getText(
            self, "Save Run", "Filename (without .json):",
            text=default_filename,
        )
        if not ok or not filename.strip():
            return
        filename = filename.strip().replace(" ", "_").replace("/", "_")
        if not filename.endswith(".json"):
            filename += ".json"

        path = runs_dir / filename
        if path.exists():
            reply = QMessageBox.question(
                self, "Overwrite?",
                f"'{filename}' already exists. Overwrite?",
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.Cancel,
            )
            if reply != QMessageBox.StandardButton.Yes:
                return

        try:
            import copy
            # Keep registry poses as name references; embed only inline poses (#94).
            save_data = copy.deepcopy(self.config)
            save_data["start_gripper"] = self.gripper_combo.currentText()
            save_data["run_name"] = display_name
            with open(path, "w") as f:
                json.dump(save_data, f, indent=2)
            self._log(f"Saved run: {display_name} → {filename}")
            self._refresh_runs_list()
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to save run: {e}")

    def _load_run(self, item: "QListWidgetItem"):
        """Load a run from the runs list into the active sequence."""
        path = item.data(Qt.ItemDataRole.UserRole)
        if not path:
            return
        try:
            with open(path) as f:
                self.config = json.load(f)
            if "start_gripper" in self.config:
                idx = self.gripper_combo.findText(self.config["start_gripper"])
                if idx >= 0:
                    self.gripper_combo.setCurrentIndex(idx)
            self.current_json_file = path
            self._refresh_tree()
            self._log(f"Loaded run: {item.text()}")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to load run: {e}")

    def _rename_run(self):
        """Rename the display name of the selected run."""
        item = self._runs_list.currentItem()
        if not item:
            return
        path = item.data(Qt.ItemDataRole.UserRole)
        current_name = item.text()

        from PyQt6.QtWidgets import QInputDialog
        new_name, ok = QInputDialog.getText(
            self, "Rename Run", "New display name:",
            text=current_name,
        )
        if not ok or not new_name.strip() or new_name.strip() == current_name:
            return

        try:
            with open(path) as f:
                data = json.load(f)
            data["run_name"] = new_name.strip()
            with open(path, "w") as f:
                json.dump(data, f, indent=2)
            self._log(f"Renamed: {current_name} → {new_name.strip()}")
            self._refresh_runs_list()
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to rename: {e}")

    def _delete_run(self):
        """Delete the selected run file."""
        item = self._runs_list.currentItem()
        if not item:
            return
        path = item.data(Qt.ItemDataRole.UserRole)
        name = item.text()
        reply = QMessageBox.question(
            self, "Delete Run",
            f"Delete '{name}'?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.Cancel,
        )
        if reply != QMessageBox.StandardButton.Yes:
            return
        try:
            Path(path).unlink()
            self._log(f"Deleted run: {name}")
            self._refresh_runs_list()
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to delete: {e}")

    # --- Logging ---

    def _log(self, msg):
        ts = time.strftime("%H:%M:%S")
        self.status_log.append(f"[{ts}] {msg}")

    def _toggle_dark_mode(self, enabled):
        theme.toggle_dark_mode(self._app(), enabled)

    def _app(self):
        from PyQt6.QtWidgets import QApplication as _QApp

        return _QApp.instance()

    def _show_about(self):
        QMessageBox.about(
            self,
            "About",
            (
                "MTC GUI Client (PyQt6)\n\n"
                "Task sequence builder for beambot orchestrator.\n"
                "Communicates via ROS2 ActionClient.\n\n"
                "Action server: beambot_execution"
            ),
        )

    # --- Pose Management ---

    def _load_beamline_yaml(self) -> tuple[dict, str | None]:
        """Load the beamline YAML; return ({}, None) on failure to allow JSON inspection."""
        import os
        import yaml
        raw = os.environ.get("BEAMBOT_BEAMLINE_CONFIG", "").strip()
        if not raw:
            self._pending_log = (
                "BEAMBOT_BEAMLINE_CONFIG not set; robot IP and pose "
                "registry will be empty until you set it and restart."
            )
            return {}, None
        # Preserve unknown '~user' handling and normalize '..' without following symlinks.
        path = Path(os.path.abspath(os.path.expanduser(raw)))
        if not path.is_file():
            self._pending_log = f"BEAMBOT_BEAMLINE_CONFIG points at missing file: {path}"
            return {}, None
        try:
            with path.open() as f:
                data = yaml.safe_load(f) or {}
            if not isinstance(data, dict):
                self._pending_log = f"{path}: expected a YAML mapping at root"
                return {}, None
            return data, str(path)
        except Exception as e:
            self._pending_log = f"Failed to parse {path}: {e}"
            return {}, None

    def _load_beamline_poses(self):
        """Push poses from the loaded beamline config into the poses panel."""
        if self._beamline_config_path:
            self.poses_panel.load_from_beamline_config(self._beamline_config_path)
        if hasattr(self, "_pending_log"):
            self._log(self._pending_log)
            del self._pending_log

    def _on_poses_loaded(self, poses: dict):
        """Log registry loading without copying poses inline into saved runs (#94)."""
        self._log(f"Loaded {len(poses)} poses from registry")

    def _manage_poses(self):
        # Dialog persists every edit through the panel; no result to apply.
        PosesManagerDialog(self.poses_panel, self).exec()

    def _save_current_pose(self):
        if self.current_robot_pose is None:
            QMessageBox.warning(
                self, "No Pose", "No robot pose available. Is the robot connected?"
            )
            return
        dlg = SavePoseDialog(
            self.current_robot_pose, self.poses_panel.get_poses_file(), self
        )
        if dlg.exec() == QDialog.DialogCode.Accepted and dlg.result is not None:
            r = dlg.result
            name = r["pose_name"]
            values = r["pose_values"]
            if r["action"] == "save_to_registry":
                if self.poses_panel.save_pose(name, values):
                    self._log(f"Saved pose '{name}' to registry")
                else:
                    QMessageBox.warning(
                        self, "No Registry",
                        "No pose registry file is loaded — cannot save."
                    )
            elif r["action"] == "save_inline":
                self.config.setdefault("poses", {})[name] = values
                self._log(f"Added inline pose '{name}' (this task only)")

    def _on_chat_message(self, text):
        self.chat_panel.append_user(text)
        self.agent_bridge.send_message(text)

    def closeEvent(self, event):
        self.agent_bridge.disconnect()
        event.accept()
