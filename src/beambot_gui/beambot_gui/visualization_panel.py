"""3D robot visualization panel using QWebEngineView + Three.js + urdf-loaders."""

import json
import math
import re
import time
import threading
import yaml
from http.server import HTTPServer, SimpleHTTPRequestHandler
from pathlib import Path
from functools import partial
from urllib.parse import unquote, urlsplit

from PyQt6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel
from PyQt6.QtCore import Qt, QUrl, QTimer

try:
    from PyQt6.QtWebEngineWidgets import QWebEngineView
    WEBENGINE_AVAILABLE = True
except ImportError:
    WEBENGINE_AVAILABLE = False

# GRIPPER_URDF_MAP is now sourced per-gripper from the active beamline YAML
# (grippers.<name>.urdf_file) via config_loader.gripper_urdf_file().

# Arm joint order expected by window.updateJoints(...) — sourced from the
# active beamline YAML's robot.arm_joints. Loaded once at module import so
# the JS viewer's ordering is stable for a given GUI session.
def _load_arm_joints() -> list[str]:
    try:
        from beambot.config_loader import arm_joint_names
        return arm_joint_names()
    except Exception:
        return [
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
        ]


_ARM_JOINTS = _load_arm_joints()


# Always required for any UR-based viewer URDF: the bare arm description
# (joint hierarchy) and the beamline's robot description (mounting + meshes).
def _base_packages() -> list[str]:
    try:
        from beambot.config_loader import description_package
        return ["ur_description", description_package()]
    except Exception:
        return ["ur_description"]


_BASE_PACKAGES = _base_packages()

# All description packages we know how to locate. The URDF parser below picks
# the subset actually referenced by the active beamline's URDFs; entries here
# that go unreferenced are simply skipped.
_KNOWN_DESCRIPTION_PACKAGES = (
    "ur_description",
    *_BASE_PACKAGES[1:],
    "zivid_description",
    "epick_description",
    "robotiq_hande_description",
    "pipette_description",
)

_PACKAGE_REF_RE = re.compile(r"package://([^/\"'\s]+)/")


def _packages_referenced_by_urdf(urdf_path: Path) -> set[str]:
    """Return all `package://<name>/` refs found in the URDF text."""
    try:
        text = urdf_path.read_text()
    except OSError:
        return set()
    return set(_PACKAGE_REF_RE.findall(text))


def _find_workspace_roots():
    """Find colcon workspace root(s). Returns list — may include worktree + main repo."""
    roots = []
    path = Path(__file__).resolve()
    for parent in path.parents:
        if (parent / "src").is_dir() and (parent / "install").is_dir():
            roots.append(parent)
            break
        if (parent / "src" / "beambot_gui").is_dir():
            roots.append(parent)
            # Also check if we're in a worktree — the main repo is above .claude/worktrees/
            for p in parent.parents:
                if (p / "install").is_dir() and (p / "src").is_dir():
                    roots.append(p)
                    break
            break
    if not roots:
        roots.append(path.parents[3])
    return roots


def _resolve_packages(required: set[str] | None = None):
    """Build a dict mapping ROS package names to their share directories on disk.

    `required` is the set of description packages actually referenced by the
    active beamline's URDFs (parsed from `package://...` lines). Defaults to
    the full known list when None is passed (e.g. test paths) so behavior
    matches the pre-config era.
    """
    if required is None:
        required = set(_KNOWN_DESCRIPTION_PACKAGES)
    required = set(required) | set(_BASE_PACKAGES)

    packages = {}

    try:
        from ament_index_python.packages import get_package_share_directory
        for pkg in required:
            try:
                packages[pkg] = get_package_share_directory(pkg)
            except Exception:
                pass
    except ImportError:
        pass

    workspace_roots = _find_workspace_roots()

    # Fill in anything ament_index didn't find
    desc_pkg = _BASE_PACKAGES[1] if len(_BASE_PACKAGES) > 1 else None
    candidates_per_pkg = {
        "ur_description": [Path("/opt/ros/jazzy/share/ur_description")],
        "zivid_description": [Path("/opt/ros/jazzy/share/zivid_description")],
        "epick_description": [],
        "robotiq_hande_description": [],
        "pipette_description": [],
    }
    if desc_pkg:
        candidates_per_pkg[desc_pkg] = []

    # Add workspace-relative paths for each root
    for ws in workspace_roots:
        if desc_pkg:
            candidates_per_pkg[desc_pkg].extend([
                ws / "install" / desc_pkg / "share" / desc_pkg,
                ws / "src" / "custom-ur-descriptions" / desc_pkg,
            ])
        candidates_per_pkg["zivid_description"].insert(0,
            ws / "install" / "zivid_description" / "share" / "zivid_description")
        # Source-tree fallback (thin clients don't build zivid_description; its
        # meshes come from the vcs-imported subtree). Mirrors the epick pattern.
        candidates_per_pkg["zivid_description"].append(
            ws / "src" / "vision" / "zivid-ros" / "zivid_description")
        candidates_per_pkg["epick_description"].extend([
            ws / "install" / "epick_description" / "share" / "epick_description",
            ws / "src" / "end_effectors" / "ros2_epick_gripper" / "epick_description",
        ])
        candidates_per_pkg["robotiq_hande_description"].extend([
            ws / "install" / "robotiq_hande_description" / "share" / "robotiq_hande_description",
            ws / "src" / "end_effectors" / "robotiq_hande_description",
        ])
        candidates_per_pkg["pipette_description"].extend([
            ws / "install" / "pipette_description" / "share" / "pipette_description",
            ws / "src" / "end_effectors" / "pipettor" / "pipette_description",
        ])

    for pkg in required:
        if pkg in packages:
            continue
        for candidate in candidates_per_pkg.get(pkg, []):
            if candidate.is_dir():
                packages[pkg] = str(candidate)
                break

    return packages


def _resolve_scene_yaml():
    """Locate the beamline's scene file via $BEAMBOT_BEAMLINE_CONFIG.

    Reads scene_file: from the active beamline config. Returns None if the
    env var is unset (same soft-fail pattern as the rest of the GUI — the
    operator can still inspect a JSON file with no scene loaded).
    """
    try:
        from beambot.config_loader import load_beamline_config, resolve_beamline_path
        config, config_path = load_beamline_config()
        scene_rel = config.get("scene_file", "")
        if not scene_rel:
            return None
        candidate = Path(resolve_beamline_path(scene_rel, config_path))
        return candidate if candidate.is_file() else None
    except Exception:
        return None


class _MeshRequestHandler(SimpleHTTPRequestHandler):
    """Serves mesh files and static resources for the 3D viewer."""

    def __init__(self, *args, package_map, resources_dir, urdf_dir, **kwargs):
        self.package_map = package_map
        self.resources_dir = resources_dir
        self.urdf_dir = urdf_dir
        super().__init__(*args, **kwargs)

    def translate_path(self, path):
        path = unquote(urlsplit(path).path)

        # Static resources (viewer.html, JS files)
        if path.startswith("/__static__/"):
            rel = path[len("/__static__/"):]
            return self._resolve_under(self.resources_dir, rel)

        # URDF files
        if path.startswith("/__urdf__/"):
            if self.urdf_dir is None:
                return str(Path(self.resources_dir) / ".beambot-path-denied")
            rel = path[len("/__urdf__/"):]
            return self._resolve_under(self.urdf_dir, rel)

        # Package mesh files: /package_name/rest/of/path
        parts = path.strip("/").split("/", 1)
        if len(parts) >= 1 and parts[0] in self.package_map:
            pkg_dir = self.package_map[parts[0]]
            rel = parts[1] if len(parts) > 1 else ""
            return self._resolve_under(pkg_dir, rel)

        return self._resolve_under(self.resources_dir, path.lstrip("/"))

    @staticmethod
    def _resolve_under(root, relative):
        root = Path(root).resolve()
        candidate = (root / relative).resolve()
        if not candidate.is_relative_to(root):
            return str(root / ".beambot-path-denied")
        return str(candidate)

    def end_headers(self):
        self.send_header("Cache-Control", "public, max-age=3600")
        super().end_headers()

    def log_message(self, format, *args):
        pass  # Suppress HTTP logs


class _MeshServer:
    """Lightweight HTTP server for serving URDF meshes to the WebEngine."""

    def __init__(self, package_map, resources_dir, urdf_dir):
        handler = partial(
            _MeshRequestHandler,
            package_map=package_map,
            resources_dir=resources_dir,
            urdf_dir=urdf_dir,
        )
        self.server = HTTPServer(("127.0.0.1", 0), handler)
        self.port = self.server.server_address[1]
        self._thread = threading.Thread(target=self.server.serve_forever, daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self.server.shutdown()
        self.server.server_close()
        self._thread.join(timeout=1)


class VisualizationPanel(QWidget):
    """3D robot visualization tab using Three.js + urdf-loaders in QWebEngineView."""

    def __init__(self, ros2_bridge=None, parent=None):
        super().__init__(parent)
        self.ros2 = ros2_bridge
        self._last_update = 0.0
        self._current_gripper = "epick"
        self._page_ready = False

        self._preview_hold = False
        self._last_live_pose = None

        # Dry-run preview playback state
        self._preview_waypoints = []  # list of (names, positions, t_seconds)
        self._preview_arm_track = []  # decimated arm-joint waypoints in degrees
        self._preview_start_time = 0.0
        self._preview_duration = 0.0
        self._preview_timer = QTimer(self)
        self._preview_timer.setInterval(33)  # ~30 Hz
        self._preview_timer.timeout.connect(self._tick_preview)

        self._resources_dir = Path(__file__).parent / "resources"
        try:
            from beambot.config_loader import description_package
            desc_pkg = description_package()
        except Exception:
            desc_pkg = None
        self._urdf_dir = None
        if desc_pkg:
            for ws in _find_workspace_roots():
                candidate = ws / "src" / "custom-ur-descriptions" / desc_pkg / "urdf"
                if candidate.is_dir():
                    self._urdf_dir = candidate
                    break
            if self._urdf_dir is None:
                self._urdf_dir = Path(__file__).resolve().parents[2] / "custom-ur-descriptions" / desc_pkg / "urdf"

        # Limit description-package lookup to the packages actually referenced
        # by the URDFs the active beamline's grippers declare. A beamline that
        # doesn't use ePick won't trigger spurious "epick_description not
        # found" log lines anymore.
        self._package_map = _resolve_packages(self._referenced_packages())
        self._mesh_server = None
        self._setup_ui()
        self._start_server()

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        # Toolbar
        toolbar = QHBoxLayout()
        toolbar.setContentsMargins(4, 4, 4, 4)

        btn_reset = QPushButton("Reset View")
        btn_reset.setFixedHeight(24)
        btn_reset.clicked.connect(self._reset_view)
        toolbar.addWidget(btn_reset)

        btn_home = QPushButton("Home Pose")
        btn_home.setFixedHeight(24)
        btn_home.clicked.connect(self._home_pose)
        toolbar.addWidget(btn_home)

        btn_demo = QPushButton("Demo")
        btn_demo.setFixedHeight(24)
        btn_demo.setCheckable(True)
        btn_demo.toggled.connect(self._toggle_demo)
        toolbar.addWidget(btn_demo)

        self._btn_stop_preview = QPushButton("Stop Preview")
        self._btn_stop_preview.setFixedHeight(24)
        self._btn_stop_preview.clicked.connect(self.stop_preview)
        self._btn_stop_preview.setVisible(False)
        toolbar.addWidget(self._btn_stop_preview)

        btn_reload_scene = QPushButton("Reload Scene")
        btn_reload_scene.setFixedHeight(24)
        btn_reload_scene.setToolTip(
            "Reload static planning scene from beamline_scene.yaml.\n"
            "Preview only — MoveIt's scene may differ until restart."
        )
        btn_reload_scene.clicked.connect(self._reload_scene)
        toolbar.addWidget(btn_reload_scene)

        toolbar.addStretch()
        self._preview_label = QLabel("")
        self._preview_label.setStyleSheet(
            "color: #ffaa00; font-size: 11px; font-weight: bold;"
        )
        toolbar.addWidget(self._preview_label)
        self._status = QLabel("Initializing...")
        self._status.setStyleSheet("color: #888; font-size: 11px;")
        toolbar.addWidget(self._status)

        layout.addLayout(toolbar)

        if not WEBENGINE_AVAILABLE:
            fallback = QLabel("Install python3-pyqt6.qtwebengine for 3D visualization")
            fallback.setAlignment(Qt.AlignmentFlag.AlignCenter)
            fallback.setStyleSheet("color: #888; font-size: 13px;")
            layout.addWidget(fallback)
            return

        self.web_view = QWebEngineView()
        self.web_view.setStyleSheet("background: #1e1e1e;")
        layout.addWidget(self.web_view, stretch=1)

    def _referenced_packages(self) -> set[str]:
        """Union of `package://` refs across every URDF this beamline can load.

        Walks the URDF filenames declared in $BEAMBOT_BEAMLINE_CONFIG's
        grippers.<name>.urdf_file fields, parses each one, and returns the
        set of referenced description packages. Falls back to the full
        known list on any read failure (so a misconfigured GUI still tries
        to load every package, the original behavior).
        """
        try:
            from beambot.config_loader import load_beamline_config
            config, _ = load_beamline_config()
        except Exception:
            return set(_KNOWN_DESCRIPTION_PACKAGES)

        urdf_files: set[str] = set()
        for gconf in config.get("grippers", {}).values():
            urdf_name = gconf.get("urdf_file")
            if urdf_name:
                urdf_files.add(urdf_name)

        if not urdf_files or self._urdf_dir is None:
            return set(_KNOWN_DESCRIPTION_PACKAGES)

        referenced: set[str] = set()
        for urdf_name in urdf_files:
            referenced |= _packages_referenced_by_urdf(self._urdf_dir / urdf_name)
        return referenced

    def _start_server(self):
        if not WEBENGINE_AVAILABLE:
            return

        self._mesh_server = _MeshServer(
            self._package_map, self._resources_dir, self._urdf_dir
        )
        self._mesh_server.start()
        port = self._mesh_server.port
        self._status.setText(f"Mesh server :{port}")

        url = f"http://127.0.0.1:{port}/__static__/viewer.html"
        self.web_view.loadFinished.connect(self._on_page_loaded)
        self.web_view.load(QUrl(url))

    def _on_page_loaded(self, ok):
        if not ok:
            self._status.setText("Failed to load viewer")
            return

        self._status.setText("Waiting for JS...")
        self._ready_poll = QTimer(self)
        self._ready_poll.timeout.connect(self._check_ready)
        self._ready_poll.start(100)

    def _check_ready(self):
        self.web_view.page().runJavaScript(
            "window._vizReady === true", self._on_ready_result
        )

    def _on_ready_result(self, ready):
        if ready:
            self._ready_poll.stop()
            self._page_ready = True
            self._status.setText("Ready")
            self._load_urdf(self._current_gripper)
            self._reload_scene()

    def _load_urdf(self, gripper):
        if not self._page_ready:
            return
        from beambot.config_loader import gripper_urdf_file
        urdf_file = gripper_urdf_file(gripper, default="ur_standalone.urdf")
        port = self._mesh_server.port
        urdf_url = f"http://127.0.0.1:{port}/__urdf__/{urdf_file}"
        js = f"window.loadRobot('{urdf_url}', {port})"
        self.web_view.page().runJavaScript(js)

    def set_gripper(self, gripper_name):
        """Called when the gripper combo changes."""
        self._current_gripper = gripper_name.lower()
        self._load_urdf(self._current_gripper)

    def _on_joint_state(self, pose_deg):
        """Receive joint angles in degrees and forward to Three.js (throttled)."""
        if not self._page_ready:
            return
        self._last_live_pose = pose_deg
        if self._preview_hold:
            return
        now = time.monotonic()
        if now - self._last_update < 0.033:
            return
        self._last_update = now
        self.web_view.page().runJavaScript(f"window.updateJoints({pose_deg})")

    def preview_pose(self, pose_deg):
        """Show a goal pose in the 3D viewer, suppressing live joint updates."""
        if not self._page_ready:
            return
        if not (isinstance(pose_deg, (list, tuple)) and len(pose_deg) == 6):
            return
        self._preview_hold = True
        self.web_view.page().runJavaScript(
            f"window.updateJoints({list(pose_deg)})"
        )

    def end_preview(self):
        """Release preview hold and snap back to the live robot pose."""
        self._preview_hold = False
        if self._last_live_pose and self._page_ready:
            self.web_view.page().runJavaScript(
                f"window.updateJoints({self._last_live_pose})"
            )

    def _reset_view(self):
        if self._page_ready:
            self.web_view.page().runJavaScript("window.resetView()")

    def _home_pose(self):
        if self._page_ready:
            self.web_view.page().runJavaScript("window.setHomePose()")

    def _toggle_demo(self, checked):
        if self._page_ready:
            self.web_view.page().runJavaScript(
                f"window.demoMode({'true' if checked else 'false'})"
            )

    # --- Dry-run trajectory preview ---

    def play_trajectory(self, _unused_names, waypoints):
        """Animate a planned trajectory in the 3D viewer.

        Args:
            _unused_names: Ignored. Per-waypoint joint names live in waypoints.
            waypoints: List of (joint_names, positions, t_seconds) where the
                t values are absolute (running total across MTC sub-trajectories).
        """
        if not self._page_ready or not waypoints:
            return

        # Project each waypoint onto the 6 arm joints. For waypoints that
        # don't include all 6 (e.g. gripper-only end-effector stages), reuse
        # the last known arm state — which is what really happens during a
        # gripper-only step: the arm holds position.
        last_arm_deg = [0.0] * 6
        # Seed from the first waypoint that has all arm joints, if any.
        for names, positions, _ in waypoints:
            if all(j in names for j in _ARM_JOINTS):
                last_arm_deg = [
                    round(math.degrees(positions[names.index(j)]), 3)
                    for j in _ARM_JOINTS
                ]
                break

        track = []
        for names, positions, t in waypoints:
            if all(j in names for j in _ARM_JOINTS):
                last_arm_deg = [
                    round(math.degrees(positions[names.index(j)]), 3)
                    for j in _ARM_JOINTS
                ]
            track.append((list(last_arm_deg), float(t)))

        self._preview_waypoints = waypoints
        self._preview_arm_track = track
        self._preview_duration = track[-1][1] if track else 0.0
        self._preview_start_time = time.monotonic()

        self._preview_label.setText(f"PREVIEW ({self._preview_duration:.1f}s)")
        self._btn_stop_preview.setVisible(True)
        self._preview_timer.start()

    def stop_preview(self):
        """Halt an in-progress preview animation."""
        self._preview_timer.stop()
        self._preview_label.setText("")
        self._btn_stop_preview.setVisible(False)
        self._preview_arm_track = []

    def _tick_preview(self):
        """QTimer callback: advance preview animation based on wall-clock."""
        if not self._preview_arm_track:
            self._preview_timer.stop()
            return

        elapsed = time.monotonic() - self._preview_start_time

        # Find the bracketing waypoints for the current elapsed time.
        # Preview tracks are short (< 1000 points typically), so a linear
        # walk is fine; no need for a binary search.
        track = self._preview_arm_track
        if elapsed >= track[-1][1]:
            # Done — snap to the final pose, then stop.
            self.web_view.page().runJavaScript(
                f"window.updateJoints({track[-1][0]})"
            )
            self._preview_timer.stop()
            self._preview_label.setText("PREVIEW done")
            self._btn_stop_preview.setVisible(False)
            QTimer.singleShot(2000, lambda: self._preview_label.setText(""))
            return

        # Find segment [i, i+1] containing elapsed
        for i in range(len(track) - 1):
            t0 = track[i][1]
            t1 = track[i + 1][1]
            if t0 <= elapsed <= t1:
                if t1 > t0:
                    alpha = (elapsed - t0) / (t1 - t0)
                else:
                    alpha = 0.0
                p0 = track[i][0]
                p1 = track[i + 1][0]
                interp = [round(p0[k] + alpha * (p1[k] - p0[k]), 3) for k in range(6)]
                self.web_view.page().runJavaScript(f"window.updateJoints({interp})")
                return

    def _reload_scene(self):
        """Read beamline_scene.yaml and push obstacles to the JS viewer.
        No-op if the page isn't ready. All errors surface via the status
        label, never raised — the panel must remain functional.
        """
        if not self._page_ready:
            return
        yaml_path = _resolve_scene_yaml()
        if yaml_path is None:
            self._status.setText("Scene YAML not found")
            return
        try:
            with open(yaml_path, "r") as f:
                cfg = yaml.safe_load(f) or {}
        except Exception as e:
            self._status.setText(f"Scene YAML parse error: {e}")
            return

        payload = []
        skipped = 0
        for obs in cfg.get("obstacles") or []:
            try:
                obs_type = str(obs["type"])
                pose = obs.get("pose") or {}
                pos = [float(pose.get(k, 0.0)) for k in ("x", "y", "z")]
                rot = [float(pose.get(k, 0.0)) for k in ("roll", "pitch", "yaw")]
                if obs_type == "box":
                    size = obs["size"]
                    dims = {"size": [float(size[0]), float(size[1]), float(size[2])]}
                elif obs_type == "cylinder":
                    dims = {
                        "height": float(obs["height"]),
                        "radius": float(obs["radius"]),
                    }
                elif obs_type == "sphere":
                    dims = {"radius": float(obs["radius"])}
                else:
                    skipped += 1
                    continue
                payload.append(
                    {
                        "name": str(obs["name"]),
                        "type": obs_type,
                        "pos": pos,
                        "rot": rot,
                        "dims": dims,
                    }
                )
            except (KeyError, TypeError, ValueError):
                skipped += 1
                continue

        self.web_view.page().runJavaScript(
            f"window.setSceneObstacles({json.dumps(payload)})"
        )
        msg = f"Scene: {len(payload)} obstacles"
        if skipped:
            msg += f" ({skipped} skipped)"
        self._status.setText(msg)

    def closeEvent(self, event):
        self._preview_timer.stop()
        if self._mesh_server:
            self._mesh_server.stop()
        super().closeEvent(event)
