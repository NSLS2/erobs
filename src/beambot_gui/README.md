# MTC GUI

PyQt6-based GUI for creating and executing robot tasks via the beambot orchestrator.

## Modules

| File | Purpose |
|------|---------|
| `main.py` | Entry point, dark theme setup |
| `main_window.py` | QMainWindow — task tree, toolbar, splitter layout |
| `ros2_bridge.py` | ROS2 node with pyqtSignal bridge for thread-safe updates |
| `task_forms.py` | 8 task type dialogs (moveto, pick/place sample, tool exchange, etc.) |
| `camera_panel.py` | Camera display with ArUco marker and contour overlays |
| `chat_panel.py` | LLM chat panel with message bubbles |
| `agent_bridge.py` | Async bridge connecting RobotAgent to Qt event loop |
| `pose_dialogs.py` | Pose editor, manager, and save dialogs |

## Usage

```bash
# Via launch file
ros2 launch beambot_gui beambot_gui_client.launch.py

# Direct
ros2 run beambot_gui beambot_gui_client
```

## Dependencies

- `python3-pyqt6`, `python3-pyqt6.qtwebengine`
- `rclpy`
- `beambot` (action servers, agent)

## Building

```bash
colcon build --packages-select beambot_gui
```
