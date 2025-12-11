# hello_orchestrator_py


## Overview

```
                    ┌─────────────────┐
                    │  orchestrator   │
                    │     client      │
                    └────────┬────────┘
                             │ JSON task
                             ▼
                    ┌─────────────────┐
                    │  orchestrator   │
                    │     server      │
                    └────────┬────────┘
                             │ 
              ┌──────────────┴──────────────┐
              │                             │
              ▼                             ▼
     ┌─────────────────┐           ┌─────────────────┐
     │  print_server   │           │   move_server   │
     │  (logs message) │           │   (MTC motion)  │
     └─────────────────┘           └────────┬────────┘
                                            │
                                            ▼
                                   ┌─────────────────┐
                                   │     MoveIt      │
                                   │   (execution)   │
                                   └─────────────────┘
```


## Prerequisites

MoveIt must be running with `ExecuteTaskSolutionCapability`. Custom configs in `ur5e_moveit_configs` include this by default:
- `ur_standalone_moveit_config`
- `ur_zivid_epick_config`
- `ur_zivid_hande_config`

The demo uses hardcoded values that must exist in your MoveIt config:
- **Planning group**: `ur_arm` (hardcoded in `base_stages.py` and `demo.launch.py`)
- **Named state**: `moveit_home` (hardcoded in `demo_task.json`)

## Quick Start

### 1. Build

```bash
colcon build --packages-select \
  ur_standalone_moveit_config \
  hello_orchestrator_py \
  hello_orchestrator_py_interfaces

source install/setup.bash
```

### 2. Launch MoveIt (Terminal 1)

```bash
ros2 launch ur_standalone_moveit_config robot_bringup.launch.py use_fake_hardware:=true
```

Wait for: `"You can start planning now!"`

### 3. Launch Demo Servers (Terminal 2)

```bash
ros2 launch hello_orchestrator_py demo.launch.py
```

### 4. Send Test Task (Terminal 3)

```bash
ros2 run hello_orchestrator_py orchestrator_client.py <path_to_task.json>
```

Example:
```bash
ros2 run hello_orchestrator_py orchestrator_client.py \
  src/demos/hello_orchestrator_py/config/demo_task.json
```

## Architecture

### Action Servers

| Server | Purpose |
|--------|---------|
| `print_server` | Logs messages to console |
| `move_server` | Moves robot to SRDF named poses via MTC |
| `orchestrator_server` | Dispatches JSON tasks to specialized servers |

### Task Format

```json
{
  "tasks": [
    {"type": "print", "message": "Hello"},
    {"type": "move", "target_pose": "moveit_home"}
  ]
}
```

### Execution Flow

1. `orchestrator_client` sends JSON to `orchestrator_server`
2. Orchestrator iterates through tasks
3. Each task dispatched to appropriate server (`print_server` or `move_server`)
4. Server executes and returns result
5. Orchestrator reports completion


