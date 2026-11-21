# bluesky_ros — Bluesky ↔ EROBS Integration

Ophyd device wrapper that lets Bluesky control the robot via ROS2 action client.

## Quick Start

```python
 export PYTHONPATH=/root/ws/erobs/src/bluesky_ros:$PYTHONPATH
```


```python
import rclpy
rclpy.init()

from bluesky import RunEngine
import bluesky.plan_stubs as bps

RE = RunEngine({})

from bluesky_ros.beambot_ophyd_device_async import BeambotExecutionDeviceAsync
robot = BeambotExecutionDeviceAsync(name="ur5e_robot")

# Execute a task (blocking — waits for robot to finish)
RE(bps.abs_set(robot, "task_sequences/complete_sequence.json", wait=True))

# Execute a task (non-blocking — returns immediately)
RE(bps.abs_set(robot, "task_sequences/complete_sequence.json", wait=False))

# Cancel a running task
robot.cancel_goal()

# Cleanup
rclpy.shutdown()
```

## Interleaving Robot + EPICS

```python
def sample_plan(robot, motor):
    yield from bps.abs_set(robot, "tasks/pick_sample.json", wait=True)
    yield from bps.mv(motor, measurement_position)
    # ... collect data ...
    yield from bps.abs_set(robot, "tasks/return_sample.json", wait=True)

RE(sample_plan(robot, motor))
```

## Requirements

- ROS2 Humble with `beambot_interfaces` sourced
- `beambot_execution` action server running (orchestrator)
- `pip install bluesky ophyd`
