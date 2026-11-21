# End Effectors

This directory mixes project-owned packages with separately imported drivers and
models for grippers, vacuum systems and the pipettor.

## Packages already tracked here

| Package | Role |
|---|---|
| `epick_config` | Local ePick overlay and suction-cup profiles. |
| `onrobot_2fg7_description` | Local 2FG7 geometry. |
| `onrobot_2fg7_driver` | Local ROS/Modbus driver, launch and hardware exercise scripts. |

These packages arrive with the EROBS checkout; they are not downloaded by the
import command below.

## Getting the Drivers

The following additional dependencies live in separate repositories. From the
EROBS repository root, the manifest imports them into named subdirectories:

```bash
vcs import src/end_effectors < src/end_effectors/end_effectors.repos
```

This pulls in:

- `serial` - ROS2 serial communication
- `robotiq_hande_driver` - Robotiq HandE gripper driver
- `robotiq_hande_description` - Robotiq HandE URDF models
- `pipettor` - Pipettor model, driver and action interface

Imported directories are gitignored and versioned separately. Re-importing can
change existing checkouts.

### Install dependencies and build

From the EROBS repository root, with ROS 2 Jazzy sourced:

```bash
# Install dependencies
rosdep install --from-paths src --ignore-src -y

# Build workspace
colcon build
```

## ePick Configuration

`epick_config` layers our setup on top of `epick_description` from `ros2_epick_gripper`:

- `urdf/epick_overlay.xacro`: mounting pose, serial port (`/tmp/ttyUR`) and mock/real hardware switch
- `config/suction_cups.yaml`: suction-cup profiles

### Suction Cup Profiles

The ePick supports swappable suction cups with different dimensions. `config/suction_cups.yaml` is the **single source of truth** for cup dimensions: `ur_with_zivid_epick.xacro` loads it with `xacro.load_yaml()`, so dimensions can't drift between config and URDF.

Available profiles: `pen_vacuum`, `7mm_dia`, `3mm_dia`, `3mm_dia_external_air` (default), `default` (stock cup, no extension).

#### Changing the physical suction cup

Set the profile in the beamline config (the YAML pointed at by `$BEAMBOT_BEAMLINE_CONFIG`, template `src/beambot/config/example_cms_beamline.yaml`) under `grippers.epick`:

```yaml
cup_profile: "7mm_dia"
```

beambot passes it to `robot_bringup.launch.py` as `cup_profile:=...`. The launch file uses it for MoveIt's robot description and exports it as `BEAMBOT_EPICK_CUP_PROFILE` for the UR driver's robot_state_publisher, so both load the same geometry. No rebuild is needed.

To launch directly, export the beamline config first; `robot_bringup.launch.py` reads the gripper settings from it:

```bash
export BEAMBOT_BEAMLINE_CONFIG=<path to beamline yaml>
ros2 launch cms_moveit_config robot_bringup.launch.py gripper:=epick cup_profile:=7mm_dia
```

#### Adding a new cup profile

Add it to `config/suction_cups.yaml`:

```yaml
cups:
  my_new_cup:
    description: "My custom cup"
    extension_length: 0.025   # meters
    extension_radius: 0.005
    suction_cup_height: 0.004
    suction_cup_radius: 0.002
```

Rebuild with `colcon build --packages-select epick_config`, then select it as above.

#### Runtime override (temporary, current session only)

```
set_cup_profile(name="7mm_dia")
```

Sets the orchestrator's `cup_profile` parameter. It takes effect on the next MoveIt restart and lasts until the orchestrator restarts.
