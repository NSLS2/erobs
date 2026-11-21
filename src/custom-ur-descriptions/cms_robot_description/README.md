# cms_robot_description

This package contains robot description files for the CMS beamline UR5e with a
Zivid camera, a tool exchanger and swappable end effectors:

- Standalone UR5e arm with Zivid camera and tool-exchanger robot plate
- UR5e with Zivid camera, tool exchanger and Robotiq Hand-E gripper
- UR5e with Zivid camera, tool exchanger and ePick vacuum gripper
- UR5e with Zivid camera, tool exchanger and OnRobot 2FG7 gripper
- UR5e with Zivid camera, tool exchanger and pipettor end-effector
- Mesh files for tool exchanger and camera components

## Available Robot Configurations

Every configuration mounts the Zivid camera on `tool0`. The tool chain is:

| Xacro | Chain after `tool0` |
|---|---|
| `ur_standalone.xacro` | wrist extension → TE robot plate |
| `ur_with_zivid_hande.xacro` | wrist extension → TE robot plate → TE tool plate → Hand-E |
| `ur_with_zivid_epick.xacro` | wrist extension → TE robot plate → TE tool plate → coupling → ePick |
| `ur_with_zivid_2fg7.xacro` | wrist extension → TE robot plate → TE tool plate → 2FG7 |
| `ur_with_zivid_pipettor.xacro` | wrist extension → TE robot plate → TE tool plate → pipettor |

## Package Structure

### URDF Files (`urdf/`)

- **ur_*.xacro**: the five configurations above
- **zivid_camera_mount.xacro**: Zivid on-arm mount and camera, placed by the hand-eye calibration
- **wrist_extension.xacro**: EX-50-35 wrist extension
- **te_robotside.xacro** / **te_toolside.xacro**: WM1 tool-exchanger robot plate and tool plate
- **te_coupling.xacro**: W-010-B coupling between the tool plate and the ePick
- **ur_*.urdf**: pre-baked copies for the beambot_gui 3D viewer (see [Regenerating Baked URDFs](#regenerating-baked-urdfs))

### Config Files (`config/`)

- **ur5e_calibration.yaml**: UR kinematics calibration of the CMS robot
- **hand_eye_calibration.yaml**: Zivid hand-eye calibration result; the camera pose in `zivid_camera_mount.xacro` comes from it
- **`<gripper>/initial_positions.yaml`**: starting joint angles for mock hardware, one per configuration

### Mesh Files (`meshes/`)

- **tool_exchanger/**: wrist extension, tool-exchanger plates and ePick coupling STL files
- **zivid/**: Modified Zivid arm mount mesh file - [mount](https://shop.zivid.com/collections/mounts/products/on-arm-mount-robot-zivid-3d) - the camera bracket is mounted backward for more space in front of the camera to allow for tool exchange.

### Dependencies

This package requires the following packages:

- **ur_description**, **ur_robot_driver**, **ur_client_library**: UR arm macro and ros2_control description ([Universal_Robots_ROS2_Description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description))
- **zivid_description**: Zivid camera meshes (imported into `src/vision/`)
- **epick_config**: ePick overlay and suction-cup profiles (`src/end_effectors/`)
- **robotiq_hande_description**, **robotiq_hande_driver**: Hand-E description and hardware plugin (imported into `src/end_effectors/`)
- **onrobot_2fg7_description**: 2FG7 description (`src/end_effectors/`)
- **pipette_description**: Pipettor description (imported into `src/end_effectors/`)

### Mesh Sources

- **Zivid camera**: Meshes from `zivid_description` ([zivid-ros](https://github.com/zivid/zivid-ros))
- **Zivid arm mount**: Custom mesh file for mounting Zivid camera to robot tool0 frame
- **Tool exchanger**: Custom STL files for the wrist extension, tool-exchanger plates and coupling
- **Grippers**: Provided by their own description packages

### Camera and UR Calibration

**Camera Calibration:**

- Zivid Hand-eye calibration based on - [Zivid Hand-eye calibration](https://support.zivid.com/en/latest/academy/applications/hand-eye.html)
- The result is kept in `config/hand_eye_calibration.yaml`; the camera pose the xacros use is in `zivid_camera_mount.xacro`.

**UR Calibration:**

Each UR robot has unique kinematics due to manufacturing tolerances. The included `config/ur5e_calibration.yaml` is specific to the CMS beamline robot and **must be replaced** with your own robot's calibration.

To generate your calibration file:
1. Follow the [ur_calibration guide](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_robot_driver/ur_calibration/doc/usage.html)
2. Replace `config/ur5e_calibration.yaml` with your robot's output

**Loading mechanism:** The calibration is loaded via the `kinematics_params_file` argument passed from each MoveIt config's `robot_bringup.launch.py` to `ur_control.launch.py`.

### Hardware Configuration Notes

**Simulation vs Real Hardware:**

All xacro files support the `use_mock_hardware` argument (default: `false`):
- `use_mock_hardware:=false` - connects to real robot and end effector hardware
- `use_mock_hardware:=true` - runs with mock hardware, starting from `config/<gripper>/initial_positions.yaml`
- **URSim:** URSim needs real hardware for the arm but mock hardware for the gripper. No argument splits the two, so change it by hand in `ur_with_zivid_hande.xacro` and `ur_with_zivid_epick.xacro`.

This parameter is passed through to both the UR robot and end effectors (Hand-E, ePick).

## Kinematic Chain Reference

Offsets come from the xacros. "Outward" means away from the flange, along `tool0`'s Z axis.

```
flange / tool0                 (same physical point, robot bolt face)
├── zivid_arm_mount            +5 mm along tool0 Z
├── zivid_optical_frame        hand-eye calibration (zivid_camera_mount.xacro)
└── wrist_extension            +16 mm along tool0 Z
    └── te_robotside           +47 mm outward
        └── te_toolside        +11 mm outward (gripper mounting face)
            ├── robotiq_hande_coupler → ... → robotiq_hande_end   (Hand-E)
            ├── te_coupling → epick_base_link → ... → epick_tip   (ePick)
            ├── 2fg7_base_link → ... → 2fg7_tip                   (2FG7)
            └── pipette_base_link → ... → pipette_tip_link        (pipettor)
```

`ur_standalone.xacro` stops at `te_robotside`. ePick extension and cup lengths come from the selected cup profile (`epick_config/config/suction_cups.yaml`).

### Key Frames for Software

| Frame | Used By | Purpose |
|-------|---------|---------|
| `flange` | MoveIt arm group tip_link; IK frame with no gripper | ROS-Industrial convention, X+ outward |
| `tool0` | Zivid hand-eye calibration | UR convention (X+ left, Y+ up, Z+ forward) |
| `robotiq_hande_end` | IK frame (Hand-E) | Hand-E fingertip frame |
| `epick_tip` | IK frame (ePick), vision stages | Suction-cup contact point |
| `epick_tcp` | Available for stroke tuning | Currently same as `epick_tip` |
| `2fg7_tip` | IK frame (2FG7) | 2FG7 fingertip frame |
| `pipette_tip_link` | IK frame (pipettor) | Pipette tip |

The IK frame for each gripper is set by `grippers.<name>.tip_frame` in the beamline config.

### Axis Conventions

Due to rotations in the chain, local axes don't always point where you'd expect:

- **`te_robotside` and `te_toolside`**: -Y points outward
- **Gripper frames** (from `epick_base_link`, `robotiq_hande_coupler`, ...): Z points outward again

### Regenerating Baked URDFs

The `.urdf` files are pre-baked copies for the beambot_gui 3D viewer (`grippers.<name>.urdf_file` in the beamline config). After changing any xacro in the chain, rebuild and regenerate them. Run xacro from the `urdf/` directory so the generated header doesn't record a local path:

```bash
colcon build --packages-select cms_robot_description
source install/setup.bash
cd src/custom-ur-descriptions/cms_robot_description/urdf
for x in ur_standalone ur_with_zivid_hande ur_with_zivid_epick ur_with_zivid_2fg7 ur_with_zivid_pipettor; do
  xacro $x.xacro name:=ur5e ur_type:=ur5e use_mock_hardware:=true \
    script_filename:= input_recipe_filename:= output_recipe_filename:= > $x.urdf
done
```

The ePick file uses the default cup profile unless `BEAMBOT_EPICK_CUP_PROFILE` is set.

## Usage

### Building the Package

```bash
colcon build --packages-select cms_robot_description
source install/setup.bash
```

### Viewing in RViz

`robot_bringup.launch.py` reads the gripper settings from the beamline config, so export it first:

```bash
export BEAMBOT_BEAMLINE_CONFIG=<path to beamline yaml>   # template: src/beambot/config/example_cms_beamline.yaml

# Launch with specific gripper configuration
ros2 launch cms_moveit_config robot_bringup.launch.py gripper:=none          # standalone
ros2 launch cms_moveit_config robot_bringup.launch.py gripper:=epick         # ePick vacuum
ros2 launch cms_moveit_config robot_bringup.launch.py gripper:=hande         # Hand-E
ros2 launch cms_moveit_config robot_bringup.launch.py gripper:=pipettor      # pipettor
ros2 launch cms_moveit_config robot_bringup.launch.py gripper:=2fg7          # OnRobot 2FG7
```

### Integration with MoveIt

This package is designed to work with the unified MoveIt configuration package `cms_moveit_config`, which supports all grippers via the `gripper:=` launch argument.
