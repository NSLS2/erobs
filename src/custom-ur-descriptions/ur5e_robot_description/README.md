# ur5e_robot_description

This package contains robot description files for UR5e robotic systems with
various end-effectors and camera configurations:

- Standalone UR5e arm with Zivid camera and tool exchanger
- UR5e with Zivid camera, tool block, and Robotiq Hand-E gripper
- UR5e with Zivid camera, tool block, and ePick vacuum gripper
- UR5e with Zivid camera, tool block, and pipettor end-effector
- Mesh files for tool exchanger and camera components

## Available Robot Configurations

### ur_standalone.xacro

UR5e arm with Zivid camera and tool exchanger robotside.
Chain: UR5e → Zivid Camera → TE_RobotSide

### ur_with_zivid_hande.xacro

UR5e arm with Zivid camera, tool block, and Robotiq Hand-E gripper.
Chain: UR5e → Zivid Camera → Tool Block → Hand-E Gripper

### ur_with_zivid_epick.xacro

UR5e arm with Zivid camera, tool block, and ePick vacuum gripper.
Chain: UR5e → Zivid Camera → Tool Block → ePick Gripper

### ur_with_zivid_pipettor.xacro

UR5e arm with Zivid camera, tool block, and pipettor end-effector.
Chain: UR5e → Zivid Camera → Tool Block → Pipettor

## Package Structure

### URDF Files (`urdf/`)

- **ur_standalone.xacro**: UR5e arm with Zivid camera and tool exchanger
- **ur_with_zivid_hande.xacro**: Complete system with Robotiq Hand-E gripper
- **ur_with_zivid_epick.xacro**: Complete system with ePick vacuum gripper
- **ur_with_zivid_pipettor.xacro**: Complete system with pipettor end-effector
- **te_robotside.xacro**: Tool exchanger robot-side component (shows correct configuration when no gripper is attached)
- **tool_block.xacro**: Combined tool block that integrates both robot-side and tool-side parts into a single component for simplified gripper attachment
- **zivid_camera_mount.xacro**: Zivid camera mounting system

### Mesh Files (`meshes/`)

- **tool_exchanger/**: Tool exchanger system STL files
- **zivid/**: Modified Zivid arm mount mesh file - [mount](https://shop.zivid.com/collections/mounts/products/on-arm-mount-robot-zivid-3d) - the camera bracket is mounted backward for more space in front of the camera to allow for tool exchange.

### Dependencies

This package requires the following external packages:

- **[Universal_Robots_ROS2_Description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description)**: Official UR robot descriptions
- **zivid_description**: Zivid camera URDF and meshes (included in `src/vision/`)
- **robotiq_hande_description**: Robotiq Hand-E gripper descriptions (included in `src/end_effectors/`)
- **epick_config**: ePick gripper descriptions (included in `src/end_effectors/`)
- **pipette_description**: Pipettor descriptions (included in `src/end_effectors/`)

```xml
<xacro:include filename="$(find ur_description)/urdf/ur_macro.xacro"/>
<xacro:include filename="$(find zivid_description)/urdf/macros/zivid_camera.xacro"/>
<xacro:include filename="$(find robotiq_hande_description)/urdf/robotiq_hande_gripper.xacro"/>
```

### Mesh Sources

- **Zivid camera**: Official URDF from [zivid-ros](https://github.com/zivid/zivid-ros) package
- **Zivid arm mount**: Custom mesh file for mounting Zivid camera to robot tool0 frame
- **Tool exchanger**: Custom STL files for the tool exchanger system
- **Hand-E gripper**: Provided by external `robotiq_hande_description` package

### Camera and UR Calibration

**Camera Calibration:**

- Zivid Hand-eye calibration based on - [Zivid Hand-eye calibration](https://support.zivid.com/en/latest/academy/applications/hand-eye.html)


**UR Calibration:**

Each UR robot has unique kinematics due to manufacturing tolerances. The included `config/ur5e_calibration.yaml` is specific to the CMS beamline robot and **must be replaced** with your own robot's calibration.

To generate your calibration file:
1. Follow the [ur_calibration guide](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_robot_driver/ur_calibration/doc/usage.html)
2. Replace `config/ur5e_calibration.yaml` with your robot's output

**Loading mechanism:** The calibration is loaded via the `kinematics_params_file` argument passed from each MoveIt config's `robot_bringup.launch.py` to `ur_control.launch.py`. 

### Hardware Configuration Notes

**Simulation vs Real Hardware:**

All xacro files support the `use_fake_hardware` argument (default: `false`):
- `use_fake_hardware:=false` - connects to real robot and end effector hardware
- `use_fake_hardware:=true` - runs in simulation mode without hardware
- **URSim:** When using URSim, this must be manually changed in the xacro files - set `use_fake_hardware:=false` for UR robot and `use_fake_hardware:=true` for the grippers in `ur_with_zivid_hande.xacro` and `ur_with_zivid_epick.xacro`

This parameter is passed through to both the UR robot and end effectors (Hand-E, ePick).

### TODO
- Add custom suction cup to the ePick gripper (pen_vacuum)

## Usage

### Building the Package

```bash
colcon build --packages-select ur5e_robot_description
source install/setup.bash
```

### Viewing in RViz (after creating moveit_config)

```bash
# Launch with specific configuration
ros2 launch ur_standalone_moveit_config robot_bringup.launch.py
ros2 launch ur_zivid_hande_moveit_config robot_bringup.launch.py
ros2 launch ur_zivid_epick_moveit_config robot_bringup.launch.py
ros2 launch ur_zivid_pipettor_moveit_config robot_bringup.launch.py
```

### Integration with MoveIt

This package is designed to work with the corresponding MoveIt configuration packages:

- `ur_standalone_moveit_config`
- `ur_zivid_hande_moveit_config`
- `ur_zivid_epick_moveit_config`
- `ur_zivid_pipettor_moveit_config`
