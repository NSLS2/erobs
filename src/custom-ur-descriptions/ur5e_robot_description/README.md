# ur5e_robot_description

This package contains robot description files for UR5e robotic systems with
various end-effectors and camera configurations:

- Standalone UR5e arm with Zivid camera and tool exchanger
- UR5e with Zivid camera, tool block, and Robotiq Hand-E gripper
- UR5e with Zivid camera, tool block, and ePick vacuum gripper
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

## Package Structure

### URDF Files (`urdf/`)

- **ur_standalone.xacro**: UR5e arm with Zivid camera and tool exchanger
- **ur_with_zivid_hande.xacro**: Complete system with Robotiq Hand-E gripper
- **ur_with_zivid_epick.xacro**: Complete system with ePick vacuum gripper
- **te_robotside.xacro**: Tool exchanger robot-side component (shows correct configuration when no gripper is attached)
- **tool_block.xacro**: Combined tool block that integrates both robot-side and tool-side parts into a single component for simplified gripper attachment
- **zivid_camera_mount.xacro**: Zivid camera mounting system

### Mesh Files (`meshes/`)

- **tool_exchanger/**: Tool exchanger system STL files
- **zivid/**: Zivid camera mesh files and protective housing

### Dependencies

This package requires the following external packages:

- **[Universal_Robots_ROS2_Description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description)**: Official UR robot descriptions
- **robotiq_hande_description**: Robotiq Hand-E gripper descriptions (included in `src/end_effectors/`)

```xml
<xacro:include filename="$(find ur_description)/urdf/ur_macro.xacro"/>
<xacro:include filename="$(find robotiq_hande_description)/urdf/robotiq_hande_gripper.xacro"/>
```

### Mesh Sources

- **Zivid camera**: Official mesh files from [Zivid Downloads](https://www.zivid.com/downloads)
- **Tool exchanger**: Custom STL files for the tool exchanger system
- **Hand-E gripper**: Provided by external `robotiq_hande_description` package

### Hardware Configuration Notes

**End Effector Hardware Settings:**

- `ur_with_zivid_hande.xacro` and `ur_with_zivid_epick.xacro` both have `use_fake_hardware="true"` hardcoded for the end effectors
- This means the grippers operate in simulation mode (not interacting with actual gripper hardware)
- **For actual hardware:** Change `use_fake_hardware="true"` to `use_fake_hardware="false"` in the respective gripper macro calls

### TODO

- For the zivid camera:
  - Use meshes/description/XACRO files from the official `zivid_ros` package instead of local mesh files
  - Remove strain relief part as we aren't using it for the actual robot
- Make `use_fake_hardware` parameter dynamic for end effectors instead of hardcoded values
- Add custom suction cup to the ePick gripper (pen_vacuum)
- Add the robot mount

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
```

### Integration with MoveIt

This package is designed to work with the corresponding MoveIt configuration packages:

- `ur_standalone_moveit_config`
- `ur_zivid_hande_moveit_config`
- `ur_zivid_epick_moveit_config`
