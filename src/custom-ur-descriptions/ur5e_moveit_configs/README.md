# MoveIt Configurations

This directory contains MoveIt configuration packages for UR5e robotic systems with various end-effector configurations. Each package provides complete MoveIt planning and execution capabilities for specific robot setups.

## Available Configurations

### ur_standalone_moveit_config

- **Robot**: UR5e arm with Zivid camera and tool exchanger robotside
- **Chain**: UR5e → Zivid Camera → TE_RobotSide

### ur_zivid_hande_moveit_config

- **Robot**: UR5e arm with Zivid camera, tool block, and Robotiq Hand-E gripper
- **Chain**: UR5e → Zivid Camera → Tool Block → Hand-E Gripper

### ur_zivid_epick_moveit_config

- **Robot**: UR5e arm with Zivid camera, tool block, and ePick vacuum gripper
- **Chain**: UR5e → Zivid Camera → Tool Block → ePick Gripper

## Package Structure

Each MoveIt configuration package follows a standard structure:

```text
ur_<config_name>_moveit_config/
├── CMakeLists.txt                    # Build configuration
├── package.xml                       # Package metadata
├── config/                           # MoveIt configuration files
│   ├── ur.srdf                      # Semantic robot description
│   ├── kinematics.yaml              # Kinematics solver configuration
│   ├── joint_limits.yaml            # Joint velocity/acceleration limits
│   ├── initial_positions.yaml       # Default joint positions
│   ├── ompl_planning.yaml          # OMPL planner configuration
│   ├── moveit_controllers.yaml     # MoveIt controller configuration
│   ├── ur_<gripper>_controllers.yaml # Hardware controller configuration
│   └── ur.urdf.xacro               # Robot description for MoveIt
├── launch/
│   └── robot_bringup.launch.py      # Main launch file
├── rviz/
│   └── view_robot_mtc.rviz         # RViz configuration
└── .setup_assistant                 # MoveIt Setup Assistant metadata
```

## Configuration Files Explained

### Core MoveIt Files

- **ur.srdf**: Semantic Robot Description Format file defining planning groups, disabled collisions, and poses
- **kinematics.yaml**: Inverse kinematics solver configuration (uses KDL plugin)
- **joint_limits.yaml**: Velocity/acceleration scaling and joint-specific limits
- **ompl_planning.yaml**: Open Motion Planning Library planner configurations
- **moveit_controllers.yaml**: Maps MoveIt planning groups to ROS2 controllers

### Hardware Integration

- **ur_&lt;gripper&gt;_controllers.yaml**: ROS2 control configuration for specific grippers
- **ur.urdf.xacro**: Robot description file that MoveIt uses internally

### Launch Configuration

- **robot_bringup.launch.py**:
  - Launches UR robot driver with hardware interface
  - Starts MoveIt move_group node with planning capabilities
  - Spawns gripper controllers
  - Launches RViz for visualization
  - Configures robot state publisher for TF transforms

## Usage

### Launch MoveIt Planning Environment

```bash
# For standalone configuration (no gripper)
ros2 launch ur_standalone_moveit_config robot_bringup.launch.py

# For Hand-E gripper configuration
ros2 launch ur_zivid_hande_moveit_config robot_bringup.launch.py

# For ePick vacuum gripper configuration
ros2 launch ur_zivid_epick_moveit_config robot_bringup.launch.py
```

### Launch Parameters

Each launch file accepts the following parameters:

- `robot_ip`: IP address of the UR robot (default: '192.168.1.10')
- `ur_type`: UR robot model (default: 'ur5e')
- `description_package`: Package for robot config files (default: 'ur_description')
- `description_file`: Custom URDF file path (points to ur5e_robot_description)
- `controllers_file`: Hardware controller configuration file
- `rviz_config`: RViz configuration file to load

Example with custom parameters:

```bash
ros2 launch ur_zivid_hande_moveit_config robot_bringup.launch.py robot_ip:=192.168.1.100 ur_type:=ur5e
```

## Creating New MoveIt Configurations

For detailed instructions on creating new MoveIt configurations, please refer to the official MoveIt documentation:

**[MoveIt Setup Assistant Tutorial](https://moveit.picknik.ai/main/doc/examples/setup_assistant/setup_assistant_tutorial.html)**

## Configuration Consistency

All configurations maintain consistency in:

- **Kinematics**: Same solver settings across all configs
- **Joint Limits**: Identical UR arm limits, gripper-specific limits vary
- **OMPL Planning**: Consistent planner configurations
- **Launch Structure**: Standardized launch file patterns

### Robot Naming Convention

Robot name is set dynamically using the `ur_type` parameter (e.g., ur3e, ur5e, ur10e) to match ur_control's behavior. The launch files pass `name:=LaunchConfiguration("ur_type")` to ensure consistency between the `/robot_description` topic (used by action servers and ur_control) and MoveIt's move_group. This enables using the same launch files for any UR robot type.

## Integration with Robot Description

These MoveIt configurations work with robot descriptions from:

- **ur5e_robot_description**: Custom URDF files for complete robot systems
- **ur_description**: Standard UR robot configuration files
- **robotiq_hande_description**: Hand-E gripper descriptions
- **epick_config**: ePick vacuum gripper descriptions

## TODO

- Have consistent "moveit_home" values across SRDFs
