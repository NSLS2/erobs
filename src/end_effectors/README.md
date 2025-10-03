# End Effectors

This directory contains drivers and configuration for robot end effectors like grippers and vacuum systems.

## Getting the Drivers

The actual driver code lives in separate repositories. To download them:

```bash
vcs import src/end_effectors < src/end_effectors/end_effectors.repos
```

This pulls in:
- `serial` - ROS2 serial communication
- `robotiq_hande` - Robotiq HandE gripper driver and URDF models
- `ros2_epick_gripper` - EPick vacuum gripper driver
- `pipettor` - Custom pipettor developed at CMS NSLS-2


**Note:** The EPick driver uses a fork from <https://github.com/bondada-a/ros2_epick_gripper.git> instead of the upstream PickNikRobotics version. This fork includes updated headers and removes the epick_moveit_studio package (requires Moveit Pro).

## EPick Configuration

The `epick_config` package provides an overlay configuration (over ros2_epick_driver) for our specific setup:

- Serial port: `/tmp/ttyUR`
- Custom positioning offsets
- Hardware vs simulation mode

## Future Work

For simpler single-robot setups, we could skip the overlay package and put EPick parameters directly in the robot URDF files?