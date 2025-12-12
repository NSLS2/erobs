# Vision

This directory contains drivers and configuration for vision systems like 3D cameras.

## Getting the Drivers

The actual driver code lives in separate repositories. To download them:

```bash
vcs import src/vision < src/vision/vision.repos
```

This pulls in:
- `zivid-ros` - Zivid 3D camera ROS2 driver and URDF descriptions
- `zed-ros2-wrapper` - Stereolabs ZED camera ROS2 wrapper

## Zivid Camera

The `zivid-ros` package provides:
- ROS2 driver for Zivid 3D cameras
- `zivid_description` - URDF models and meshes for Zivid cameras
- Camera calibration and capture services

**Note:** Zivid SDK must be installed separately. See [Zivid installation guide](https://support.zivid.com/en/latest/getting-started/software-installation.html).

## ZED Camera

The `zed-ros2-wrapper` package provides:
- ROS2 wrapper for Stereolabs ZED stereo cameras
- Depth sensing and spatial mapping capabilities

**Note:** ZED SDK must be installed separately. See [ZED SDK installation](https://www.stereolabs.com/developers/release).

## Dependencies

The robot description packages (e.g., `ur5e_robot_description`) depend on `zivid_description` for camera URDF models. 