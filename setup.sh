#!/bin/bash
set -e

vcs import < src/ros2.repos src
vcs import < src/end_effectors/end_effectors.repos src/end_effectors
sudo apt-get update
rosdep update
rosdep install --from-paths src --ignore-src -y --skip-keys moveit_studio_behavior_interface --skip-keys epick_moveit_studio