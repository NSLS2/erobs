#!/bin/bash
set -e
source /opt/ros/${ROS_DISTRO}/setup.bash
ament_${LINTER} src/
