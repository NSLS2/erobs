#!/bin/bash
# Test the ROS2 driver node with GripperCommand action goals.
# Starts socat, launches the driver, sends close/open commands.
#
# Prerequisites:
#   - UR teach pendant: Tool I/O = User, RS485 = 1Mbps/Even parity, Voltage = 24V
#   - Workspace built: colcon build --packages-select onrobot_2fg7_driver
#
# Usage:
#   bash test_ros2_driver.sh [ROBOT_IP]

ROBOT_IP=${1:-192.168.1.101}
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/../../../.." && pwd)"

set -e

# Clean up
pkill -f "socat.*ttyUR" 2>/dev/null || true
pkill -f "onrobot_2fg7_driver" 2>/dev/null || true
sleep 1
rm -f /tmp/ttyUR

# Start socat
socat pty,link=/tmp/ttyUR,raw,ignoreeof tcp:${ROBOT_IP}:54321 &
SOCAT_PID=$!
trap 'kill $DRIVER_PID $SOCAT_PID 2>/dev/null || true' EXIT
sleep 2
echo "Socat running (PID $SOCAT_PID)"

# Source workspace
source "${WS_DIR}/install/setup.bash"

# Start driver in background
ros2 run onrobot_2fg7_driver onrobot_2fg7_driver_node \
  --ros-args \
  -p serial_port:=/tmp/ttyUR \
  -p slave_id:=65 \
  -p baudrate:=1000000 &
DRIVER_PID=$!
sleep 5
echo "Driver running (PID $DRIVER_PID)"

# Test close
echo ""
echo "=== CLOSE (position=0.0) ==="
ros2 action send_goal /gripper_action_controller/gripper_cmd \
  control_msgs/action/GripperCommand \
  "{command: {position: 0.0, max_effort: 40.0}}"

sleep 2

# Test open
echo ""
echo "=== OPEN (position=0.03) ==="
ros2 action send_goal /gripper_action_controller/gripper_cmd \
  control_msgs/action/GripperCommand \
  "{command: {position: 0.03, max_effort: 40.0}}"

sleep 2

echo ""
echo "Done."
