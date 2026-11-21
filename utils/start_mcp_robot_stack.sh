#!/bin/bash
# Start beambot (including rosbridge for MCP).
#
# Requires BEAMBOT_BEAMLINE_CONFIG to point at a beamline YAML.
# Usage: ./utils/start_mcp_robot_stack.sh [beambot launch args]
#   e.g. ./utils/start_mcp_robot_stack.sh use_mock_hardware:=true enable_vision:=false
#
# Launch args (default):
#   use_mock_hardware:=false       mock robot hardware (camera/pipettor unaffected)
#   enable_vision:=false           vision/sample servers + Zivid camera
#   enable_pipettor:=false         pipettor server
#   enable_joystick:=false         gamepad control
#   enable_batching:=true          MTC stage batching in the orchestrator
#   enable_rosbridge:=true         rosbridge on port 9090
#   orchestrator_log_level:=info   debug|info|warn|error|fatal

set -e

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# install/setup.bash also sources the ROS underlay it was built against.
if [[ ! -f "$WORKSPACE_DIR/install/setup.bash" ]]; then
    echo "ERROR: workspace not built. Run: cd \"$WORKSPACE_DIR\" && colcon build" >&2
    exit 1
fi
source "$WORKSPACE_DIR/install/setup.bash"

# Required: no default config, so the wrong beamline is never loaded silently.
if [[ -z "$BEAMBOT_BEAMLINE_CONFIG" ]]; then
    echo "ERROR: BEAMBOT_BEAMLINE_CONFIG is not set." >&2
    exit 1
fi
if [[ ! -f "$BEAMBOT_BEAMLINE_CONFIG" ]]; then
    echo "ERROR: BEAMBOT_BEAMLINE_CONFIG points at missing file: $BEAMBOT_BEAMLINE_CONFIG" >&2
    exit 1
fi

# Copy all output to a log file; -i keeps tee alive through Ctrl-C for shutdown logs.
exec > >(tee -i /tmp/beambot_launch.log) 2>&1
echo "Beamline config: $BEAMBOT_BEAMLINE_CONFIG"

exec ros2 launch beambot beambot_bringup.launch.py "$@"
