cmake_minimum_required(VERSION 3.22)
project(beambot)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python package
ament_python_install_package(beambot)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.py"
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(FILES models/spincoater_sample_seg.pt
  DESTINATION share/${PROJECT_NAME}/models
)

# Install Python scripts as executables
install(PROGRAMS
  # Action servers (consolidated - main() is in each server file)
  beambot/action_servers/move_to_server.py
  beambot/action_servers/end_effector_server.py
  beambot/action_servers/sample_pick_place_server.py
  beambot/action_servers/tool_exchange_server.py
  beambot/action_servers/vision_scan_server.py
  beambot/action_servers/vision_task_server.py
  beambot/action_servers/pipettor_server.py
  beambot/action_servers/orchestrator.py
  # Utility nodes
  beambot/mapping/pointcloud_relay.py
  beambot/mapping/octomap_to_planning_scene.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
