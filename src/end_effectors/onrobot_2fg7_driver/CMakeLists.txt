cmake_minimum_required(VERSION 3.8)
project(onrobot_2fg7_driver)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python package (version-independent path)
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  onrobot_2fg7_driver/driver_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME onrobot_2fg7_driver_node
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
