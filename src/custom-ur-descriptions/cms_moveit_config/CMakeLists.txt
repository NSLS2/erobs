cmake_minimum_required(VERSION 3.22)
project(cms_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(rclcpp REQUIRED)
find_package(realtime_tools REQUIRED)

add_executable(cms_servo_node src/servo_node_jazzy.cpp)
target_link_libraries(cms_servo_node
  moveit_core::moveit_utils
  moveit_servo::moveit_servo_lib_cpp
  rclcpp::rclcpp
  realtime_tools::realtime_tools)

install(TARGETS cms_servo_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY srdf DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})

ament_package()
