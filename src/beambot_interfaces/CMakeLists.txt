cmake_minimum_required(VERSION 3.8)
project(beambot_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/BeambotExecution.action"
  "action/MoveToAction.action"
  "action/EndEffectorAction.action"
  "action/PickSampleAction.action"
  "action/PlaceSampleAction.action"
  "action/ToolExchangeAction.action"
  "action/VisionScanAction.action"
  "action/VisionTaskAction.action"
  "action/PipettorAction.action"
  DEPENDENCIES std_msgs
)

ament_package()
