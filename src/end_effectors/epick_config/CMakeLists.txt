cmake_minimum_required(VERSION 3.8)
project(epick_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()