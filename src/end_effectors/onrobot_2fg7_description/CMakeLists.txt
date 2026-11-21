cmake_minimum_required(VERSION 3.8)
project(onrobot_2fg7_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
