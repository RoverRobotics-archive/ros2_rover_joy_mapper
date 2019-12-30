cmake_minimum_required(VERSION 3.5)
project(openrover_joy_mapper)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY
  scripts/modules
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  scripts/mapper_node.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
