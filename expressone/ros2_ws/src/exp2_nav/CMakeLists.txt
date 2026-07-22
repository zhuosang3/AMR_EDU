cmake_minimum_required(VERSION 3.8)
project(exp2_nav)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/reloc_manager.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
