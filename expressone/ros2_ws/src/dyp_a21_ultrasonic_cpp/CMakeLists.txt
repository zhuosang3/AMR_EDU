cmake_minimum_required(VERSION 3.8)
project(dyp_a21_ultrasonic_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -O2)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(ultrasonic_node src/ultrasonic_node.cpp)
ament_target_dependencies(ultrasonic_node rclcpp sensor_msgs)

install(TARGETS ultrasonic_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
