cmake_minimum_required(VERSION 3.8)
project(power_monitor)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

find_library(MOSQUITTO_LIB mosquitto REQUIRED)
find_path(MOSQUITTO_INCLUDE_DIR mosquitto.h REQUIRED)

add_library(power_monitor_core STATIC
  src/battery_protocol.cpp
  src/can_reader.cpp
  src/mqtt_publisher.cpp
)
target_include_directories(power_monitor_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(power_monitor_core PUBLIC ${MOSQUITTO_LIB})
target_include_directories(power_monitor_core SYSTEM PUBLIC ${MOSQUITTO_INCLUDE_DIR})
target_compile_definitions(power_monitor_core PRIVATE MOSQ_NO_DEPRECATED)

add_executable(power_monitor_node src/power_monitor_node.cpp)
target_link_libraries(power_monitor_node
  power_monitor_core
  rclcpp::rclcpp
  std_msgs::std_msgs__rosidl_typesupport_cpp
)

install(TARGETS power_monitor_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
