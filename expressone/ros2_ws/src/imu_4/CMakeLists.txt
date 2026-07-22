cmake_minimum_required(VERSION 3.8)
project(hi12_imu)

# Default to C++17 (ROS2 Humble+ standard)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

# ---- Dependencies ----
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# ---- Library: hi12_imu_core (reusable protocol + serial driver) ----
add_library(hi12_imu_core STATIC
  src/protocol.cpp
  src/serial_port.cpp
  src/hi12_driver.cpp
)
target_include_directories(hi12_imu_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(hi12_imu_core PUBLIC
  rclcpp::rclcpp
)
# On Linux, serial port uses pthread for the background reader thread
if(UNIX)
  target_link_libraries(hi12_imu_core PUBLIC pthread)
endif()

# ---- Executable: hi12_imu_node ----
add_executable(hi12_imu_node
  src/hi12_node.cpp
)
target_link_libraries(hi12_imu_node PRIVATE
  hi12_imu_core
  rclcpp::rclcpp
)
ament_target_dependencies(hi12_imu_node PUBLIC sensor_msgs)

# ---- Install ----
install(TARGETS hi12_imu_node hi12_imu_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# ---- Linting ----
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
