cmake_minimum_required(VERSION 3.8)
project(exp2_lights_bridge)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(exp2_lights REQUIRED)

find_library(LIBGPIOD gpiod REQUIRED)
find_library(EXP2_LIGHTS_LIB exp2_lights REQUIRED)

add_executable(lights_bridge src/lights_bridge.cpp)

target_include_directories(lights_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

ament_target_dependencies(lights_bridge rclcpp std_msgs)
target_link_libraries(lights_bridge
  ${EXP2_LIGHTS_LIB}
  ${LIBGPIOD}
)

install(TARGETS lights_bridge DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(FILES lights_client.py DESTINATION lib/${PROJECT_NAME})

ament_package()
