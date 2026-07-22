cmake_minimum_required(VERSION 3.8)
project(exp2_lights C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)

# ── 系统库 ──
find_library(LIBGPIOD gpiod REQUIRED)

# ── 静态库 ──
add_library(exp2_lights STATIC lights.c)
target_include_directories(exp2_lights PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(exp2_lights PUBLIC ${LIBGPIOD})

# ── 测试程序 ──
add_executable(test_lights test_lights.c)
target_link_libraries(test_lights exp2_lights)

# ── 安装 ──
install(TARGETS exp2_lights ARCHIVE DESTINATION lib)
install(TARGETS test_lights  DESTINATION lib/${PROJECT_NAME})
install(FILES lights.h     DESTINATION include/${PROJECT_NAME})

ament_package()
