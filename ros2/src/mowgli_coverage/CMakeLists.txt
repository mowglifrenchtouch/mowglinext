cmake_minimum_required(VERSION 3.8)
project(mowgli_coverage)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(opennav_coverage_msgs REQUIRED)

# Fields2Cover 2.0.0 is built and installed at /opt/fields2cover-200 by the
# project Dockerfile / host bootstrap. Pin the find_package PATHS so we
# don't accidentally pull /opt/fields2cover-121 (which is the legacy v1
# install kept around while we migrate). When the v1.2.1 install is
# removed and v2 is the only one on the system, the PATHS hint can drop.
find_package(Fields2Cover 2.0.0 REQUIRED
  PATHS /opt/fields2cover-200
  NO_DEFAULT_PATH)

add_library(${PROJECT_NAME}_core SHARED
  src/coverage_server.cpp
)

target_include_directories(${PROJECT_NAME}_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_core
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  rclcpp_components
  nav2_util
  nav_msgs
  geometry_msgs
  builtin_interfaces
  opennav_coverage_msgs
)

target_link_libraries(${PROJECT_NAME}_core Fields2Cover::Fields2Cover)

# /etc/ld.so.conf.d/fields2cover-121.conf still puts the legacy v1.2.1
# install on the global ld search path. RUNPATH (DT_RUNPATH, the
# default on Noble) only applies to DIRECT dependencies of the binary
# being loaded — transitive lookups (libFields2Cover.so loaded BY
# libmowgli_coverage_core.so) consult the global path first and pick
# v1.2.1 over our v2 install. Set RPATH on the core lib too so the
# loader has a v2-aware path when resolving Fields2Cover.
set_target_properties(${PROJECT_NAME}_core PROPERTIES
  INSTALL_RPATH "/opt/fields2cover-200/lib"
)

add_executable(${PROJECT_NAME}
  src/main.cpp
)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_core)
ament_target_dependencies(${PROJECT_NAME} rclcpp)

# Make sure the executable can find libmowgli_coverage_core.so at the
# install lib/ dir + libFields2Cover.so at /opt/fields2cover-200/lib.
# CMake doesn't propagate RPATH from imported targets reliably, so spell
# both out here. The first $ORIGIN/.. hop is from
# install/mowgli_coverage/lib/mowgli_coverage/ → install/mowgli_coverage/lib/.
set_target_properties(${PROJECT_NAME} PROPERTIES
  INSTALL_RPATH "$ORIGIN/..:/opt/fields2cover-200/lib"
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core)
ament_export_dependencies(
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  rclcpp_components
  nav2_util
  nav_msgs
  geometry_msgs
  builtin_interfaces
  opennav_coverage_msgs
)

ament_package()
