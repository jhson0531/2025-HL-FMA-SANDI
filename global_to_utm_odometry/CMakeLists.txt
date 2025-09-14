cmake_minimum_required(VERSION 3.8)
project(global_to_utm_odometry)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

# Create executable
add_executable(global_to_utm_odometry_node
  global_to_utm_odometry/global_to_utm_odometry.cpp
)

# Include directories
target_include_directories(global_to_utm_odometry_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Link libraries
ament_target_dependencies(global_to_utm_odometry_node
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  eigen3_cmake_module
)

# Link Eigen3
target_link_libraries(global_to_utm_odometry_node
  Eigen3::Eigen
)

# Install executable
install(TARGETS
  global_to_utm_odometry_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files if they exist
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}/
  )
endif()

# Install config files if they exist
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config")
  install(DIRECTORY
    config
    DESTINATION share/${PROJECT_NAME}/
  )
endif()

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

