cmake_minimum_required(VERSION 3.5)
project(leo_gazebo_plugins)

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(tf2_ros REQUIRED)  # Added line to find tf2_ros

add_library(leo_gazebo_differential_plugin SHARED
  src/differential_plugin.cpp
)
ament_target_dependencies(leo_gazebo_differential_plugin
  gazebo_dev
  gazebo_ros
  tf2_ros  # Added tf2_ros as a dependency
)
ament_export_libraries(leo_gazebo_differential_plugin)

ament_package()

install(
  TARGETS leo_gazebo_differential_plugin
  LIBRARY DESTINATION lib
)
