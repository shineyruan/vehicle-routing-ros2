cmake_minimum_required(VERSION 3.8)
project(dvr_nodes)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)

ament_auto_find_build_dependencies()

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

ament_auto_add_library(vehicle_node SHARED
  src/vehicle_node.cpp
)

rclcpp_components_register_node(vehicle_node
  PLUGIN "dvr::VehicleNode"
  EXECUTABLE vehicle
)

ament_auto_add_library(vehicle_viz_node SHARED
  src/visualization/vehicle_viz_node.cpp
)

rclcpp_components_register_node(vehicle_viz_node
  PLUGIN "dvr::VehicleVizNode"
  EXECUTABLE vehicle_visualization
)

ament_auto_add_library(world_viz_node SHARED
  src/visualization/world_viz_node.cpp
)

rclcpp_components_register_node(world_viz_node
  PLUGIN "dvr::WorldVizNode"
  EXECUTABLE world_visualization
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
