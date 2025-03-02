cmake_minimum_required(VERSION 3.8)
project(projects)

# Enable compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(obstacles_msgs REQUIRED)
find_package(graph_msgs REQUIRED)

# Include directories
include_directories(include)

 # Only non-node files
add_library(target_rescue SHARED
    src/dubins_planner.cpp
)

# Link dependencies
ament_target_dependencies(target_rescue
    rclcpp
    geometry_msgs
    nav_msgs
    visualization_msgs
    tf2
    tf2_geometry_msgs
    nav2_msgs
    obstacles_msgs
    graph_msgs
)

#  Executables for each node
add_executable(target_rescue_node src/target_rescue_node.cpp)
target_link_libraries(target_rescue_node target_rescue)

add_executable(map_generator_node src/map_generator.cpp)
target_link_libraries(map_generator_node target_rescue)

# Install all targets
install(TARGETS
  target_rescue
  target_rescue_node
  map_generator_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
)

# Export dependencies
ament_export_dependencies(
    rclcpp
    geometry_msgs
    nav_msgs
    visualization_msgs
    tf2
    tf2_geometry_msgs
    nav2_msgs
    obstacles_msgs
    graph_msgs
)

# Finalize the package
ament_package()
