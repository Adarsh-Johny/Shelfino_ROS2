cmake_minimum_required(VERSION 3.8)
project(projects)

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)

# Include directories
include_directories(include)

# Target Rescue Node
add_library(target_rescue SHARED
    src/target_rescue_node.cpp
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
)



ament_package()
# Create the executable before installing
add_executable(target_rescue_node src/main.cpp)
target_link_libraries(target_rescue_node target_rescue)

# Install targets
install(TARGETS
  target_rescue
  target_rescue_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install other files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

# # Enable testing
# if(BUILD_TESTING)
#   find_package(ament_cmake_gtest REQUIRED)

#   # Add test executable
#   ament_add_gtest(test_dubins_planner
#     test/test_dubins_planner.cpp
#   )
#   target_link_libraries(test_dubins_planner
#     target_rescue  # Replace with the actual name of your library
#   )
# endif()
