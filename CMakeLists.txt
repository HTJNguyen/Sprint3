cmake_minimum_required(VERSION 3.5)
project(sprint3)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)  # Added for SpawnEntity service
find_package(Eigen3 REQUIRED)        # Added for Eigen3 dependency

# Include directories
include_directories(include ${EIGEN3_INCLUDE_DIRS})  # Include Eigen3 directories

# Declare the first executable (detect_cylinder)
add_executable(detect_cylinder src/detect_cylinder.cpp)

# Declare the second executable (spawn_cylinder_node)
add_executable(spawn_cylinder src/spawn_cylinder.cpp)

# Specify libraries to link against for detect_cylinder
ament_target_dependencies(detect_cylinder
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_geometry_msgs
)

# Link Eigen3 to the detect_cylinder target
target_link_libraries(detect_cylinder Eigen3::Eigen)

# Specify libraries to link against for spawn_cylinder_node
ament_target_dependencies(spawn_cylinder
  rclcpp
  gazebo_msgs  # Gazebo services dependency
)

# Install targets
install(TARGETS
  detect_cylinder
  spawn_cylinder
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch and other necessary files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Enable testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
