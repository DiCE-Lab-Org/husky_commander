cmake_minimum_required(VERSION 3.8)
project(husky_commander)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Build the Servo-based arm teleop node (current default).
add_executable(arm_teleop_node src/arm_teleop_node.cpp)
ament_target_dependencies(arm_teleop_node
  rclcpp
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  std_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
)

# Build the legacy Cartesian-path arm teleop node.
# Kept for fallback / comparison; not used by default in husky.launch.py.
add_executable(arm_teleop_node_cartesian src/arm_teleop_node_cartesian.cpp)
ament_target_dependencies(arm_teleop_node_cartesian
  rclcpp
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
)

# Install both executables
install(TARGETS
  arm_teleop_node
  arm_teleop_node_cartesian
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files (will create the dir later)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()