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
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Build the arm teleop node
add_executable(arm_teleop_node src/arm_teleop_node.cpp)
ament_target_dependencies(arm_teleop_node
  rclcpp
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
)

# Install the executable
install(TARGETS
  arm_teleop_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files (will create the dir later)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
