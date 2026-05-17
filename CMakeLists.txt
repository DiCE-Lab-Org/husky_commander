cmake_minimum_required(VERSION 3.8)
project(husky_commander)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)

# VR pose -> FZI cartesian_motion_controller bridge.
# Also handles gripper Float64 -> GripperCommand action forwarding.
add_executable(arm_teleop_node_cartesian src/arm_teleop_node_cartesian.cpp)
ament_target_dependencies(arm_teleop_node_cartesian
  rclcpp
  rclcpp_action
  geometry_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  control_msgs
)

install(TARGETS
  arm_teleop_node_cartesian
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/joint_state_relay.py
  scripts/neck_direct.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
