from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("a200_0876", package_name="husky_dual_ur_moveit_config")
        .robot_description(file_path="config/a200_0876.urdf.xacro")
        .robot_description_semantic(file_path="config/a200_0876.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .to_moveit_configs()
    )

    arm_teleop_node = Node(
        package="husky_commander",
        executable="arm_teleop_node",
        name="arm_teleop_node",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    return LaunchDescription([arm_teleop_node])