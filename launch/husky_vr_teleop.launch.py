#!/usr/bin/env python3
"""
husky_vr_teleop.launch.py

One launch file that brings up the entire VR teleop pipeline AFTER you have
already done the cartesian boot step (~/scripts/start_cartesian.sh + press
Play on pendants).

Components:
  1. ros_tcp_endpoint            (Unity <-> ROS bridge, port 10000)
  2. joint_state_relay           (platform joint states -> /unity/joint_states)
  3. neck_direct                 (Unity head pose -> Dynamixel pan/tilt)
  4. usb_cam node                (stereo USB camera -> /a200_0876/sensors/camera_3/...)
  5. arm_teleop_node_cartesian   (VR hands -> FZI cartesian controllers)

Usage:
  ros2 launch husky_commander husky_vr_teleop.launch.py
  ros2 launch husky_commander husky_vr_teleop.launch.py enable_neck:=false
  ros2 launch husky_commander husky_vr_teleop.launch.py position_scale:=0.7

The Python helper scripts (joint_state_relay, neck_direct) live in the
husky_commander package's scripts/ directory and are installed as executables.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument("enable_tcp",         default_value="true"),
        DeclareLaunchArgument("enable_joint_relay", default_value="true"),
        DeclareLaunchArgument("enable_neck",        default_value="true"),
        DeclareLaunchArgument("enable_camera",      default_value="true"),
        DeclareLaunchArgument("enable_teleop",      default_value="true"),
        DeclareLaunchArgument("position_scale",     default_value="0.5"),
        DeclareLaunchArgument("smooth_step",        default_value="0.1"),
        DeclareLaunchArgument("translation_only",   default_value="false"),
        DeclareLaunchArgument("camera_device",      default_value="/dev/video12"),
        DeclareLaunchArgument("camera_width",       default_value="1280"),
        DeclareLaunchArgument("camera_height",      default_value="480"),
        DeclareLaunchArgument("camera_fps",         default_value="30.0"),
    ]

    enable_tcp         = LaunchConfiguration("enable_tcp")
    enable_joint_relay = LaunchConfiguration("enable_joint_relay")
    enable_neck        = LaunchConfiguration("enable_neck")
    enable_camera      = LaunchConfiguration("enable_camera")
    enable_teleop      = LaunchConfiguration("enable_teleop")
    position_scale     = LaunchConfiguration("position_scale")
    smooth_step        = LaunchConfiguration("smooth_step")
    translation_only   = LaunchConfiguration("translation_only")
    camera_device      = LaunchConfiguration("camera_device")
    camera_width       = LaunchConfiguration("camera_width")
    camera_height      = LaunchConfiguration("camera_height")
    camera_fps         = LaunchConfiguration("camera_fps")

    # 1. ROS-TCP-Endpoint
    tcp_endpoint = Node(
        package="ros_tcp_endpoint",
        executable="default_server_endpoint",
        name="ros_tcp_endpoint",
        output="screen",
        parameters=[{
            "ROS_IP": "0.0.0.0",
            "ROS_TCP_PORT": 10000,
        }],
        condition=IfCondition(enable_tcp),
    )

    # 2. joint_state_relay (now a package-installed script)
    joint_relay = Node(
        package="husky_commander",
        executable="joint_state_relay.py",
        name="joint_state_relay",
        output="screen",
        condition=IfCondition(enable_joint_relay),
    )

    # 3. neck_direct (now a package-installed script)
    neck = Node(
        package="husky_commander",
        executable="neck_direct.py",
        name="neck_direct",
        output="screen",
        condition=IfCondition(enable_neck),
    )

    # 4. usb_cam for stereo USB cam
    usb_cam = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        name="usb_cam",
        namespace="/a200_0876/sensors/camera_3",
        output="screen",
        parameters=[{
            "video_device":  camera_device,
            "pixel_format":  "mjpeg2rgb",
            "image_width":   camera_width,
            "image_height":  camera_height,
            "framerate":     camera_fps,
            "camera_name":   "camera_3",
            "frame_id":      "camera_3_color_optical_frame",
        }],
        condition=IfCondition(enable_camera),
    )

    # 5. arm teleop bridge
    arm_teleop = Node(
        package="husky_commander",
        executable="arm_teleop_node_cartesian",
        name="arm_teleop_node",
        namespace="/a200_0876",
        output="screen",
        parameters=[{
            "position_scale":   position_scale,
            "smooth_step":      smooth_step,
            "translation_only": translation_only,
        }],
        condition=IfCondition(enable_teleop),
    )

    return LaunchDescription(args + [
        LogInfo(msg="==> Launching Husky VR teleop pipeline"),
        LogInfo(msg="    (cartesian controllers should already be active via ~/scripts/start_cartesian.sh)"),
        tcp_endpoint,
        joint_relay,
        neck,
        usb_cam,
        arm_teleop,
    ])
