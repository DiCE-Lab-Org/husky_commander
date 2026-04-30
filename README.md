# husky_commander

VR teleoperation arm controller for the Husky A200 with dual UR5e arms.

`arm_teleop_node` subscribes to pose commands from a VR application and drives both UR5e arms using MoveIt motion planning.

## Topics

Subscribed:
- `/unity/arm_0_target_pose`, `/unity/arm_1_target_pose` (`geometry_msgs/PoseStamped`)
- `/unity/gripper_0_cmd`, `/unity/gripper_1_cmd` (`std_msgs/Float32`)

Pose protocol uses `frame_id` field: `grip_start` (lock anchor), `delta` (offset from anchor), `grip_end` (release).

## Dependencies

- ROS 2 Humble + MoveIt 2
- [husky_description](https://github.com/DiCE-Lab-Org/husky_description)
- [husky_dual_ur_moveit_config](https://github.com/DiCE-Lab-Org/husky_dual_ur_moveit_config)

## Run

Usually launched via [husky_bringup](https://github.com/DiCE-Lab-Org/husky_bringup):
