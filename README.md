# husky_commander

VR teleoperation pipeline for the Husky A200 with dual UR5e arms, Dynamixel pan/tilt neck, and a stereo USB head camera. Designed for Quest / Meta XR controllers via Unity.

## What's in this package

```
husky_commander/
├── src/
│   └── arm_teleop_node_cartesian.cpp     C++ bridge: Unity VR pose -> FZI cartesian controllers
├── scripts/
│   ├── joint_state_relay.py              Republishes platform joint states for Unity
│   └── neck_direct.py                    Drives Dynamixel pan/tilt from VR head pose
└── launch/
    └── husky_vr_teleop.launch.py         Brings up all 5 components together
```

## Topics

**Subscribed:**
- `/unity/arm_0_target_pose`, `/unity/arm_1_target_pose` (`geometry_msgs/PoseStamped`)
  - Pose protocol: `frame_id="grip_start"` (capture anchor), `frame_id="delta"` (relative move), `frame_id="grip_end"` (release).
- `/unity/gripper_0_cmd`, `/unity/gripper_1_cmd` (`std_msgs/Float64`, 0.0=open 1.0=close)
- `/a200_0876/manipulators/cartesian_motion_controller_arm_X/current_pose` (read EE pose for anchoring)

**Published:**
- `/a200_0876/manipulators/cartesian_motion_controller_arm_X/target_frame` (`geometry_msgs/PoseStamped`)
- Gripper commands via `GripperCommand` action

## Parameters

| Parameter | Default | Notes |
|---|---|---|
| `position_scale` | 0.5 | Hand motion → arm motion ratio. 1.0 = 1:1 |
| `smooth_step` | 0.1 | Exponential smoothing factor. Lower = smoother, more lag |
| `mirror_mode` | false | Side-by-side (operator behind robot). True = face-to-face |
| `operator_yaw_offset_deg` | 0.0 | Yaw alignment between operator and chassis |
| `translation_only` | false | Lock orientation to anchor (for top-down pick-and-place tests) |
| `sign_x`, `sign_y`, `sign_z` | +1 each | Per-axis empirical sign flips for tuning |

## Boot workflow

Before launching the teleop pipeline, the FZI cartesian controllers must be active. Two helper scripts live on the robot at `~/scripts/`:

- `~/scripts/start_cartesian.sh` — boot-time setup. Copies the patched yaml (with URDF embedded) into Clearpath's config directory, restarts the manipulators service, waits for the user to press Play on both UR pendants, then switches controllers to cartesian mode. Idempotent — safe to run multiple times.
- `~/scripts/generate_cartesian_yaml.py` — one-time tool that embeds the current URDF dump into `~/control_with_cartesian.yaml`. Only needs to be re-run if the URDF changes (calibration update, new gripper, etc).

Daily startup after powering on the robot:
```bash
~/scripts/start_cartesian.sh
# Walk to both pendants, load External Control program, press Play, hit Enter in the script
ros2 launch husky_commander husky_vr_teleop.launch.py
```

The launch file brings up:
1. `ros_tcp_endpoint` (Unity ↔ ROS bridge, port 10000)
2. `joint_state_relay` (platform joint states → `/unity/joint_states`)
3. `neck_direct` (Unity head pose → Dynamixel pan/tilt motors via U2D2)
4. `usb_cam` (stereo USB head camera → `/a200_0876/sensors/camera_3/...`)
5. `arm_teleop_node_cartesian` (this package's C++ bridge)

Disable individual components for debugging:
```bash
ros2 launch husky_commander husky_vr_teleop.launch.py enable_neck:=false
ros2 launch husky_commander husky_vr_teleop.launch.py translation_only:=true
ros2 launch husky_commander husky_vr_teleop.launch.py position_scale:=0.3 smooth_step:=0.05
```

## Dependencies

- ROS 2 Humble
- FZI `cartesian_controllers` (built from source)
- `ros_tcp_endpoint` (Unity Robotics package)
- `usb_cam` (`ros-humble-usb-cam`)
- `dynamixel_sdk` (pip or apt)
- A working Clearpath manipulators service running the dual UR5e
- Quest / Meta XR headset paired with a Unity scene that publishes the expected topics

## Companion Unity scripts

This package needs companion C# scripts running in Unity:
- `CartesianArmController.cs` — publishes VR hand pose to `/unity/arm_X_target_pose`
- `HeadPosePublisher.cs` — publishes head pose to `/unity/head_pose` and neck toggle to `/unity/neck_enable`
- `GripperController.cs` — trigger → `/unity/gripper_X_cmd`
- `BaseDriveController.cs` — joystick → `/a200_0876/platform/cmd_vel_unstamped`
- `OdometryTracker.cs` — odom → URDF in Unity scene

## License

Apache-2.0
