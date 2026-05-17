#!/usr/bin/env python3
"""
neck_direct.py  (with enable/disable toggle)

Standalone neck controller for the Husky head-cam pan/tilt mount.

Subscribes to:
  /unity/head_pose     (PoseStamped) - head orientation from VR
  /unity/neck_enable   (Bool)        - on/off toggle, default ENABLED

When disabled, motor goal writes stop. Torque stays on so the head doesn't
flop. When re-enabled, tracking resumes immediately from the head's current
position (no jump back to where it was when disabled).

Conventions:
  Motor ID 1 -> pan_0_pan_joint  (yaw, head left/right around Z)
  Motor ID 2 -> tilt_0_tilt_joint (pitch, head nodding up/down around Y)
  Baud:       57600
  Port:       /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTB8HM0E-if00-port0
"""

import math
import time
import signal
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS


# ---- HARDWARE CONFIG --------------------------------------------------
DEVICE_PORT = '/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTB8HM0E-if00-port0'
BAUD_RATE = 57600
PROTOCOL_VERSION = 2.0

PAN_MOTOR_ID = 1
TILT_MOTOR_ID = 2

ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

CENTER_RAW = 2048
RAW_PER_RAD = 2048.0 / math.pi
RAW_MAX = 4095
RAW_MIN = 0


# ---- BEHAVIOR CONFIG --------------------------------------------------
JOINT_LIMIT_RAD = math.pi / 2
DEADBAND_RAD = 0.012
MAX_WRITE_HZ = 30.0
PAN_SIGN = 1.0
TILT_SIGN = 1.0


def quat_to_yaw_pitch(qx, qy, qz, qw):
    yaw = math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    )
    sinp = max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx)))
    pitch = math.asin(sinp)
    return yaw, pitch


def rad_to_raw(angle_rad):
    raw = int(CENTER_RAW + angle_rad * RAW_PER_RAD)
    return max(RAW_MIN, min(RAW_MAX, raw))


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


class NeckDirect(Node):
    def __init__(self):
        super().__init__('neck_direct')

        self.port = PortHandler(DEVICE_PORT)
        self.packet = PacketHandler(PROTOCOL_VERSION)

        if not self.port.openPort():
            self.get_logger().fatal(f'Failed to open {DEVICE_PORT}')
            raise RuntimeError(f'Failed to open {DEVICE_PORT}')
        if not self.port.setBaudRate(BAUD_RATE):
            self.get_logger().fatal(f'Failed to set baud rate {BAUD_RATE}')
            raise RuntimeError(f'Failed to set baud rate {BAUD_RATE}')

        self.get_logger().info(
            f'Opened {DEVICE_PORT} at {BAUD_RATE} baud, protocol {PROTOCOL_VERSION}'
        )

        self._enable_torque(PAN_MOTOR_ID, 'pan')
        self._enable_torque(TILT_MOTOR_ID, 'tilt')

        self._write_goal(PAN_MOTOR_ID, CENTER_RAW)
        self._write_goal(TILT_MOTOR_ID, CENTER_RAW)
        self.get_logger().info('Centered both motors')

        # State
        self.target_pan = 0.0
        self.target_tilt = 0.0
        self.have_pose = False
        self.last_sent_pan = 0.0
        self.last_sent_tilt = 0.0
        self.have_sent = False
        self.last_write_time = 0.0
        self.min_write_interval = 1.0 / MAX_WRITE_HZ
        self.enabled = True   # start enabled; Unity will republish on startup

        # ROS interfaces
        self.create_subscription(
            PoseStamped, '/unity/head_pose', self.head_pose_callback, 10,
        )
        self.create_subscription(
            Bool, '/unity/neck_enable', self.enable_callback, 10,
        )

        self.create_timer(1.0 / (MAX_WRITE_HZ * 2.0), self.maybe_send)

        self.get_logger().info(
            f'neck_direct ready. deadband={math.degrees(DEADBAND_RAD):.2f}deg, '
            f'max_rate={MAX_WRITE_HZ}Hz, range=±{math.degrees(JOINT_LIMIT_RAD):.0f}deg'
        )

    def _enable_torque(self, motor_id, name):
        result, err = self.packet.write1ByteTxRx(
            self.port, motor_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE
        )
        if result != COMM_SUCCESS:
            self.get_logger().error(
                f'Torque enable failed for {name} (ID {motor_id}): '
                f'{self.packet.getTxRxResult(result)}'
            )
        elif err != 0:
            self.get_logger().error(
                f'Torque enable hardware error for {name} (ID {motor_id}): '
                f'{self.packet.getRxPacketError(err)}'
            )
        else:
            self.get_logger().info(f'Torque enabled on {name} (ID {motor_id})')

    def _disable_torque(self, motor_id):
        self.packet.write1ByteTxRx(
            self.port, motor_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE
        )

    def _write_goal(self, motor_id, raw):
        self.packet.write4ByteTxRx(
            self.port, motor_id, ADDR_GOAL_POSITION, int(raw)
        )

    def head_pose_callback(self, msg):
        q = msg.pose.orientation
        yaw, pitch = quat_to_yaw_pitch(q.x, q.y, q.z, q.w)
        self.target_pan = clamp(PAN_SIGN * yaw,
                                -JOINT_LIMIT_RAD, JOINT_LIMIT_RAD)
        self.target_tilt = clamp(TILT_SIGN * pitch,
                                 -JOINT_LIMIT_RAD, JOINT_LIMIT_RAD)
        self.have_pose = True

    def enable_callback(self, msg):
        new_state = bool(msg.data)
        if new_state and not self.enabled:
            # Re-enabling: clear have_sent so the next send fires regardless
            # of deadband. This jumps the motor to wherever the head is now.
            self.have_sent = False
            self.get_logger().info('Neck ENABLED (resume tracking)')
        elif not new_state and self.enabled:
            self.get_logger().info('Neck DISABLED (holding last position)')
        self.enabled = new_state

    def maybe_send(self):
        if not self.enabled:
            return  # frozen
        if not self.have_pose:
            return

        now = time.time()
        if now - self.last_write_time < self.min_write_interval:
            return

        if not self.have_sent:
            self._send_now()
            return

        d_pan = abs(self.target_pan - self.last_sent_pan)
        d_tilt = abs(self.target_tilt - self.last_sent_tilt)
        if d_pan < DEADBAND_RAD and d_tilt < DEADBAND_RAD:
            return

        self._send_now()

    def _send_now(self):
        pan_raw = rad_to_raw(self.target_pan)
        tilt_raw = rad_to_raw(self.target_tilt)
        self._write_goal(PAN_MOTOR_ID, pan_raw)
        self._write_goal(TILT_MOTOR_ID, tilt_raw)
        self.last_sent_pan = self.target_pan
        self.last_sent_tilt = self.target_tilt
        self.have_sent = True
        self.last_write_time = time.time()

    def shutdown(self):
        self.get_logger().info('Shutting down: centering and releasing motors')
        try:
            self._write_goal(PAN_MOTOR_ID, CENTER_RAW)
            self._write_goal(TILT_MOTOR_ID, CENTER_RAW)
            time.sleep(0.5)
            self._disable_torque(PAN_MOTOR_ID)
            self._disable_torque(TILT_MOTOR_ID)
            self.port.closePort()
        except Exception as e:
            self.get_logger().warn(f'Shutdown error (ignored): {e}')


def main():
    rclpy.init()
    node = NeckDirect()

    def handle_signal(signum, frame):
        rclpy.shutdown()

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
