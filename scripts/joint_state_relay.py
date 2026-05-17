#!/usr/bin/env python3
"""
Simple relay: republishes /a200_0876/platform/joint_states to /joint_states
so the ROS-TCP-Endpoint can pick it up for Unity.

Run with:
  export ROS_DOMAIN_ID=0
  python3 joint_state_relay.py
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateRelay(Node):
    def __init__(self):
        super().__init__('joint_state_relay')

        self.publisher = self.create_publisher(JointState, '/unity/joint_states', 10)

        self.create_subscription(
            JointState,
            '/a200_0876/platform/joint_states',
            self.callback,
            10
        )

        self.get_logger().info('Relay started: /a200_0876/platform/joint_states -> /joint_states')

    def callback(self, msg):
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = JointStateRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
