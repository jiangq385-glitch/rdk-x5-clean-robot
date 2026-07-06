#!/usr/bin/env python3
"Publish the arm commanded joints as JointState."

from __future__ import annotations

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory


class JointStateNode(Node):
    "Mirror the latest arm target to /joint_states and /arm/joint_states."

    def __init__(self):
        super().__init__('joint_state_node')

        self.declare_parameter('joint_names', ['arm_joint_1', 'arm_joint_2', 'arm_joint_3', 'arm_joint_4', 'arm_joint_5', 'arm_joint_6'])
        self.declare_parameter('home_positions', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('target_topic', '/arm/joint_target')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('arm_joint_states_topic', '/arm/joint_states')
        self.declare_parameter('publish_rate_hz', 20.0)

        self.joint_names = list(self.get_parameter('joint_names').value)
        self.positions = [float(value) for value in self.get_parameter('home_positions').value]
        self.velocities = [0.0 for _ in self.joint_names]
        self._last_update_ns = None

        if len(self.positions) != len(self.joint_names):
            raise ValueError('home_positions length must match joint_names length')

        target_topic = self.get_parameter('target_topic').value
        joint_states_topic = self.get_parameter('joint_states_topic').value
        arm_joint_states_topic = self.get_parameter('arm_joint_states_topic').value
        publish_rate_hz = max(1.0, float(self.get_parameter('publish_rate_hz').value))

        self.joint_states_pub = self.create_publisher(JointState, joint_states_topic, 10)
        self.arm_joint_states_pub = self.create_publisher(JointState, arm_joint_states_topic, 10)
        self.create_subscription(JointTrajectory, target_topic, self._on_target, 10)
        self.create_timer(1.0 / publish_rate_hz, self._publish_joint_state)

    def _on_target(self, msg: JointTrajectory):
        if not msg.points:
            return
        if msg.joint_names and list(msg.joint_names) != self.joint_names:
            self.get_logger().warn('ignore target with mismatched joint_names order')
            return

        point = msg.points[-1]
        positions = list(point.positions)
        if len(positions) != len(self.joint_names):
            self.get_logger().warn('ignore target with mismatched positions length')
            return

        now_ns = self.get_clock().now().nanoseconds
        if self._last_update_ns is None:
            self.velocities = [0.0 for _ in self.joint_names]
        else:
            dt = max((now_ns - self._last_update_ns) / 1e9, 1e-3)
            self.velocities = [
                (new_value - old_value) / dt
                for old_value, new_value in zip(self.positions, positions)
            ]
        self.positions = positions
        self._last_update_ns = now_ns

    def _publish_joint_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.joint_names)
        msg.position = list(self.positions)
        msg.velocity = list(self.velocities)
        self.joint_states_pub.publish(msg)
        self.arm_joint_states_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
