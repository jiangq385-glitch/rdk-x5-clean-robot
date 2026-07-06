from __future__ import annotations

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from robot_msgs.msg import ArmStatus, ObjectPose, RobotState


class RobotStateAggregator(Node):
    def __init__(self) -> None:
        super().__init__('robot_state_aggregator')
        self.left_ready = False
        self.right_ready = False
        self.detected_objects: list[str] = []
        self.available_actions = ['pick', 'place', 'navigate_to_object', 'clean_area', 'return_home']

        self.create_subscription(ArmStatus, '/arm/status', self.on_arm_status, 10)
        self.create_subscription(JointState, '/arm/joint_states', self.on_joint_state, 10)
        self.create_subscription(ObjectPose, '/perception/object_pose', self.on_object_pose, 10)
        self.state_pub = self.create_publisher(RobotState, '/robot/state', 10)
        self.create_timer(0.5, self.publish_state)

    def on_arm_status(self, msg: ArmStatus) -> None:
        ready = msg.moving_state == 0 and not bool(msg.error_text)
        if msg.arm_name == 'left':
            self.left_ready = ready
        elif msg.arm_name == 'right':
            self.right_ready = ready

    def on_joint_state(self, _: JointState) -> None:
        return

    def on_object_pose(self, msg: ObjectPose) -> None:
        if msg.object_name not in self.detected_objects:
            self.detected_objects.append(msg.object_name)

    def publish_state(self) -> None:
        state = RobotState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.battery_pct = 100.0
        state.mode = 'IDLE'
        state.left_arm_ready = self.left_ready
        state.right_arm_ready = self.right_ready
        state.detected_objects = list(self.detected_objects)
        state.available_actions = list(self.available_actions)
        self.state_pub.publish(state)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RobotStateAggregator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
