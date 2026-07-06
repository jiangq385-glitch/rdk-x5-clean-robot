from __future__ import annotations

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


class ObjectTracker(Node):
    def __init__(self) -> None:
        super().__init__('object_tracker')
        self.create_subscription(PoseStamped, '/perception/object_pose', self.on_object_pose, 10)

    def on_object_pose(self, _: PoseStamped) -> None:
        return


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObjectTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
