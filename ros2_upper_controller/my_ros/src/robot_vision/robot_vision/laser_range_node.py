from __future__ import annotations

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Range


class LaserRangeNode(Node):
    def __init__(self) -> None:
        super().__init__('laser_range_node')
        self.create_subscription(LaserScan, '/scan', self.on_scan, 10)
        self.range_pub = self.create_publisher(Range, '/perception/laser_range', 10)

    def on_scan(self, msg: LaserScan) -> None:
        if not msg.ranges:
            return
        mid = len(msg.ranges) // 2
        range_msg = Range()
        range_msg.header = msg.header
        range_msg.radiation_type = Range.INFRARED
        range_msg.field_of_view = 0.1
        range_msg.min_range = msg.range_min
        range_msg.max_range = msg.range_max
        range_msg.range = float(msg.ranges[mid])
        self.range_pub.publish(range_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LaserRangeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
