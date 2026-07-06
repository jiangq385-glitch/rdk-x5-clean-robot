#!/usr/bin/env python3
"""将原始 LaserScan 重采样为固定点数的 scan。"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanResampler(Node):
    def __init__(self):
        super().__init__('scan_resampler')

        self.declare_parameter('input_topic', '/scan')
        self.declare_parameter('output_topic', '/scan_fixed')
        self.declare_parameter('target_count', 692)

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.target_count = int(self.get_parameter('target_count').value)

        if self.target_count < 2:
            raise ValueError('target_count must be >= 2')

        self.publisher = self.create_publisher(LaserScan, self.output_topic, 10)
        self.subscription = self.create_subscription(LaserScan, self.input_topic, self._on_scan, 10)

        self.get_logger().info(
            f'Resampling {self.input_topic} -> {self.output_topic} with target_count={self.target_count}'
        )

    @staticmethod
    def _is_finite_range(value: float) -> bool:
        return math.isfinite(value) and value > 0.0

    @staticmethod
    def _lerp(a: float, b: float, t: float) -> float:
        return a + (b - a) * t

    def _sample_range(self, scan: LaserScan, angle: float) -> float:
        if scan.angle_increment == 0.0 or not scan.ranges:
            return float('inf')

        source_index = (angle - scan.angle_min) / scan.angle_increment
        if source_index < 0.0 or source_index > (len(scan.ranges) - 1):
            return float('inf')

        left_index = int(math.floor(source_index))
        right_index = min(left_index + 1, len(scan.ranges) - 1)
        fraction = source_index - left_index

        left_value = scan.ranges[left_index]
        right_value = scan.ranges[right_index]

        if left_index == right_index:
            return left_value if self._is_finite_range(left_value) else float('inf')

        if not self._is_finite_range(left_value) and not self._is_finite_range(right_value):
            return float('inf')
        if not self._is_finite_range(left_value):
            return right_value
        if not self._is_finite_range(right_value):
            return left_value

        return self._lerp(left_value, right_value, fraction)

    def _on_scan(self, scan: LaserScan):
        if not scan.ranges:
            return

        output = LaserScan()
        output.header = scan.header
        output.header.stamp = scan.header.stamp
        output.header.frame_id = scan.header.frame_id
        output.angle_min = scan.angle_min
        output.angle_max = scan.angle_max
        output.angle_increment = (scan.angle_max - scan.angle_min) / (self.target_count - 1)
        output.time_increment = scan.time_increment
        output.scan_time = scan.scan_time
        output.range_min = scan.range_min
        output.range_max = scan.range_max
        output.ranges = []

        for index in range(self.target_count):
            angle = output.angle_min + index * output.angle_increment
            output.ranges.append(self._sample_range(scan, angle))

        self.publisher.publish(output)


def main():
    rclpy.init()
    node = ScanResampler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()