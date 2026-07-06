#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo


class CameraInfoPublisher(Node):
    def __init__(self):
        super().__init__('camera_info_publisher')

        self.declare_parameter('camera_frame_id', 'camera_link')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fx', 525.0)
        self.declare_parameter('fy', 525.0)
        self.declare_parameter('cx', 320.0)
        self.declare_parameter('cy', 240.0)
        self.declare_parameter('fps', 30.0)

        topic = self.get_parameter('camera_info_topic').value
        self.publisher = self.create_publisher(CameraInfo, topic, 10)
        self.msg = self._make_camera_info()

        fps = float(self.get_parameter('fps').value)
        self.timer = self.create_timer(1.0 / fps, self._publish)

    def _make_camera_info(self):
        msg = CameraInfo()
        msg.header.frame_id = self.get_parameter('camera_frame_id').value
        msg.width = int(self.get_parameter('width').value)
        msg.height = int(self.get_parameter('height').value)

        fx = float(self.get_parameter('fx').value)
        fy = float(self.get_parameter('fy').value)
        cx = float(self.get_parameter('cx').value)
        cy = float(self.get_parameter('cy').value)

        msg.distortion_model = 'plumb_bob'
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return msg

    def _publish(self):
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.msg)


def main(args=None):
    rclpy.init(args=args)
    node = CameraInfoPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
