#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time

import cv2
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from .preprocess import preprocess_yolov8


class MipiYoloNode(Node):
    def __init__(self):
        super().__init__('mipi_yolo_node')

        self.declare_parameter('image_topic', '/image_raw')
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value

        self.bridge = CvBridge()
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.frame_count = 0
        self.start_time = time.time()

        self.subscription = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            image_qos
        )

        self.get_logger().info(f'已启动，正在订阅图像话题: {image_topic}')

    def image_callback(self, msg: Image):
        self.frame_count += 1
        elapsed = time.time() - self.start_time

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'图像转换失败: {e}')
            return

        tensor, scale, dw, dh = preprocess_yolov8(cv_image)

        if self.frame_count % 30 == 0:
            fps = self.frame_count / elapsed if elapsed > 0 else 0.0
            h, w = cv_image.shape[:2]
            self.get_logger().info(
                f'已接收 {self.frame_count} 帧, '
                f'msg: {msg.width}x{msg.height}, '
                f'cv_image: {w}x{h}, '
                f'encoding: {msg.encoding}, '
                f'fps: {fps:.2f}, '
                f'tensor.shape: {tensor.shape}, '
                f'scale: {scale:.6f}, '
                f'dw: {dw:.2f}, '
                f'dh: {dh:.2f}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = MipiYoloNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()