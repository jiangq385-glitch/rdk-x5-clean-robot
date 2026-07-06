from __future__ import annotations

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from robot_msgs.msg import ObjectOnPlane


class ObjectPoseNode(Node):
    "Map detector centers on the camera image to XY coordinates on the work plane."

    def __init__(self) -> None:
        super().__init__('object_on_plane_node')
        self.declare_parameter('detections_topic', '/vision/detections_json')
        self.declare_parameter('object_on_plane_topic', '/perception/object_on_plane')
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('homography', [0.001, 0.0, -0.32, 0.0, 0.001, -0.24, 0.0, 0.0, 1.0])
        self.declare_parameter('min_confidence', 0.35)
        self.declare_parameter('target_class', '')

        self.homography = [float(v) for v in self.get_parameter('homography').value]
        if len(self.homography) != 9:
            raise ValueError('homography must contain 9 numbers')
        self.min_confidence = float(self.get_parameter('min_confidence').value)
        self.target_class = str(self.get_parameter('target_class').value)
        self.target_frame = str(self.get_parameter('target_frame').value)
        self.plane_pub = self.create_publisher(ObjectOnPlane, self.get_parameter('object_on_plane_topic').value, 10)
        self.create_subscription(String, self.get_parameter('detections_topic').value, self.on_detections, 10)

    def on_detections(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f'ignore malformed detections json: {exc}')
            return
        detection = self._select_detection(payload)
        if detection is None:
            return
        center = self._center_of_detection(detection)
        if center is None:
            return
        x_plane, y_plane = self._pixel_to_plane(center[0], center[1])
        out = ObjectOnPlane()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.target_frame
        out.class_name = str(detection.get('class_name') or detection.get('label') or detection.get('class') or '')
        out.confidence = float(detection.get('confidence', detection.get('score', 0.0)))
        out.x = float(x_plane)
        out.y = float(y_plane)
        self.plane_pub.publish(out)

    def _select_detection(self, payload: dict) -> dict | None:
        detections = payload.get('detections', []) if isinstance(payload, dict) else []
        best = None
        best_score = self.min_confidence
        for detection in detections:
            if not isinstance(detection, dict):
                continue
            class_name = str(detection.get('class_name') or detection.get('label') or detection.get('class') or '')
            if self.target_class and class_name != self.target_class:
                continue
            score = float(detection.get('confidence', detection.get('score', 0.0)))
            if score >= best_score:
                best = detection
                best_score = score
        return best

    def _center_of_detection(self, detection: dict) -> tuple[float, float] | None:
        center = detection.get('center')
        if isinstance(center, dict) and 'x' in center and 'y' in center:
            return float(center['x']), float(center['y'])
        if isinstance(center, (list, tuple)) and len(center) >= 2:
            return float(center[0]), float(center[1])
        keys = ('x_min', 'y_min', 'x_max', 'y_max')
        if all(key in detection for key in keys):
            return (
                (float(detection['x_min']) + float(detection['x_max'])) * 0.5,
                (float(detection['y_min']) + float(detection['y_max'])) * 0.5,
            )
        bbox = detection.get('bbox') or detection.get('box')
        if isinstance(bbox, dict) and all(key in bbox for key in keys):
            return (
                (float(bbox['x_min']) + float(bbox['x_max'])) * 0.5,
                (float(bbox['y_min']) + float(bbox['y_max'])) * 0.5,
            )
        if isinstance(bbox, dict) and all(key in bbox for key in ('x', 'y', 'w', 'h')):
            return float(bbox['x']) + float(bbox['w']) * 0.5, float(bbox['y']) + float(bbox['h']) * 0.5
        if isinstance(bbox, (list, tuple)) and len(bbox) >= 4:
            x0, y0, x1, y1 = [float(v) for v in bbox[:4]]
            return (x0 + x1) * 0.5, (y0 + y1) * 0.5
        self.get_logger().warn('detection has no usable center/bbox')
        return None

    def _pixel_to_plane(self, u: float, v: float) -> tuple[float, float]:
        h = self.homography
        denom = h[6] * u + h[7] * v + h[8]
        if abs(denom) < 1e-9:
            raise ValueError('homography denominator is near zero')
        x = (h[0] * u + h[1] * v + h[2]) / denom
        y = (h[3] * u + h[4] * v + h[5]) / denom
        return x, y


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObjectPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
