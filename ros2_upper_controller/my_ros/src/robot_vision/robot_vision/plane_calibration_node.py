from __future__ import annotations

import json
import math
import os
import tempfile

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import String

from robot_msgs.action import CalibratePlaneHeight
from robot_msgs.msg import PlaneHeight


class PlaneCalibrationNode(Node):
    "Calibrate and publish the work-plane height from a fixed laser probe pose."

    def __init__(self) -> None:
        super().__init__('plane_calibration_node')
        self.declare_parameter('laser_range_topic', '/perception/laser_range')
        self.declare_parameter('plane_height_topic', '/perception/plane_height')
        self.declare_parameter('status_topic', '/perception/plane_calibration_status')
        self.declare_parameter('calibration_file', '/home/sunrise/my_ros/src/robot_vision/config/plane_height_calibration.json')
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('z_sensor_in_base', 0.12)
        self.declare_parameter('laser_tilt_rad', 0.0)
        self.declare_parameter('min_valid_range_m', 0.02)
        self.declare_parameter('max_valid_range_m', 2.0)
        self.declare_parameter('auto_calibrate_on_first_range', False)
        self.calibration_file = str(self.get_parameter('calibration_file').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.z_sensor_in_base = float(self.get_parameter('z_sensor_in_base').value)
        self.laser_tilt_rad = float(self.get_parameter('laser_tilt_rad').value)
        self.min_valid_range_m = float(self.get_parameter('min_valid_range_m').value)
        self.max_valid_range_m = float(self.get_parameter('max_valid_range_m').value)
        self.auto_calibrate_on_first_range = bool(self.get_parameter('auto_calibrate_on_first_range').value)
        self.latest_range: Range | None = None
        self.plane_height = PlaneHeight()
        self.plane_height.valid = False
        self.plane_height.frame_id = self.frame_id
        self._load_calibration()
        self.plane_pub = self.create_publisher(PlaneHeight, self.get_parameter('plane_height_topic').value, 10)
        self.status_pub = self.create_publisher(String, self.get_parameter('status_topic').value, 10)
        self.create_subscription(Range, self.get_parameter('laser_range_topic').value, self.on_range, 10)
        self.action_server = ActionServer(self, CalibratePlaneHeight, '/calibrate_plane_height', self.execute_callback)
        self.create_timer(1.0, self.publish_plane_height)

    def on_range(self, msg: Range) -> None:
        self.latest_range = msg
        if self.auto_calibrate_on_first_range and not self.plane_height.valid:
            self._calibrate_from_range(float(msg.range))

    async def execute_callback(self, goal_handle):
        feedback = CalibratePlaneHeight.Feedback()
        feedback.current_phase = 'read_laser_range'
        feedback.progress = 0.4
        goal_handle.publish_feedback(feedback)
        result = CalibratePlaneHeight.Result()
        if self.latest_range is None:
            goal_handle.abort()
            result.success = False
            result.failed_reason = 'no laser range has been received'
            result.plane_height = self.plane_height
            return result
        try:
            self._calibrate_from_range(float(self.latest_range.range))
        except ValueError as exc:
            goal_handle.abort()
            result.success = False
            result.failed_reason = str(exc)
            result.plane_height = self.plane_height
            return result
        feedback.current_phase = 'publish_plane_height'
        feedback.progress = 1.0
        goal_handle.publish_feedback(feedback)
        goal_handle.succeed()
        result.success = True
        result.failed_reason = ''
        result.plane_height = self.plane_height
        return result

    def _calibrate_from_range(self, measured_range_m: float) -> None:
        if not math.isfinite(measured_range_m):
            raise ValueError('laser range is not finite')
        if measured_range_m < self.min_valid_range_m or measured_range_m > self.max_valid_range_m:
            raise ValueError(f'laser range {measured_range_m:.3f} outside valid range')
        z_plane = self.z_sensor_in_base - measured_range_m * math.cos(self.laser_tilt_rad)
        self.plane_height.valid = True
        self.plane_height.z_plane_in_base = float(z_plane)
        self.plane_height.measured_range_m = float(measured_range_m)
        self.plane_height.frame_id = self.frame_id
        self._save_calibration()
        self.publish_plane_height()
        self._publish_status('calibrated', f'z_plane_in_base={z_plane:.4f}, range={measured_range_m:.4f}')

    def publish_plane_height(self) -> None:
        self.plane_pub.publish(self.plane_height)

    def _load_calibration(self) -> None:
        if not os.path.exists(self.calibration_file):
            return
        try:
            with open(self.calibration_file, 'r', encoding='utf-8') as file_obj:
                data = json.load(file_obj)
            self.plane_height.valid = bool(data.get('valid', False))
            self.plane_height.z_plane_in_base = float(data.get('z_plane_in_base', 0.0))
            self.plane_height.measured_range_m = float(data.get('measured_range_m', 0.0))
            self.plane_height.frame_id = str(data.get('frame_id', self.frame_id))
        except Exception as exc:
            self.get_logger().warn(f'failed to load plane calibration: {exc}')

    def _save_calibration(self) -> None:
        os.makedirs(os.path.dirname(self.calibration_file), exist_ok=True)
        payload = {
            'valid': bool(self.plane_height.valid),
            'z_plane_in_base': float(self.plane_height.z_plane_in_base),
            'measured_range_m': float(self.plane_height.measured_range_m),
            'frame_id': self.plane_height.frame_id,
            'timestamp_ns': self.get_clock().now().nanoseconds,
        }
        fd, tmp_path = tempfile.mkstemp(prefix='.plane_height_', suffix='.json', dir=os.path.dirname(self.calibration_file))
        try:
            with os.fdopen(fd, 'w', encoding='utf-8') as file_obj:
                json.dump(payload, file_obj, ensure_ascii=False, indent=2)
                file_obj.write('\n')
            os.replace(tmp_path, self.calibration_file)
        finally:
            if os.path.exists(tmp_path):
                os.unlink(tmp_path)

    def _publish_status(self, state: str, message: str) -> None:
        payload = {'state': state, 'message': message, 'timestamp_ns': self.get_clock().now().nanoseconds}
        self.status_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PlaneCalibrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
