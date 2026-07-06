from __future__ import annotations

import json
import math
import os
import sys
import threading
import time
from pathlib import Path
from typing import Any

import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String

VOICE_INTERACTION_SRC = '/home/sunrise/ros2_ws/src/voice_interaction'
if VOICE_INTERACTION_SRC not in sys.path:
    sys.path.insert(0, VOICE_INTERACTION_SRC)

try:
    from voice_interaction.llm_client import CloudAPIClient
    from voice_interaction.vision_llm_test import DEFAULT_API_BASE_URL, DEFAULT_API_KEY, DEFAULT_MODEL
except Exception:  # pragma: no cover
    CloudAPIClient = None
    DEFAULT_API_BASE_URL = 'https://ark.cn-beijing.volces.com/api/v3'
    DEFAULT_API_KEY = os.environ.get('LLM_API_KEY', '')
    DEFAULT_MODEL = os.environ.get('LLM_MODEL', 'doubao-seed-2-0-mini-260428')


GOALS = {
    'big_table': (-2.58508, -1.1317, 3.04223),
    'trash_bin': (-2.61424, -0.782089, 2.96889),
    'small_table': (-1.85905, 0.254937, 2.3897),
}

AREA_ALIASES = {
    'big_table': 'big_table',
    'large_table': 'big_table',
    '???': 'big_table',
    '??': 'big_table',
    'big table': 'big_table',
    'trash_bin': 'trash_bin',
    'trash': 'trash_bin',
    '???': 'trash_bin',
    'small_table': 'small_table',
    '???': 'small_table',
    '??': 'small_table',
    'small table': 'small_table',
}

AREA_LABELS = {
    'big_table': '???',
    'trash_bin': '???',
    'small_table': '???',
}


class CleanTableTestNode(Node):
    def __init__(self) -> None:
        super().__init__('clean_table_test_node')
        self.declare_parameter('command_topic', '/robot_test/clean_area_command')
        self.declare_parameter('status_topic', '/robot_status')
        self.declare_parameter('detections_topic', '/vision/detections_json')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('navigate_action', '/navigate_to_pose')
        self.declare_parameter('nav_server_wait_sec', 180.0)
        self.declare_parameter('skip_vision', False)
        self.declare_parameter('yolo_wait_sec', 3.0)
        self.declare_parameter('navigation_timeout_sec', 180.0)
        self.declare_parameter('min_detection_score', 0.35)
        self.declare_parameter('llm_output_dir', '/home/sunrise/robot_test_frames')
        self.declare_parameter('llm_api_base_url', os.environ.get('LLM_API_BASE_URL', DEFAULT_API_BASE_URL))
        self.declare_parameter('llm_api_key', os.environ.get('LLM_API_KEY', DEFAULT_API_KEY))
        self.declare_parameter('llm_model', os.environ.get('LLM_MODEL', DEFAULT_MODEL))
        self.declare_parameter('llm_timeout_sec', 90.0)
        self.declare_parameter('llm_max_tokens', 256)
        self.declare_parameter('llm_temperature', 0.2)
        self.declare_parameter('llm_system_prompt', '????????????')
        self.declare_parameter('llm_prompt', '??????????USB?????????????????????????')
        self.declare_parameter('auto_start_area', '')

        self.command_topic = str(self.get_parameter('command_topic').value)
        self.status_topic = str(self.get_parameter('status_topic').value)
        self.detections_topic = str(self.get_parameter('detections_topic').value)
        self.image_topic = str(self.get_parameter('image_topic').value)
        self.yolo_wait_sec = float(self.get_parameter('yolo_wait_sec').value)
        self.navigation_timeout_sec = float(self.get_parameter('navigation_timeout_sec').value)
        self.min_detection_score = float(self.get_parameter('min_detection_score').value)
        self.nav_server_wait_sec = float(self.get_parameter('nav_server_wait_sec').value)
        self.skip_vision = bool(self.get_parameter('skip_vision').value)

        self.bridge = CvBridge()
        self.latest_image_msg: Image | None = None
        self.latest_yolo_payload: dict[str, Any] | None = None
        self.task_running = False
        self.active_area = ''
        self.yolo_active = False
        self.yolo_started_at = 0.0
        self.nav_started_at = 0.0
        self.llm_thread: threading.Thread | None = None
        self.auto_start_used = False

        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.nav_client = ActionClient(self, NavigateToPose, str(self.get_parameter('navigate_action').value))
        self.create_subscription(String, self.command_topic, self.on_command, 10)
        self.create_subscription(String, self.detections_topic, self.on_detections, 10)
        image_qos = QoSProfile(depth=1)
        image_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(Image, self.image_topic, self.on_image, image_qos)
        self.create_timer(0.2, self._poll_yolo_result)

        self.get_logger().info(
            f'robot_test ready: command={self.command_topic}, image={self.image_topic}, yolo={self.detections_topic}'
        )
        self.auto_start_area = str(self.get_parameter('auto_start_area').value).strip()
        if self.auto_start_area:
            self.create_timer(1.0, self._auto_start_tick)

    def _auto_start_tick(self) -> None:
        if self.auto_start_used:
            return
        self.auto_start_used = True
        self.start_task(self.auto_start_area)

    def on_command(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            payload = {'text': msg.data}
        area_text = self._area_text_from_payload(payload)
        self.get_logger().info(f'received clean test command: {payload}')
        self.start_task(area_text, command_payload=payload)

    def on_image(self, msg: Image) -> None:
        self.latest_image_msg = msg

    def on_detections(self, msg: String) -> None:
        if not self.yolo_active:
            return
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f'invalid YOLO JSON: {exc}')
            return
        self.latest_yolo_payload = payload

    def start_task(self, area_text: str, command_payload: dict[str, Any] | None = None) -> None:
        area = self._normalize_area(area_text)
        if area not in GOALS:
            self._publish_status('rejected', 0.0, f'??????: {area_text}', {'area_text': area_text})
            return
        if self.task_running:
            self._publish_status('busy', 0.0, f'???????: {AREA_LABELS.get(self.active_area, self.active_area)}')
            return
        self.task_running = True
        self.active_area = area
        self.latest_yolo_payload = None
        self._print(f'????????: {AREA_LABELS[area]} ({area})')
        self._publish_status('accepted', 0.05, f'?????????: {AREA_LABELS[area]}', {'area': area, 'command': command_payload or {}})
        self._start_navigation(area)

    def _start_navigation(self, area: str) -> None:
        self._publish_status('navigating', 0.15, f'?????{AREA_LABELS[area]}', {'area': area})
        self._print(f'????: {AREA_LABELS[area]} -> {GOALS[area]}')
        self._print(f'?? /navigate_to_pose action server??? {self.nav_server_wait_sec:.1f}s...')
        if not self.nav_client.wait_for_server(timeout_sec=self.nav_server_wait_sec):
            self._fail_task('/navigate_to_pose action server ??????? Nav2 ????????')
            return
        x, y, yaw = GOALS[area]
        goal = NavigateToPose.Goal()
        goal.pose = self._make_pose(x, y, yaw)
        self.nav_started_at = time.monotonic()
        future = self.nav_client.send_goal_async(goal, feedback_callback=self._on_nav_feedback)
        future.add_done_callback(self._on_nav_goal_response)

    def _on_nav_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._fail_task(f'????????: {exc}')
            return
        if not goal_handle.accepted:
            self._fail_task('???????')
            return
        self._publish_status('navigating', 0.25, '???????')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_nav_result)

    def _on_nav_feedback(self, _feedback_msg) -> None:
        if time.monotonic() - self.nav_started_at > self.navigation_timeout_sec:
            self.get_logger().warn('navigation timeout exceeded; waiting for Nav2 result callback')
            return
        self._publish_status('navigating', 0.35, f'????{AREA_LABELS.get(self.active_area, self.active_area)}')

    def _on_nav_result(self, future) -> None:
        try:
            wrapped = future.result()
        except Exception as exc:
            self._fail_task(f'??????: {exc}')
            return
        status = getattr(wrapped, 'status', None)
        if status != 4:
            self._fail_task(f'????????Nav2 status={status}')
            return
        area_label = AREA_LABELS.get(self.active_area, self.active_area)
        self._print(f'????: {area_label}')
        self._publish_status('arrived', 0.55, f'???{area_label}')
        if self.skip_vision:
            self._print('skip_vision=true????/????????')
            self._publish_status('done', 1.0, f'???{area_label}????????', {
                'area': self.active_area,
                'vision_skipped': True,
            })
            self._finish_task()
            return
        self._begin_yolo_check()

    def _begin_yolo_check(self) -> None:
        self.latest_yolo_payload = None
        self.yolo_active = True
        self.yolo_started_at = time.monotonic()
        self._print('?? YOLO ????? USB ???????...')
        self._publish_status('yolo_detecting', 0.68, '????YOLO??')

    def _poll_yolo_result(self) -> None:
        if not self.yolo_active:
            return
        detections = self._filtered_detections(self.latest_yolo_payload)
        if detections:
            self.yolo_active = False
            self._handle_yolo_success(detections)
            return
        if time.monotonic() - self.yolo_started_at >= self.yolo_wait_sec:
            self.yolo_active = False
            self._print(f'YOLO ? {self.yolo_wait_sec:.1f}s ???????????? Doubao ?? LLM?')
            self._publish_status('llm_describing', 0.78, 'YOLO???????????USB???Doubao??')
            self._start_llm_thread()

    def _handle_yolo_success(self, detections: list[dict[str, Any]]) -> None:
        lines = ['YOLO ?????:']
        status_detections = []
        for det in detections:
            bbox = det.get('bbox') or {}
            x = float(bbox.get('x', 0.0))
            y = float(bbox.get('y', 0.0))
            w = float(bbox.get('w', 0.0))
            h = float(bbox.get('h', 0.0))
            cx = x + w / 2.0
            cy = y + h / 2.0
            item = {
                'class_name': det.get('class_name', 'unknown'),
                'score': float(det.get('score', det.get('confidence', 0.0)) or 0.0),
                'bbox': {'x': x, 'y': y, 'w': w, 'h': h},
                'center': {'x': cx, 'y': cy},
            }
            status_detections.append(item)
            lines.append(
                f"- {item['class_name']} score={item['score']:.3f} "
                f"bbox=({x:.1f},{y:.1f},{w:.1f},{h:.1f}) center=({cx:.1f},{cy:.1f})"
            )
        self._print('\n'.join(lines))
        self._publish_status('done', 1.0, f'YOLO??????{len(status_detections)}???', {'area': self.active_area, 'detections': status_detections})
        self._finish_task()

    def _start_llm_thread(self) -> None:
        self.llm_thread = threading.Thread(target=self._run_llm_flow, daemon=True)
        self.llm_thread.start()

    def _run_llm_flow(self) -> None:
        try:
            image_path = self._save_latest_usb_frame()
            self._print(f'USB ??????: {image_path}')
            reply = self._ask_doubao(image_path)
            self._print('Doubao ?? LLM ????:')
            self._print(reply)
            self._publish_status('done', 1.0, 'Doubao??????', {'area': self.active_area, 'llm_reply': reply, 'image_path': str(image_path)})
        except Exception as exc:
            self._fail_task(f'Doubao??????: {exc}')
            return
        self._finish_task()

    def _save_latest_usb_frame(self) -> Path:
        msg = self.latest_image_msg
        if msg is None:
            raise RuntimeError(f'?????USB??: {self.image_topic}')
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        output_dir = Path(str(self.get_parameter('llm_output_dir').value)).expanduser()
        output_dir.mkdir(parents=True, exist_ok=True)
        path = output_dir / f'usb_frame_{int(time.time() * 1000)}.jpg'
        if not cv2.imwrite(str(path), frame):
            raise RuntimeError(f'??USB????: {path}')
        return path

    def _ask_doubao(self, image_path: Path) -> str:
        if CloudAPIClient is None:
            raise RuntimeError('???? voice_interaction.llm_client.CloudAPIClient')
        api_key = str(self.get_parameter('llm_api_key').value).strip()
        if not api_key:
            raise RuntimeError('?? Doubao API key???? LLM_API_KEY ? llm_api_key ??')
        client = CloudAPIClient(
            api_base_url=str(self.get_parameter('llm_api_base_url').value),
            api_key=api_key,
            model=str(self.get_parameter('llm_model').value),
            timeout_sec=float(self.get_parameter('llm_timeout_sec').value),
            num_predict=int(self.get_parameter('llm_max_tokens').value),
            temperature=float(self.get_parameter('llm_temperature').value),
            enable_vision=True,
            image_path=str(image_path),
        )
        return client.generate(str(self.get_parameter('llm_system_prompt').value), str(self.get_parameter('llm_prompt').value))

    def _filtered_detections(self, payload: dict[str, Any] | None) -> list[dict[str, Any]]:
        if not payload:
            return []
        raw = payload.get('detections')
        if not isinstance(raw, list):
            return []
        detections = []
        for det in raw:
            if not isinstance(det, dict):
                continue
            score = float(det.get('score', det.get('confidence', 0.0)) or 0.0)
            if score >= self.min_detection_score:
                detections.append(det)
        return detections

    def _make_pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.z = math.sin(float(yaw) / 2.0)
        pose.pose.orientation.w = math.cos(float(yaw) / 2.0)
        return pose

    def _area_text_from_payload(self, payload: Any) -> str:
        if not isinstance(payload, dict):
            return str(payload)
        for key in ('area', 'target_area', 'target', 'goal', 'room', 'text', 'utterance'):
            value = payload.get(key)
            if value:
                return str(value)
        nested = payload.get('payload')
        if isinstance(nested, dict):
            return self._area_text_from_payload(nested)
        rooms = payload.get('rooms')
        if isinstance(rooms, list) and rooms:
            return str(rooms[0])
        return ''

    def _normalize_area(self, text: str) -> str:
        value = str(text or '').strip()
        if value in AREA_ALIASES:
            return AREA_ALIASES[value]
        lowered = value.lower().replace('-', '_').replace(' ', '_')
        if lowered in AREA_ALIASES:
            return AREA_ALIASES[lowered]
        for alias, area in AREA_ALIASES.items():
            if alias and alias in value:
                return area
        return lowered

    def _publish_status(self, phase: str, progress: float, message: str, extra: dict[str, Any] | None = None) -> None:
        payload: dict[str, Any] = {
            'status': 'task_running' if phase not in ('done', 'failed', 'rejected', 'busy') else phase,
            'task_type': 'clean_area_test',
            'task_phase': phase,
            'task_progress': round(float(progress), 3),
            'progress': round(float(progress), 3),
            'message': message,
            'area': self.active_area,
            'ts': int(time.time() * 1000),
        }
        if extra:
            payload.update(extra)
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.status_pub.publish(msg)
        self.get_logger().info(message)

    def _fail_task(self, reason: str) -> None:
        self._print(f'??????: {reason}')
        self._publish_status('failed', 1.0, reason, {'failed_reason': reason})
        self._finish_task()

    def _finish_task(self) -> None:
        self.task_running = False
        self.yolo_active = False
        self.active_area = ''

    def _print(self, text: str) -> None:
        print(f'[robot_test] {text}', flush=True)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CleanTableTestNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
