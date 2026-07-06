#!/usr/bin/env python3
"Arm trajectory controller that does interpolation and soft-limit checks."

from __future__ import annotations

import json
import queue
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from my_arm_control.trajectory_interp import interpolate_segment, normalize_trajectory


def seconds_to_duration(seconds: float):
    seconds = max(0.0, float(seconds))
    sec = int(seconds)
    nanosec = int(round((seconds - sec) * 1e9))
    if nanosec >= 1_000_000_000:
        sec += 1
        nanosec -= 1_000_000_000
    return sec, nanosec


class ArmControllerNode(Node):
    "Convert a coarse arm trajectory into stepwise target commands."

    def __init__(self):
        super().__init__('arm_controller')

        self.declare_parameter('joint_names', ['arm_joint_1', 'arm_joint_2', 'arm_joint_3', 'arm_joint_4', 'arm_joint_5', 'arm_joint_6'])
        self.declare_parameter('home_positions', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('lower_limits', [-2.80, -1.80, -2.20, -2.60, -3.14, -3.14])
        self.declare_parameter('upper_limits', [2.80, 1.80, 2.20, 2.60, 3.14, 3.14])
        self.declare_parameter('trajectory_topic', '/arm/joint_trajectory')
        self.declare_parameter('target_topic', '/arm/joint_target')
        self.declare_parameter('status_topic', '/arm/controller_status')
        self.declare_parameter('default_move_time_s', 2.0)
        self.declare_parameter('interpolation_step_s', 0.1)

        self.joint_names = list(self.get_parameter('joint_names').value)
        self.home_positions = [float(value) for value in self.get_parameter('home_positions').value]
        self.lower_limits = [float(value) for value in self.get_parameter('lower_limits').value]
        self.upper_limits = [float(value) for value in self.get_parameter('upper_limits').value]
        self.default_move_time_s = float(self.get_parameter('default_move_time_s').value)
        self.interpolation_step_s = float(self.get_parameter('interpolation_step_s').value)

        if not self.joint_names:
            raise ValueError('joint_names must not be empty')
        if not (len(self.joint_names) == len(self.home_positions) == len(self.lower_limits) == len(self.upper_limits)):
            raise ValueError('joint configuration arrays must have the same length')

        self._lock = threading.Lock()
        self._current_positions = list(self.home_positions)
        self._goal_queue: queue.Queue[tuple[int, float, list]] = queue.Queue(maxsize=1)
        self._stop_event = threading.Event()
        self._next_command_id = 1
        self._active_command_id: int | None = None

        trajectory_topic = self.get_parameter('trajectory_topic').value
        target_topic = self.get_parameter('target_topic').value
        status_topic = self.get_parameter('status_topic').value

        self.target_pub = self.create_publisher(JointTrajectory, target_topic, 10)
        self.status_pub = self.create_publisher(String, status_topic, 10)
        self.create_subscription(JointTrajectory, trajectory_topic, self._on_trajectory, 10)

        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()
        self._publish_status('idle', 'arm controller ready')

    def _publish_status(
        self,
        state: str,
        message: str,
        *,
        command_id: int = 0,
        expected_duration_s: float = 0.0,
    ):
        payload = {
            'state': state,
            'message': message,
            'joint_names': self.joint_names,
            'positions': list(self._current_positions),
            'timestamp_ns': self.get_clock().now().nanoseconds,
            'command_id': int(command_id),
            'expected_duration_s': float(expected_duration_s),
        }
        if self._active_command_id is not None:
            payload['active_command_id'] = int(self._active_command_id)
        self.status_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    def _on_trajectory(self, msg: JointTrajectory):
        try:
            segments = normalize_trajectory(msg, self.joint_names, self.default_move_time_s)
            for segment in segments:
                self._validate_positions(segment.positions)
        except Exception as exc:
            self.get_logger().error(f'reject trajectory: {exc}')
            self._publish_status('rejected', str(exc))
            return

        command_id = self._next_command_id
        self._next_command_id += 1
        expected_duration_s = sum(float(segment.duration_s) for segment in segments)

        while True:
            try:
                self._goal_queue.get_nowait()
            except queue.Empty:
                break
        self._goal_queue.put((command_id, expected_duration_s, segments))
        self._publish_status(
            'accepted',
            f'received {len(segments)} trajectory point(s)',
            command_id=command_id,
            expected_duration_s=expected_duration_s,
        )

    def _validate_positions(self, positions: list[float]):
        for joint_name, value, lower, upper in zip(self.joint_names, positions, self.lower_limits, self.upper_limits):
            if value < lower or value > upper:
                raise ValueError(f'{joint_name}={value:.3f} exceeds soft limit [{lower:.3f}, {upper:.3f}]')

    def _worker_loop(self):
        while not self._stop_event.is_set():
            try:
                command_id, expected_duration_s, segments = self._goal_queue.get(timeout=0.1)
            except queue.Empty:
                continue

            self._active_command_id = command_id
            self._publish_status(
                'executing',
                'trajectory execution started',
                command_id=command_id,
                expected_duration_s=expected_duration_s,
            )
            with self._lock:
                current = list(self._current_positions)

            try:
                for segment in segments:
                    frames, dt = interpolate_segment(current, segment.positions, segment.duration_s, self.interpolation_step_s)
                    for frame in frames:
                        if self._stop_event.is_set():
                            return
                        self._publish_target(frame, dt)
                        with self._lock:
                            self._current_positions = list(frame)
                        time.sleep(dt)
                    current = list(segment.positions)
                self._publish_status(
                    'completed',
                    'trajectory execution finished',
                    command_id=command_id,
                    expected_duration_s=expected_duration_s,
                )
            except Exception as exc:
                self._publish_status(
                    'failed',
                    f'trajectory execution failed: {exc}',
                    command_id=command_id,
                    expected_duration_s=expected_duration_s,
                )
            finally:
                if self._active_command_id == command_id:
                    self._active_command_id = None

    def _publish_target(self, positions: list[float], duration_s: float):
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = list(self.joint_names)
        point = JointTrajectoryPoint()
        point.positions = list(positions)
        sec, nanosec = seconds_to_duration(duration_s)
        point.time_from_start.sec = sec
        point.time_from_start.nanosec = nanosec
        msg.points = [point]
        self.target_pub.publish(msg)

    def destroy_node(self):
        self._stop_event.set()
        if self._worker.is_alive():
            self._worker.join(timeout=1.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArmControllerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

