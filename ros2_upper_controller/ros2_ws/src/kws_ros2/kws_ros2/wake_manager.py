import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .config import (
    DEFAULT_TAKEOVER_TOPIC,
    DEFAULT_TAKEOVER_WINDOW_SEC,
    DEFAULT_VOICE_IDLE_TIMEOUT_SEC,
    DEFAULT_VOICE_STATE_TOPIC,
    DEFAULT_WAKEUP_TOPIC,
    VOICE_STATE_SLEEPING,
    is_voice_active_state,
)


MANAGER_STATE_SLEEPING = VOICE_STATE_SLEEPING
MANAGER_STATE_TAKEOVER = 'takeover'
MANAGER_STATE_ACTIVE = 'active'

PARAM_DEFAULTS = {
    'wakeup_topic': DEFAULT_WAKEUP_TOPIC,
    'voice_state_topic': DEFAULT_VOICE_STATE_TOPIC,
    'takeover_topic': DEFAULT_TAKEOVER_TOPIC,
    'takeover_window_sec': DEFAULT_TAKEOVER_WINDOW_SEC,
    'idle_timeout_sec': DEFAULT_VOICE_IDLE_TIMEOUT_SEC,
}


def _string_msg(text: str) -> String:
    msg = String()
    msg.data = text
    return msg


class WakeManager(Node):
    def __init__(self):
        super().__init__('wake_manager')

        for name, default in PARAM_DEFAULTS.items():
            self.declare_parameter(name, default)

        self.wakeup_topic = self.get_parameter('wakeup_topic').value
        self.voice_state_topic = self.get_parameter('voice_state_topic').value
        self.takeover_topic = self.get_parameter('takeover_topic').value
        self.takeover_window_sec = float(self.get_parameter('takeover_window_sec').value)
        self.idle_timeout_sec = float(self.get_parameter('idle_timeout_sec').value)

        if self.takeover_window_sec <= 0.0:
            raise ValueError('takeover_window_sec must be positive')
        if self.idle_timeout_sec <= 0.0:
            raise ValueError('idle_timeout_sec must be positive')

        self.voice_state_pub = self.create_publisher(String, self.voice_state_topic, 10)
        self.takeover_pub = self.create_publisher(String, self.takeover_topic, 10)
        self.wakeup_sub = self.create_subscription(String, self.wakeup_topic, self._on_wakeup, 10)
        self.voice_state_sub = self.create_subscription(String, self.voice_state_topic, self._on_voice_state, 10)
        self.timer = self.create_timer(0.1, self._tick)

        self._state = MANAGER_STATE_SLEEPING
        self._takeover_started_at = 0.0

        self._publish_state(VOICE_STATE_SLEEPING)
        self.get_logger().info(
            f'Wake manager started. wakeup_topic={self.wakeup_topic}, takeover_topic={self.takeover_topic}, '
            f'voice_state_topic={self.voice_state_topic}, takeover_window_sec={self.takeover_window_sec}, '
            f'idle_timeout_sec={self.idle_timeout_sec}'
        )

    def _publish_state(self, state: str):
        self.voice_state_pub.publish(_string_msg(state))

    def _publish_takeover(self, command: str):
        self.takeover_pub.publish(_string_msg(command))

    def _on_wakeup(self, msg: String):
        wakeup_text = msg.data.strip()
        if not wakeup_text:
            self.get_logger().info('Wakeup ignored with empty payload')
            return

        if self._state != MANAGER_STATE_SLEEPING:
            self.get_logger().info(f'Wakeup ignored while {self._state}: {wakeup_text}')
            return

        self._state = MANAGER_STATE_TAKEOVER
        self._takeover_started_at = time.monotonic()
        self._publish_state('acking')
        self._publish_takeover('start')
        self.get_logger().info(f'Wakeup accepted: {wakeup_text}')

    def _on_voice_state(self, msg: String):
        state = msg.data.strip()
        if is_voice_active_state(state):
            if self._state in (MANAGER_STATE_TAKEOVER, MANAGER_STATE_ACTIVE):
                self._state = MANAGER_STATE_ACTIVE
                self._last_activity_at = time.monotonic()
            return

        if state == VOICE_STATE_SLEEPING:
            if self._state != MANAGER_STATE_SLEEPING:
                self.get_logger().info('Session returned to sleep')
            self._state = MANAGER_STATE_SLEEPING
            self._takeover_started_at = 0.0

    def _tick(self):
        if self._state == MANAGER_STATE_SLEEPING:
            return

        if self._state == MANAGER_STATE_TAKEOVER:
            if time.monotonic() - self._takeover_started_at > self.takeover_window_sec:
                self.get_logger().info('Takeover timeout, back to sleep')
                self._publish_takeover('cancel')
                self._publish_state(VOICE_STATE_SLEEPING)
                self._state = MANAGER_STATE_SLEEPING
                self._takeover_started_at = 0.0
            return

        if self._state == MANAGER_STATE_ACTIVE:
            if time.monotonic() - self._last_activity_at > self.idle_timeout_sec:
                self.get_logger().info('Voice session idle timeout, back to sleep')
                self._publish_state(VOICE_STATE_SLEEPING)
                self._state = MANAGER_STATE_SLEEPING
                self._takeover_started_at = 0.0


def main(args=None):
    rclpy.init(args=args)
    node = WakeManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
