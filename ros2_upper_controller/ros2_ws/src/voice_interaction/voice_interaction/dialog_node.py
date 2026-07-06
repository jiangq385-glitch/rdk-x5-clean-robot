import json
import queue
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .config import (
    DEFAULT_API_KEY,
    DEFAULT_ASR_TOPIC,
    DEFAULT_ENABLE_FAST_COMMANDS,
    DEFAULT_ENABLE_INTENT_ROUTING,
    DEFAULT_ENABLE_VISION,
    DEFAULT_IMAGE_PATH,
    DEFAULT_LLM_MODEL,
    DEFAULT_MAX_RULE_TEXT_LENGTH,
    DEFAULT_NUM_CTX,
    DEFAULT_NUM_PREDICT,
    DEFAULT_SYSTEM_PROMPT,
    DEFAULT_TEMPERATURE,
    DEFAULT_TIMEOUT_SEC,
    DEFAULT_TTS_TOPIC,
    DEFAULT_VOICE_STATE_TOPIC,
    DEFAULT_WAKEUP_TOPIC,
    DEFAULT_WAKE_ACK_TEXT,
    VOICE_STATE_SLEEPING,
    VOICE_STATE_SPEAKING,
    VOICE_STATE_THINKING,
)
from .dialog_manager import DialogManager
from .llm_client import CloudAPIClient


PARAM_DEFAULTS = {
    'asr_topic': DEFAULT_ASR_TOPIC,
    'tts_topic': DEFAULT_TTS_TOPIC,
    'voice_state_topic': DEFAULT_VOICE_STATE_TOPIC,
    'wakeup_topic': DEFAULT_WAKEUP_TOPIC,
    'wake_ack_text': DEFAULT_WAKE_ACK_TEXT,
    'api_base_url': '',
    'llm_model': DEFAULT_LLM_MODEL,
    'api_key': DEFAULT_API_KEY,
    'system_prompt': DEFAULT_SYSTEM_PROMPT,
    'request_timeout_sec': DEFAULT_TIMEOUT_SEC,
    'num_predict': DEFAULT_NUM_PREDICT,
    'num_ctx': DEFAULT_NUM_CTX,
    'temperature': DEFAULT_TEMPERATURE,
    'enable_fast_commands': DEFAULT_ENABLE_FAST_COMMANDS,
    'enable_intent_routing': DEFAULT_ENABLE_INTENT_ROUTING,
    'max_rule_text_length': DEFAULT_MAX_RULE_TEXT_LENGTH,
    'enable_vision': DEFAULT_ENABLE_VISION,
    'image_path': DEFAULT_IMAGE_PATH,
    'clean_area_command_topic': '/robot_test/clean_area_command',
    'enable_clean_area_voice_routing': True,
}


def _string_msg(text: str) -> String:
    msg = String()
    msg.data = text
    return msg


class DialogNode(Node):
    def __init__(self):
        super().__init__('dialog_node')

        for name, default in PARAM_DEFAULTS.items():
            self.declare_parameter(name, default)

        asr_topic = self.get_parameter('asr_topic').value
        self.tts_topic = self.get_parameter('tts_topic').value
        self.voice_state_topic = self.get_parameter('voice_state_topic').value
        self.command_topic = self.get_parameter('wakeup_topic').value
        self.clean_area_command_topic = self.get_parameter('clean_area_command_topic').value
        self.enable_clean_area_voice_routing = bool(self.get_parameter('enable_clean_area_voice_routing').value)
        self.wake_ack_text = self.get_parameter('wake_ack_text').value.strip()
        api_base_url = self.get_parameter('api_base_url').value
        llm_model = self.get_parameter('llm_model').value
        api_key = self.get_parameter('api_key').value
        system_prompt = self.get_parameter('system_prompt').value
        timeout_sec = float(self.get_parameter('request_timeout_sec').value)
        num_predict = int(self.get_parameter('num_predict').value)
        num_ctx = int(self.get_parameter('num_ctx').value)
        temperature = float(self.get_parameter('temperature').value)
        enable_fast_commands = bool(self.get_parameter('enable_fast_commands').value)
        enable_intent_routing = bool(self.get_parameter('enable_intent_routing').value)
        max_rule_text_length = int(self.get_parameter('max_rule_text_length').value)
        enable_vision = bool(self.get_parameter('enable_vision').value)
        image_path = self.get_parameter('image_path').value

        if self.tts_topic == self.voice_state_topic:
            raise ValueError('tts_topic and voice_state_topic must be different')
        if self.command_topic == self.voice_state_topic:
            raise ValueError('wakeup_topic and voice_state_topic must be different')
        if timeout_sec <= 0.0:
            raise ValueError('request_timeout_sec must be positive')
        if num_predict <= 0:
            raise ValueError('num_predict must be positive')
        if num_ctx <= 0:
            raise ValueError('num_ctx must be positive')
        if max_rule_text_length <= 0:
            raise ValueError('max_rule_text_length must be positive')
        if not llm_model:
            raise ValueError('llm_model must not be empty')
        if not api_base_url:
            raise ValueError('api_base_url must not be empty')
        if not api_key:
            raise ValueError('api_key must not be empty')

        self.tts_pub = self.create_publisher(String, self.tts_topic, 10)
        self.voice_state_pub = self.create_publisher(String, self.voice_state_topic, 10)
        self.clean_area_pub = self.create_publisher(String, self.clean_area_command_topic, 10)
        self.command_sub = self.create_subscription(String, self.command_topic, self._command_callback, 10)
        self.tts_done_sub = self.create_subscription(String, '/tts_done', self._on_tts_done, 10)
        self.queue = queue.Queue(maxsize=8)
        self._tts_pending_lock = threading.Lock()
        self._pending_dialog_tts = 0

        llm_client = CloudAPIClient(
            api_base_url=api_base_url,
            api_key=api_key,
            model=llm_model,
            timeout_sec=timeout_sec,
            num_predict=num_predict,
            temperature=temperature,
            enable_vision=enable_vision,
            image_path=image_path,
        )

        self.dialog_manager = DialogManager(
            llm_client=llm_client,
            system_prompt=system_prompt,
            enable_fast_commands=enable_fast_commands,
            enable_intent_routing=enable_intent_routing,
            max_rule_text_length=max_rule_text_length,
        )

        self.subscription = self.create_subscription(String, asr_topic, self._asr_callback, 10)
        self.worker = threading.Thread(target=self._worker_loop, daemon=True)
        self.worker.start()

        self._publish_state(VOICE_STATE_SLEEPING)
        self.get_logger().info(
            f'Dialog node started with asr_topic={asr_topic}, tts_topic={self.tts_topic}, '
            f'voice_state_topic={self.voice_state_topic}, command_topic={self.command_topic}, '
            f'model={llm_model}, '
            f'api_base_url={api_base_url}, timeout_sec={timeout_sec}, num_predict={num_predict}, '
            f'num_ctx={num_ctx}, temperature={temperature}, enable_fast_commands={enable_fast_commands}, '
            f'enable_intent_routing={enable_intent_routing}, max_rule_text_length={max_rule_text_length}, '
            f'enable_vision={enable_vision}, image_path={image_path}, '
            f'clean_area_command_topic={self.clean_area_command_topic}, '
            f'enable_clean_area_voice_routing={self.enable_clean_area_voice_routing}'
        )

    def _publish_state(self, state: str):
        self.voice_state_pub.publish(_string_msg(state))

    def _publish_tts(self, text: str):
        self.tts_pub.publish(_string_msg(text))

    def _command_callback(self, msg: String):
        if not self.wake_ack_text:
            return
        if msg.data.strip().lower() != 'start':
            self.get_logger().info(f'Wakeup ack ignored with unsupported command: {msg.data}')
            return

        self._publish_tts(self.wake_ack_text)
        self.get_logger().info(f'Wakeup ack queued: {msg.data} -> {self.wake_ack_text}')

    def _on_tts_done(self, msg: String):
        if msg.data.strip() != 'tts_done':
            return

        should_sleep = False
        with self._tts_pending_lock:
            if self._pending_dialog_tts > 0:
                self._pending_dialog_tts -= 1
                should_sleep = self._pending_dialog_tts == 0

        if not should_sleep:
            return

        self._publish_state(VOICE_STATE_SLEEPING)
        self.get_logger().info('Dialog TTS done, returning to sleep')

    def _asr_callback(self, msg: String):
        user_text = msg.data.strip()
        if not user_text:
            return

        if self.queue.full():
            self.get_logger().warning('Dialog queue full, dropping oldest request')
            try:
                self.queue.get_nowait()
                self.queue.task_done()
            except queue.Empty:
                pass

        self.queue.put_nowait(user_text)
        self._publish_state(VOICE_STATE_THINKING)
        self.get_logger().info(f'ASR final received: {user_text}')

    def _try_publish_clean_area_task(self, user_text: str) -> str:
        if not self.enable_clean_area_voice_routing:
            return ''

        text = ''.join(user_text.strip().split())
        if not text:
            return ''

        clean_keywords = (
            '\u6e05\u6d01',  # ??
            '\u6253\u626b',  # ??
            '\u6e05\u7406',  # ??
            '\u6536\u62fe',  # ??
            '\u6574\u7406',  # ??
        )
        if not any(keyword in text for keyword in clean_keywords):
            return ''

        area = ''
        area_name = ''
        if any(keyword in text for keyword in ('\u5c0f\u684c\u5b50', '\u5c0f\u684c', '\u5c0f\u684c\u9762')):
            area = 'small_table'
            area_name = '\u5c0f\u684c\u5b50'
        elif any(keyword in text for keyword in ('\u5927\u684c\u5b50', '\u5927\u684c', '\u684c\u5b50', '\u684c\u9762')):
            area = 'big_table'
            area_name = '\u5927\u684c\u5b50'
        elif any(keyword in text for keyword in ('\u5783\u573e\u6876', '\u5783\u573e\u7bb1')):
            area = 'trash_bin'
            area_name = '\u5783\u573e\u6876'

        if not area:
            return ''

        payload = {
            'command_id': f'voice_clean_{int(time.time() * 1000)}',
            'source': 'voice_interaction',
            'type': 'clean_area',
            'area': area,
            'area_name': area_name,
            'utterance': user_text,
            'ts': int(time.time() * 1000),
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.clean_area_pub.publish(msg)
        self.get_logger().info(f'Voice clean task published: {payload}')
        return f'\u597d\u7684\uff0c\u5f00\u59cb\u6e05\u6d01{area_name}\u3002'

    def _worker_loop(self):
        while rclpy.ok():
            try:
                user_text = self.queue.get(timeout=0.5)
            except queue.Empty:
                continue

            started_at = time.perf_counter()
            try:
                clean_task_reply = self._try_publish_clean_area_task(user_text)
                if clean_task_reply:
                    elapsed_sec = time.perf_counter() - started_at
                    with self._tts_pending_lock:
                        self._pending_dialog_tts += 1
                    self._publish_state(VOICE_STATE_SPEAKING)
                    self._publish_tts(clean_task_reply)
                    self.get_logger().info(
                        f'Dialog reply source=voice_clean_task elapsed={elapsed_sec:.2f}s text={clean_task_reply}'
                    )
                    continue

                route_result = self.dialog_manager.reply(user_text)
                elapsed_sec = time.perf_counter() - started_at
                with self._tts_pending_lock:
                    self._pending_dialog_tts += 1
                self._publish_state(VOICE_STATE_SPEAKING)
                self._publish_tts(route_result.reply)
                self.get_logger().info(
                    f'Dialog reply source={route_result.source} elapsed={elapsed_sec:.2f}s text={route_result.reply}'
                )
            except Exception as e:
                elapsed_sec = time.perf_counter() - started_at
                self._publish_state(VOICE_STATE_SLEEPING)
                self.get_logger().error(f'Dialog failed after {elapsed_sec:.2f}s: {e}')
            finally:
                self.queue.task_done()

def main(args=None):
    rclpy.init(args=args)
    node = DialogNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
