import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .asr_engine import ASREngine
from .audio_capture import AudioCapture
from .config import (
    DEFAULT_ASR_API_BASE_URL,
    DEFAULT_ASR_API_KEY,
    DEFAULT_ASR_API_MODEL,
    DEFAULT_ASR_LANGUAGE,
    DEFAULT_ASR_MAX_UTTERANCE_MS,
    DEFAULT_ASR_MIN_SPEECH_MS,
    DEFAULT_ASR_PROMPT,
    DEFAULT_ASR_SILENCE_THRESHOLD,
    DEFAULT_ASR_TRAILING_SILENCE_MS,
    DEFAULT_CHANNELS,
    DEFAULT_INPUT_DEVICE,
    DEFAULT_MODE,
    DEFAULT_PROVIDER,
    DEFAULT_SAMPLE_RATE,
    DEFAULT_SAMPLES_PER_CHUNK,
    DEFAULT_SESSION_TIMEOUT_SEC,
    DEFAULT_TOPIC_FINAL,
    DEFAULT_TOPIC_PARTIAL,
    DEFAULT_VOICE_STATE_TOPIC,
    DEFAULT_WAKEUP_TOPIC,
    VOICE_STATE_LISTENING,
    VOICE_STATE_SLEEPING,
    VOICE_STATE_SPEAKING,
    VOICE_STATE_THINKING,
)


PARAM_DEFAULTS = {
    'input_device': DEFAULT_INPUT_DEVICE,
    'sample_rate': DEFAULT_SAMPLE_RATE,
    'channels': DEFAULT_CHANNELS,
    'samples_per_chunk': DEFAULT_SAMPLES_PER_CHUNK,
    'topic_final': DEFAULT_TOPIC_FINAL,
    'topic_partial': DEFAULT_TOPIC_PARTIAL,
    'provider': DEFAULT_PROVIDER,
    'mode': DEFAULT_MODE,
    'wakeup_topic': DEFAULT_WAKEUP_TOPIC,
    'voice_state_topic': DEFAULT_VOICE_STATE_TOPIC,
    'session_timeout_sec': DEFAULT_SESSION_TIMEOUT_SEC,
    'wakeup_cooldown_sec': 1.0,
    'tokens': '',
    'encoder': '',
    'decoder': '',
    'joiner': '',
    'paraformer': '',
    'enable_endpoint_detection': True,
    'rule1_min_trailing_silence': 2.4,
    'rule2_min_trailing_silence': 1.2,
    'rule3_min_utterance_length': 20.0,
    'api_base_url': DEFAULT_ASR_API_BASE_URL,
    'api_key': DEFAULT_ASR_API_KEY,
    'api_model': DEFAULT_ASR_API_MODEL,
    'language': DEFAULT_ASR_LANGUAGE,
    'prompt': DEFAULT_ASR_PROMPT,
    'silence_threshold': DEFAULT_ASR_SILENCE_THRESHOLD,
    'min_speech_ms': DEFAULT_ASR_MIN_SPEECH_MS,
    'trailing_silence_ms': DEFAULT_ASR_TRAILING_SILENCE_MS,
    'max_utterance_ms': DEFAULT_ASR_MAX_UTTERANCE_MS,
}


def _string_msg(text: str) -> String:
    msg = String()
    msg.data = text
    return msg


class ASRNode(Node):
    def __init__(self):
        super().__init__('asr_node')

        for name, default in PARAM_DEFAULTS.items():
            self.declare_parameter(name, default)

        self.input_device = self.get_parameter('input_device').value
        self.sample_rate = int(self.get_parameter('sample_rate').value)
        self.channels = int(self.get_parameter('channels').value)
        self.samples_per_chunk = int(self.get_parameter('samples_per_chunk').value)
        self.topic_final = self.get_parameter('topic_final').value
        self.topic_partial = self.get_parameter('topic_partial').value
        self.mode = self.get_parameter('mode').value
        self.command_topic = self.get_parameter('wakeup_topic').value
        self.voice_state_topic = self.get_parameter('voice_state_topic').value
        self.session_timeout_sec = float(self.get_parameter('session_timeout_sec').value)
        self.wakeup_cooldown_sec = float(self.get_parameter('wakeup_cooldown_sec').value)

        if self.sample_rate <= 0:
            raise ValueError('sample_rate must be positive')
        if self.channels <= 0:
            raise ValueError('channels must be positive')
        if self.samples_per_chunk <= 0:
            raise ValueError('samples_per_chunk must be positive')
        if self.mode not in ('always_on', 'wake_gated'):
            raise ValueError("mode must be 'always_on' or 'wake_gated'")
        if self.session_timeout_sec <= 0.0:
            raise ValueError('session_timeout_sec must be positive')
        if self.wakeup_cooldown_sec < 0.0:
            raise ValueError('wakeup_cooldown_sec must be non-negative')

        self.final_pub = self.create_publisher(String, self.topic_final, 10)
        self.partial_pub = self.create_publisher(String, self.topic_partial, 10)
        self.voice_state_pub = self.create_publisher(String, self.voice_state_topic, 10)

        param = self.get_parameter
        self.audio_capture = AudioCapture(
            input_device=self.input_device,
            sample_rate=self.sample_rate,
            channels=self.channels,
        )
        self.asr_engine = ASREngine(
            sample_rate=self.sample_rate,
            provider=param('provider').value,
            tokens=param('tokens').value,
            encoder=param('encoder').value,
            decoder=param('decoder').value,
            joiner=param('joiner').value,
            paraformer=param('paraformer').value,
            enable_endpoint_detection=param('enable_endpoint_detection').value,
            rule1_min_trailing_silence=param('rule1_min_trailing_silence').value,
            rule2_min_trailing_silence=param('rule2_min_trailing_silence').value,
            rule3_min_utterance_length=param('rule3_min_utterance_length').value,
            api_base_url=param('api_base_url').value,
            api_key=param('api_key').value,
            api_model=param('api_model').value,
            language=param('language').value,
            prompt=param('prompt').value,
            silence_threshold=param('silence_threshold').value,
            min_speech_ms=param('min_speech_ms').value,
            trailing_silence_ms=param('trailing_silence_ms').value,
            max_utterance_ms=param('max_utterance_ms').value,
        )

        self._running = True
        self._session_active = self.mode == 'always_on'
        self._pending_wakeup = False
        self._arming_session = False
        self._kws_capture_state = 'unknown'
        self._session_started_at = 0.0
        self._start_retry_count = 0
        self._last_wakeup_at = 0.0
        self._worker = threading.Thread(target=self._loop, daemon=True)
        self._worker.start()

        self.wakeup_sub = self.create_subscription(String, self.command_topic, self._on_wakeup, 10)
        self.voice_state_sub = self.create_subscription(String, self.voice_state_topic, self._on_voice_state, 10)
        self.kws_capture_sub = self.create_subscription(String, '/voice/kws_capture_state', self._on_kws_capture_state, 10)
        self.tts_done_sub = self.create_subscription(String, '/tts_done', self._on_tts_done, 10)

        self._publish_state(VOICE_STATE_LISTENING if self.mode == 'always_on' else VOICE_STATE_SLEEPING)
        self.get_logger().info(
            f'ASR node started with input_device={self.input_device}, sample_rate={self.sample_rate}, '
            f'channels={self.channels}, samples_per_chunk={self.samples_per_chunk}, mode={self.mode}, '
            f'command_topic={self.command_topic}, voice_state_topic={self.voice_state_topic}, '
            f'final_topic={self.topic_final}, partial_topic={self.topic_partial}, '
            f'session_timeout_sec={self.session_timeout_sec}, wakeup_cooldown_sec={self.wakeup_cooldown_sec}, '
            f'provider={self.get_parameter("provider").value}, api_model={self.get_parameter("api_model").value}'
        )

    def _publish_state(self, state: str):
        self.voice_state_pub.publish(_string_msg(state))

    def _publish_text(self, text: str, is_final: bool):
        msg = _string_msg(text)
        if is_final:
            self.final_pub.publish(msg)
            self.get_logger().info(f'ASR final: {text}')
        else:
            self.partial_pub.publish(msg)

    def _on_wakeup(self, msg: String):
        if self.mode == 'always_on':
            return

        wakeup_command = msg.data.strip().lower()
        if wakeup_command != 'start':
            self.get_logger().info(f'Wakeup ignored with unsupported command: {msg.data}')
            return

        now = time.monotonic()
        if (now - self._last_wakeup_at) < self.wakeup_cooldown_sec:
            self.get_logger().info(f'Wakeup ignored during cooldown: {msg.data}')
            return

        self.get_logger().info(f'Wakeup received, waiting for wake ack TTS to complete: {msg.data}')
        self.asr_engine.reset()
        self._pending_wakeup = True
        self._arming_session = False
        self._last_wakeup_at = now

    def _on_voice_state(self, msg: String):
        if self.mode != 'wake_gated':
            return

        state = msg.data.strip()
        if state in (VOICE_STATE_THINKING, VOICE_STATE_SPEAKING):
            self._session_active = False
            self._arming_session = False
            self.audio_capture.stop()
        elif state == VOICE_STATE_SLEEPING:
            self._session_active = False
            self._pending_wakeup = False
            self._arming_session = False
            self.audio_capture.stop()
            self.asr_engine.reset()

    def _on_kws_capture_state(self, msg: String):
        state = msg.data.strip()
        self._kws_capture_state = state
        self.get_logger().info(f'KWS capture state: {state}')


    def _on_tts_done(self, msg: String):
        if self.mode != 'wake_gated' or self._session_active or self._arming_session:
            return

        done_event = msg.data.strip()
        if done_event != 'wake_ack_done' or not self._pending_wakeup:
            return

        if self._kws_capture_state != 'released':
            self.get_logger().warning(f'Wake ack done before KWS reported release: {self._kws_capture_state}')

        self.get_logger().info('Wake ack TTS done, starting ASR capture')
        self._pending_wakeup = False
        self._arming_session = True
        self._start_retry_count = 0
        self._start_asr_session()

    def _start_asr_session(self):
        if not rclpy.ok() or self.mode != 'wake_gated':
            return
        if self._session_active or not self._arming_session:
            return

        self.get_logger().info('Starting ASR capture')
        self._session_active = True
        self._session_started_at = time.monotonic()
        self._arming_session = False
        self._publish_state(VOICE_STATE_LISTENING)

    def _loop(self):
        capture_started = False
        while self._running and rclpy.ok():
            try:
                if not self._session_active:
                    if capture_started:
                        self.audio_capture.stop()
                        capture_started = False
                    self._start_retry_count = 0
                    time.sleep(0.01)
                    continue

                if not capture_started:
                    self.audio_capture.start()
                    capture_started = True

                if self.mode == 'wake_gated' and (time.monotonic() - self._session_started_at) > self.session_timeout_sec:
                    self.get_logger().info('ASR session timeout, back to sleep')
                    self._session_active = False
                    self._pending_wakeup = False
                    self.audio_capture.stop()
                    capture_started = False
                    self._arming_session = False
                    self._start_retry_count = 0
                    self.asr_engine.reset()
                    self._publish_state(VOICE_STATE_SLEEPING)
                    continue

                pcm = self.audio_capture.read_chunk(self.samples_per_chunk)
                self._start_retry_count = 0

                result = self.asr_engine.accept_audio(pcm)
                if result and result.text:
                    self._publish_text(result.text, result.is_final)
                    if result.is_final and self.mode == 'wake_gated':
                        self._session_active = False
                        self._pending_wakeup = False
                        self.audio_capture.stop()
                        capture_started = False
                        self._arming_session = False
                        self._start_retry_count = 0
                        self._publish_state(VOICE_STATE_THINKING)
            except Exception as e:
                self.get_logger().warning(f'ASR loop recovered from error: {e}')
                self.audio_capture.stop()
                capture_started = False
                if self.mode == 'wake_gated' and self._session_active and self._start_retry_count < 2:
                    self._start_retry_count += 1
                    self.get_logger().warning(f'ASR capture retry {self._start_retry_count}/2')
                    time.sleep(0.15)
                    continue

                self._start_retry_count = 0
                self._arming_session = False
                if self.mode == 'wake_gated':
                    self._session_active = False
                    self._pending_wakeup = False
                    self.asr_engine.reset()
                    self._publish_state(VOICE_STATE_SLEEPING)
                time.sleep(0.2)

        self.audio_capture.stop()

    def destroy_node(self):
        self._running = False
        self.audio_capture.stop()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ASRNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
