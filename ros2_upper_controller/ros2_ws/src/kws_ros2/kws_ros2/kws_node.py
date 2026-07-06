import time
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from sherpa_onnx_ros2.audio_capture import AudioCapture

from .audio_buffer import AudioBuffer
from .config import (
    DEFAULT_CHANNELS,
    DEFAULT_COOLDOWN_SEC,
    DEFAULT_ENGINE,
    DEFAULT_FBANK_DITHER,
    DEFAULT_FBANK_FRAME_LENGTH_MS,
    DEFAULT_FBANK_FRAME_SHIFT_MS,
    DEFAULT_FEATURE_DIM,
    DEFAULT_HOP_MS,
    DEFAULT_INPUT_DEVICE,
    DEFAULT_KWS_MODEL_PATH,
    DEFAULT_SAMPLE_RATE,
    DEFAULT_SAMPLES_PER_CHUNK,
    DEFAULT_RESUME_GRACE_SEC,
    DEFAULT_SCORE_THRESHOLD,
    DEFAULT_TRIGGER_COUNT,
    DEFAULT_VOICE_STATE_TOPIC,
    DEFAULT_WAKEUP_TOPIC,
    DEFAULT_WAKE_WORDS,
    DEFAULT_PAUSE_ON_SPEAKING,
    DEFAULT_WINDOW_MS,
    VOICE_STATE_SLEEPING,
)
from .kws_engine import KWSBackendError, create_kws_engine


PARAM_DEFAULTS = {
    'input_device': DEFAULT_INPUT_DEVICE,
    'sample_rate': DEFAULT_SAMPLE_RATE,
    'channels': DEFAULT_CHANNELS,
    'samples_per_chunk': DEFAULT_SAMPLES_PER_CHUNK,
    'wakeup_topic': DEFAULT_WAKEUP_TOPIC,
    'voice_state_topic': DEFAULT_VOICE_STATE_TOPIC,
    'engine': DEFAULT_ENGINE,
    'wake_words': DEFAULT_WAKE_WORDS,
    'cooldown_sec': DEFAULT_COOLDOWN_SEC,
    'resume_grace_sec': DEFAULT_RESUME_GRACE_SEC,
    'score_threshold': DEFAULT_SCORE_THRESHOLD,
    'trigger_count': DEFAULT_TRIGGER_COUNT,
    'window_ms': DEFAULT_WINDOW_MS,
    'hop_ms': DEFAULT_HOP_MS,
    'feature_dim': DEFAULT_FEATURE_DIM,
    'fbank_frame_length_ms': DEFAULT_FBANK_FRAME_LENGTH_MS,
    'fbank_frame_shift_ms': DEFAULT_FBANK_FRAME_SHIFT_MS,
    'fbank_dither': DEFAULT_FBANK_DITHER,
    'kws_model_path': DEFAULT_KWS_MODEL_PATH,
    'pause_on_speaking': DEFAULT_PAUSE_ON_SPEAKING,
}


def _string_msg(text: str) -> String:
    msg = String()
    msg.data = text
    return msg


class KWSNode(Node):
    def __init__(self):
        super().__init__('kws_node')

        for name, default in PARAM_DEFAULTS.items():
            self.declare_parameter(name, default)

        self.input_device = self.get_parameter('input_device').value
        self.sample_rate = int(self.get_parameter('sample_rate').value)
        self.channels = int(self.get_parameter('channels').value)
        self.samples_per_chunk = int(self.get_parameter('samples_per_chunk').value)
        self.wakeup_topic = self.get_parameter('wakeup_topic').value
        self.voice_state_topic = self.get_parameter('voice_state_topic').value
        self.engine_name = self.get_parameter('engine').value
        self.wake_words = list(self.get_parameter('wake_words').value)
        self.cooldown_sec = float(self.get_parameter('cooldown_sec').value)
        self.resume_grace_sec = float(self.get_parameter('resume_grace_sec').value)
        self.score_threshold = float(self.get_parameter('score_threshold').value)
        self.trigger_count = max(1, int(self.get_parameter('trigger_count').value))
        self.window_ms = int(self.get_parameter('window_ms').value)
        self.hop_ms = int(self.get_parameter('hop_ms').value)
        self.feature_dim = int(self.get_parameter('feature_dim').value)
        self.fbank_frame_length_ms = float(self.get_parameter('fbank_frame_length_ms').value)
        self.fbank_frame_shift_ms = float(self.get_parameter('fbank_frame_shift_ms').value)
        self.fbank_dither = float(self.get_parameter('fbank_dither').value)
        self.kws_model_path = self.get_parameter('kws_model_path').value
        self.pause_on_speaking = bool(self.get_parameter('pause_on_speaking').value)

        if self.sample_rate <= 0:
            raise ValueError('sample_rate must be positive')
        if self.channels <= 0:
            raise ValueError('channels must be positive')
        if self.samples_per_chunk <= 0:
            raise ValueError('samples_per_chunk must be positive')
        if self.window_ms <= 0:
            raise ValueError('window_ms must be positive')
        if self.hop_ms <= 0:
            raise ValueError('hop_ms must be positive')
        if self.cooldown_sec < 0.0:
            raise ValueError('cooldown_sec must be non-negative')
        if self.resume_grace_sec < 0.0:
            raise ValueError('resume_grace_sec must be non-negative')
        if not 0.0 <= self.score_threshold <= 1.0:
            raise ValueError('score_threshold must be between 0.0 and 1.0')
        if not self.wake_words or not any(word.strip() for word in self.wake_words):
            raise ValueError('wake_words must not be empty')
        if self.engine_name != 'bpu_kws':
            raise ValueError("Only 'bpu_kws' engine is allowed")
        if self.feature_dim <= 0:
            raise ValueError('feature_dim must be positive')
        if self.fbank_frame_length_ms <= 0.0:
            raise ValueError('fbank_frame_length_ms must be positive')
        if self.fbank_frame_shift_ms <= 0.0:
            raise ValueError('fbank_frame_shift_ms must be positive')

        self.audio_capture = AudioCapture(
            input_device=self.input_device,
            sample_rate=self.sample_rate,
            channels=self.channels,
        )
        self.wakeup_pub = self.create_publisher(String, self.wakeup_topic, 10)
        self.capture_state_pub = self.create_publisher(String, '/voice/kws_capture_state', 10)
        self.voice_state_sub = self.create_subscription(String, self.voice_state_topic, self._on_voice_state, 10)

        self.engine = create_kws_engine(
            engine_name=self.engine_name,
            sample_rate=self.sample_rate,
            score_threshold=self.score_threshold,
            wake_words=self.wake_words,
            kws_model_path=self.kws_model_path,
            feature_dim=self.feature_dim,
            window_ms=self.window_ms,
            fbank_frame_length_ms=self.fbank_frame_length_ms,
            fbank_frame_shift_ms=self.fbank_frame_shift_ms,
            fbank_dither=self.fbank_dither,
            logger=self.get_logger(),
        )

        self._running = True
        self._paused = False
        self._cooldown_until = 0.0
        self._hit_count = 0
        self._hop_bytes = max(1, self.sample_rate * self.hop_ms // 1000) * 2 * self.channels
        self._window_bytes = max(1, self.sample_rate * self.window_ms // 1000) * 2 * self.channels
        self._buffer = AudioBuffer(self._window_bytes)
        self._timer = self.create_timer(self.hop_ms / 1000.0, self._tick)

        self.audio_capture.start()
        self._publish_capture_state('capturing')
        self.get_logger().info(
            f'KWS node started with input_device={self.input_device}, sample_rate={self.sample_rate}, '
            f'channels={self.channels}, samples_per_chunk={self.samples_per_chunk}, engine={self.engine_name}, '
            f'wakeup_topic={self.wakeup_topic}, voice_state_topic={self.voice_state_topic}, '
            f'cooldown_sec={self.cooldown_sec}, resume_grace_sec={self.resume_grace_sec}, score_threshold={self.score_threshold}, '
            f'trigger_count={self.trigger_count}, window_ms={self.window_ms}, hop_ms={self.hop_ms}, '
            f'feature_dim={self.feature_dim}, fbank_frame_length_ms={self.fbank_frame_length_ms}, '
            f'fbank_frame_shift_ms={self.fbank_frame_shift_ms}, fbank_dither={self.fbank_dither}, '
            f'wake_words={self.wake_words}, kws_model_path={self.kws_model_path}, '
            f'pause_on_speaking={self.pause_on_speaking}'
        )

    def _publish_capture_state(self, state: str):
        self.capture_state_pub.publish(_string_msg(state))

    def _on_voice_state(self, msg: String):
        state = msg.data.strip()
        should_run = state == VOICE_STATE_SLEEPING
        if should_run == (not self._paused):
            return

        self._paused = not should_run
        self._hit_count = 0
        self._buffer.clear()
        if self._paused:
            self.get_logger().info(f'KWS paused on voice_state={state}')
            self.audio_capture.stop()
            self._publish_capture_state('released')
            return

        self.get_logger().info(f'KWS resumed on voice_state={state}')
        self.audio_capture.start()
        self._cooldown_until = max(self._cooldown_until, time.monotonic() + self.resume_grace_sec)
        self._publish_capture_state('capturing')

    def _tick(self):
        if not self._running or self._paused:
            return
        if time.monotonic() < self._cooldown_until:
            return

        try:
            pcm = self.audio_capture.read_chunk(self.samples_per_chunk)
        except Exception as e:
            self.audio_capture.stop()
            try:
                self.audio_capture.start()
            except Exception as restart_error:
                self.get_logger().warning(f'KWS audio restart failed: {restart_error}')
            self.get_logger().warning(f'KWS audio read failed: {e}')
            return

        if not pcm:
            return
        # ── 调试：打印原始 PCM 统计 ──
        pcm_arr = np.frombuffer(pcm, dtype=np.int16)
        self.get_logger().info(
            f'[PCM] len={len(pcm)}, min={int(pcm_arr.min())}, '
            f'max={int(pcm_arr.max())}, mean={float(pcm_arr.mean()):.1f}, '
            f'std={float(pcm_arr.std()):.1f}'
        )

        self._buffer.append(pcm)
        if len(self._buffer) < self._window_bytes:
            return

        window = self._buffer.get_bytes()
        try:
            detected, score = self.engine.detect(window)
        except KWSBackendError as e:
            self.get_logger().error(f'KWS backend failed: {e}')
            self._hit_count = 0
            self._cooldown_until = time.monotonic() + self.cooldown_sec
            return

        # ── 加这行调试日志 ──
        if detected:
            self._hit_count += 1
        else:
            self._hit_count = 0

        self.get_logger().info(
            f'KWS score={score:.4f}, detected={detected}, hits={self._hit_count}'
        )

        if self._hit_count >= self.trigger_count:
            self._publish_wakeup(f'{self.engine_name}:{score:.3f}')
            self._buffer.clear()
            self._hit_count = 0
            return

        self._buffer.drop_prefix(self._hop_bytes)

    def _publish_wakeup(self, reason: str):
        self.wakeup_pub.publish(_string_msg(reason))
        self._cooldown_until = time.monotonic() + self.cooldown_sec
        self.get_logger().info(f'Wakeup published: {reason}')

    def destroy_node(self):
        self._running = False
        self.audio_capture.stop()

        if hasattr(self.engine, 'close'):
            try:
                self.engine.close()
            except Exception as e:
                self.get_logger().warning(f'KWS engine close failed: {e}')

        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = KWSNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
