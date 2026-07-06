import queue
import threading
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .audio_player import AudioPlayer
from .config import (
    DEFAULT_OUTPUT_DEVICE,
    DEFAULT_RATE,
    DEFAULT_TMP_DIR,
    DEFAULT_TTS_TOPIC,
    DEFAULT_VOICE,
    DEFAULT_VOLUME,
)
from .tts_engine import TTSEngine


def _string_msg(text: str) -> String:
    msg = String()
    msg.data = text
    return msg


class TTSNode(Node):
    def __init__(self):
        super().__init__('tts_node')

        self.declare_parameter('voice', DEFAULT_VOICE)
        self.declare_parameter('rate', DEFAULT_RATE)
        self.declare_parameter('volume', DEFAULT_VOLUME)
        self.declare_parameter('output_device', DEFAULT_OUTPUT_DEVICE)
        self.declare_parameter('tts_topic', DEFAULT_TTS_TOPIC)
        self.declare_parameter('tmp_dir', DEFAULT_TMP_DIR)
        self.declare_parameter('proxy', '')
        self.declare_parameter('wake_ack_text', '我在，你说')

        voice = self.get_parameter('voice').value
        rate = self.get_parameter('rate').value
        volume = self.get_parameter('volume').value
        output_device = self.get_parameter('output_device').value
        tts_topic = self.get_parameter('tts_topic').value
        tmp_dir = self.get_parameter('tmp_dir').value
        proxy = self.get_parameter('proxy').value
        self.wake_ack_text = self.get_parameter('wake_ack_text').value.strip()

        self.tts_engine = TTSEngine(voice=voice, rate=rate, volume=volume, tmp_dir=tmp_dir, proxy=proxy)
        self.audio_player = AudioPlayer(output_device=output_device)
        self.queue = queue.Queue()
        self.tts_done_pub = self.create_publisher(String, '/tts_done', 10)
        self.worker = threading.Thread(target=self._worker_loop, daemon=True)
        self.worker.start()

        self.subscription = self.create_subscription(String, tts_topic, self._tts_callback, 10)
        self.get_logger().info(f'TTS node started. topic={tts_topic}, voice={voice}, proxy={self.tts_engine.proxy or "<env>"}')

    def _tts_callback(self, msg: String):
        text = msg.data.strip()
        if text:
            self.queue.put(text)
            self.get_logger().info(f'TTS queued: {text}')

    def _worker_loop(self):
        while rclpy.ok():
            try:
                text = self.queue.get(timeout=0.5)
            except queue.Empty:
                continue

            try:
                started_at = time.perf_counter()
                self.get_logger().info(f'TTS synth start: {text}')
                audio_path = self.tts_engine.synthesize(text)
                synth_elapsed = time.perf_counter() - started_at
                size = 0
                try:
                    size = Path(audio_path).stat().st_size
                except Exception:
                    pass
                self.get_logger().info(
                    f'TTS synth done: {audio_path} ({size} bytes) elapsed={synth_elapsed:.2f}s'
                )

                play_started_at = time.perf_counter()
                self.get_logger().info(f'TTS play start: {audio_path}')
                self.audio_player.play(audio_path)
                play_elapsed = time.perf_counter() - play_started_at
                self.get_logger().info(f'TTS play done: elapsed={play_elapsed:.2f}s')
                done_event = 'wake_ack_done' if text == self.wake_ack_text else 'tts_done'
                self.tts_done_pub.publish(_string_msg(done_event))
                self.get_logger().info(f'TTS done published: {done_event}')
            except Exception as e:
                self.get_logger().error(f'TTS failed: {e}')
            finally:
                self.queue.task_done()


def main(args=None):
    rclpy.init(args=args)
    node = TTSNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
