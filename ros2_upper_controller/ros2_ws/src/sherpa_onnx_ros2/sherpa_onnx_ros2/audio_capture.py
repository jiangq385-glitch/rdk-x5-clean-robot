import subprocess
import threading
from typing import Optional


class AudioCapture:
    def __init__(self, input_device: str = 'default', sample_rate: int = 16000, channels: int = 1):
        self.input_device = input_device
        self.sample_rate = sample_rate
        self.channels = channels
        self._proc: Optional[subprocess.Popen] = None
        self._lock = threading.RLock()

    def is_running(self) -> bool:
        return self._proc is not None and self._proc.poll() is None

    def start(self):
        with self._lock:
            if self.is_running():
                return

            self.stop()

            cmd = [
                'arecord',
                '-q',
                '-t', 'raw',
                '-f', 'S16_LE',
                '-r', str(self.sample_rate),
                '-c', str(self.channels),
            ]
            if self.input_device != 'default':
                cmd += ['-D', self.input_device]
            cmd += ['-']

            self._proc = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                bufsize=0,
            )

    def read_chunk(self, samples_per_chunk: int) -> bytes:
        with self._lock:
            proc = self._proc
            if proc is None or proc.poll() is not None or proc.stdout is None:
                raise RuntimeError('Audio capture has not started')

            bytes_per_sample = 2 * self.channels
            chunk_size = samples_per_chunk * bytes_per_sample
            data = proc.stdout.read(chunk_size)
        if not data:
            raise RuntimeError('No audio data read from arecord')
        return data

    def stop(self):
        with self._lock:
            proc = self._proc
            self._proc = None

        if proc is None:
            return

        if proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait(timeout=1.0)

        if proc.stdout is not None:
            proc.stdout.close()
        if proc.stderr is not None:
            proc.stderr.close()
