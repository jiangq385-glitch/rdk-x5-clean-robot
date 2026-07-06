import audioop
import io
import json
import urllib.error
import urllib.request
import uuid
import wave
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np


@dataclass
class ASRResult:
    text: str
    is_final: bool


class ASREngine:
    def __init__(
        self,
        sample_rate: int,
        provider: str = 'cpu',
        tokens: str = '',
        encoder: str = '',
        decoder: str = '',
        joiner: str = '',
        paraformer: str = '',
        enable_endpoint_detection: bool = True,
        rule1_min_trailing_silence: float = 2.4,
        rule2_min_trailing_silence: float = 1.2,
        rule3_min_utterance_length: float = 20.0,
        api_base_url: str = '',
        api_key: str = '',
        api_model: str = '',
        language: str = 'zh',
        prompt: str = '',
        silence_threshold: int = 900,
        min_speech_ms: int = 300,
        trailing_silence_ms: int = 900,
        max_utterance_ms: int = 10000,
    ):
        self.sample_rate = sample_rate
        self.provider = provider
        self.tokens = tokens
        self.encoder = encoder
        self.decoder = decoder
        self.joiner = joiner
        self.paraformer = paraformer
        self.enable_endpoint_detection = enable_endpoint_detection
        self.rule1_min_trailing_silence = rule1_min_trailing_silence
        self.rule2_min_trailing_silence = rule2_min_trailing_silence
        self.rule3_min_utterance_length = rule3_min_utterance_length
        self.api_base_url = api_base_url.rstrip('/')
        self.api_key = api_key.strip()
        self.api_model = api_model.strip()
        self.language = language.strip()
        self.prompt = prompt.strip()
        self.silence_threshold = int(silence_threshold)
        self.min_speech_ms = int(min_speech_ms)
        self.trailing_silence_ms = int(trailing_silence_ms)
        self.max_utterance_ms = int(max_utterance_ms)
        self._recognizer = None
        self._stream = None
        self._last_partial = ''
        self._cloud_chunks: list[bytes] = []
        self._cloud_active = False
        self._cloud_active_ms = 0.0
        self._cloud_speech_ms = 0.0
        self._cloud_trailing_silence_ms = 0.0

        if self._is_cloud_provider():
            self._validate_cloud_config()
        else:
            self._init_recognizer()

    def _is_cloud_provider(self) -> bool:
        return self.provider == 'openai_compatible'

    def _validate_cloud_config(self):
        if not self.api_base_url:
            raise RuntimeError('云端 ASR 缺少 api_base_url 配置')
        if not self.api_key:
            raise RuntimeError('云端 ASR 缺少 api_key 配置')
        if not self.api_model:
            raise RuntimeError('云端 ASR 缺少 api_model 配置')

    def _validate_optional_path(self, path_str: str) -> str:
        if not path_str:
            return ''
        path = Path(path_str)
        if not path.exists():
            raise FileNotFoundError(f'Model file not found: {path_str}')
        return str(path)

    def _init_recognizer(self):
        try:
            import sherpa_onnx
        except ImportError as exc:
            raise RuntimeError(
                '未安装 sherpa_onnx。请先在板端安装对应 aarch64 版本。'
            ) from exc

        tokens = self._validate_optional_path(self.tokens)
        encoder = self._validate_optional_path(self.encoder)
        decoder = self._validate_optional_path(self.decoder)
        joiner = self._validate_optional_path(self.joiner)
        paraformer = self._validate_optional_path(self.paraformer)

        if paraformer:
            self._recognizer = sherpa_onnx.OnlineRecognizer.from_paraformer(
                paraformer=paraformer,
                tokens=tokens,
                num_threads=1,
                sample_rate=self.sample_rate,
                feature_dim=80,
                enable_endpoint_detection=self.enable_endpoint_detection,
                rule1_min_trailing_silence=self.rule1_min_trailing_silence,
                rule2_min_trailing_silence=self.rule2_min_trailing_silence,
                rule3_min_utterance_length=self.rule3_min_utterance_length,
                provider=self.provider,
                decoding_method='greedy_search',
            )
        elif encoder and decoder and joiner:
            self._recognizer = sherpa_onnx.OnlineRecognizer.from_transducer(
                encoder=encoder,
                decoder=decoder,
                joiner=joiner,
                tokens=tokens,
                num_threads=1,
                sample_rate=self.sample_rate,
                feature_dim=80,
                enable_endpoint_detection=self.enable_endpoint_detection,
                rule1_min_trailing_silence=self.rule1_min_trailing_silence,
                rule2_min_trailing_silence=self.rule2_min_trailing_silence,
                rule3_min_utterance_length=self.rule3_min_utterance_length,
                provider=self.provider,
                decoding_method='greedy_search',
            )
        else:
            raise RuntimeError(
                'ASR 模型参数未配置。请至少提供 paraformer，或 encoder/decoder/joiner/tokens。'
            )

        self._stream = self._recognizer.create_stream()

    def reset(self):
        if self._is_cloud_provider():
            self._cloud_chunks = []
            self._cloud_active = False
            self._cloud_active_ms = 0.0
            self._cloud_speech_ms = 0.0
            self._cloud_trailing_silence_ms = 0.0
            return

        self._stream = self._recognizer.create_stream()
        self._last_partial = ''

    def _pcm16_to_float32(self, pcm_bytes: bytes) -> np.ndarray:
        audio = np.frombuffer(pcm_bytes, dtype=np.int16).astype(np.float32)
        audio = audio / 32768.0
        return audio

    def _chunk_duration_ms(self, pcm_bytes: bytes) -> float:
        bytes_per_sample = 2  # mono int16
        sample_count = len(pcm_bytes) / bytes_per_sample
        return sample_count * 1000.0 / self.sample_rate

    def _chunk_has_voice(self, pcm_bytes: bytes) -> bool:
        if not pcm_bytes:
            return False
        try:
            rms = audioop.rms(pcm_bytes, 2)
        except audioop.error:
            return False
        return rms >= self.silence_threshold

    def _build_wav_bytes(self, pcm_bytes: bytes) -> bytes:
        buffer = io.BytesIO()
        with wave.open(buffer, 'wb') as wav_file:
            wav_file.setnchannels(1)
            wav_file.setsampwidth(2)
            wav_file.setframerate(self.sample_rate)
            wav_file.writeframes(pcm_bytes)
        return buffer.getvalue()

    def _build_multipart_body(self, wav_bytes: bytes) -> tuple[bytes, str]:
        boundary = f'----ClaudeBoundary{uuid.uuid4().hex}'
        fields = [('model', self.api_model)]

        parts: list[bytes] = []
        for key, value in fields:
            parts.append(f'--{boundary}\r\n'.encode('utf-8'))
            parts.append(
                f'Content-Disposition: form-data; name="{key}"\r\n\r\n{value}\r\n'.encode('utf-8')
            )

        parts.append(f'--{boundary}\r\n'.encode('utf-8'))
        parts.append(
            b'Content-Disposition: form-data; name="file"; filename="speech.wav"\r\n'
        )
        parts.append(b'Content-Type: audio/wav\r\n\r\n')
        parts.append(wav_bytes)
        parts.append(b'\r\n')
        parts.append(f'--{boundary}--\r\n'.encode('utf-8'))
        return b''.join(parts), boundary

    def _transcribe_cloud_audio(self, pcm_bytes: bytes) -> str:
        wav_bytes = self._build_wav_bytes(pcm_bytes)
        body, boundary = self._build_multipart_body(wav_bytes)
        req = urllib.request.Request(
            url=f'{self.api_base_url}/audio/transcriptions',
            data=body,
            headers={
                'Authorization': f'Bearer {self.api_key}',
                'Content-Type': f'multipart/form-data; boundary={boundary}',
            },
            method='POST',
        )

        try:
            with urllib.request.urlopen(req, timeout=60.0) as resp:
                payload = resp.read().decode('utf-8')
        except urllib.error.HTTPError as e:
            err = e.read().decode('utf-8', errors='ignore')
            raise RuntimeError(f'云端 ASR HTTPError: {e.code} {err}') from e
        except urllib.error.URLError as e:
            raise RuntimeError(f'无法连接云端 ASR: {e}') from e

        result = json.loads(payload)
        text = str(result.get('text', '')).strip()
        return text

    def _accept_audio_cloud(self, pcm_bytes: bytes) -> Optional[ASRResult]:
        chunk_ms = self._chunk_duration_ms(pcm_bytes)
        has_voice = self._chunk_has_voice(pcm_bytes)

        if not self._cloud_active:
            if not has_voice:
                return None
            self._cloud_active = True

        self._cloud_chunks.append(pcm_bytes)
        self._cloud_active_ms += chunk_ms

        if has_voice:
            self._cloud_speech_ms += chunk_ms
            self._cloud_trailing_silence_ms = 0.0
        else:
            self._cloud_trailing_silence_ms += chunk_ms

        should_finalize = (
            self._cloud_trailing_silence_ms >= self.trailing_silence_ms
            or self._cloud_active_ms >= self.max_utterance_ms
        )
        if not should_finalize:
            return None

        merged_pcm = b''.join(self._cloud_chunks)
        speech_ms = self._cloud_speech_ms
        self.reset()

        if speech_ms < self.min_speech_ms:
            return None

        text = self._transcribe_cloud_audio(merged_pcm)
        if not text:
            return None
        return ASRResult(text=text, is_final=True)

    def accept_audio(self, pcm_bytes: bytes) -> Optional[ASRResult]:
        if self._is_cloud_provider():
            return self._accept_audio_cloud(pcm_bytes)

        samples = self._pcm16_to_float32(pcm_bytes)
        self._stream.accept_waveform(self.sample_rate, samples)
        while self._recognizer.is_ready(self._stream):
            self._recognizer.decode_stream(self._stream)

        text = self._recognizer.get_result(self._stream).strip()
        endpoint = self._recognizer.is_endpoint(self._stream)

        if endpoint:
            final_text = text
            self.reset()
            if final_text:
                return ASRResult(text=final_text, is_final=True)
            return None

        if text and text != self._last_partial:
            self._last_partial = text
            return ASRResult(text=text, is_final=False)

        return None
