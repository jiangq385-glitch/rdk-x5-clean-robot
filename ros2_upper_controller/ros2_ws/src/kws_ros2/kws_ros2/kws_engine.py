import math
import time
import threading
import queue
from pathlib import Path
from typing import Sequence

import numpy as np


DEBUG_RANDOM_INPUT_EVERY = 0
DEBUG_PRINT_MODEL_IO = False


class KWSBackendError(RuntimeError):
    pass


class BaseKWSEngine:
    def score(self, pcm: bytes) -> float:
        raise NotImplementedError

    def detect(self, pcm: bytes) -> tuple[bool, float]:
        score = self.score(pcm)
        return score >= self.score_threshold, score


class EnergyGateEngine(BaseKWSEngine):
    def __init__(self, score_threshold: float):
        self.score_threshold = float(score_threshold)

    def score(self, pcm: bytes) -> float:
        if not pcm:
            return 0.0

        sample_count = len(pcm) // 2
        if sample_count <= 0:
            return 0.0

        total = 0
        for i in range(0, len(pcm), 2):
            sample = int.from_bytes(pcm[i:i + 2], byteorder='little', signed=True)
            total += sample * sample

        return math.sqrt(total / sample_count)


class BPUKwsEngine(BaseKWSEngine):
    def __init__(
        self,
        sample_rate: int,
        score_threshold: float,
        wake_words: Sequence[str],
        kws_model_path: str,
        feature_dim: int,
        window_ms: int,
        fbank_frame_length_ms: float,
        fbank_frame_shift_ms: float,
        fbank_dither: float,
        logger=None,
    ):
        self.sample_rate = int(sample_rate)
        self.score_threshold = float(score_threshold)
        self.wake_words = tuple(str(w).strip() for w in wake_words if str(w).strip())
        self.kws_model_path = kws_model_path.strip()
        self.feature_dim = int(feature_dim)
        self.window_ms = int(window_ms)
        self.fbank_frame_length_ms = float(fbank_frame_length_ms)
        self.fbank_frame_shift_ms = float(fbank_frame_shift_ms)
        self.fbank_dither = float(fbank_dither)
        self.logger = logger

        self._stft = None
        self._mel_basis = None
        self._ready = False

        self._expected_samples = max(1, int(self.sample_rate * self.window_ms / 1000))
        self._target_frames = max(1, int(round(self.window_ms / self.fbank_frame_shift_ms)))

        self._infer_queue = queue.Queue(maxsize=2)
        self._result_map = {}
        self._result_events = {}
        self._request_counter = 0
        self._counter_lock = threading.Lock()
        self._infer_thread = None
        self._stop_event = threading.Event()

        self._debug_infer_count = 0

        self._init_backend()
        self._start_infer_thread()

    def _init_backend(self):
        if not self.kws_model_path:
            raise KWSBackendError('BPU KWS selected but kws_model_path is empty')

        model_path = Path(self.kws_model_path)
        if not model_path.exists() or not model_path.is_file():
            raise KWSBackendError(f'KWS model not found: {self.kws_model_path}')

        try:
            import bpu_infer_lib  # type: ignore
            self._bpu_infer_lib = bpu_infer_lib
        except Exception as exc:
            raise KWSBackendError(f'Failed to import bpu_infer_lib: {exc}') from exc

        self._create_fbank_extractor()
        self._ready = True

        if self.logger:
            self.logger.info(
                f'KWS model validated: {self.kws_model_path}, '
                f'wake_words={list(self.wake_words)}, '
                f'feature_dim={self.feature_dim}, window_ms={self.window_ms}'
            )

    def _start_infer_thread(self):
        self._infer_thread = threading.Thread(target=self._infer_loop, daemon=True)
        self._infer_thread.start()

    def close(self):
        self._stop_event.set()
        try:
            self._infer_queue.put_nowait(None)
        except Exception:
            pass

    def _infer_loop(self):
        while not self._stop_event.is_set():
            item = self._infer_queue.get()

            if item is None:
                break

            request_id, input_tensor = item

            try:
                score = self._do_bpu_infer(input_tensor)
                self._result_map[request_id] = score
            except Exception as exc:
                self._result_map[request_id] = 0.0
                if self.logger:
                    self.logger.error(f'Infer thread error: {exc}')
            finally:
                evt = self._result_events.get(request_id)
                if evt:
                    evt.set()

    def _print_model_io_info(self, infer):
        if not self.logger or not DEBUG_PRINT_MODEL_IO:
            return

        try:
            inputs = getattr(infer, 'inputs', None)
            outputs = getattr(infer, 'outputs', None)

            if inputs is not None:
                for i, inp in enumerate(inputs):
                    try:
                        props = inp.properties
                        self.logger.info(f'BPU input[{i}] properties: shape={props.validShape}, '
                                         f'type={props.tensorType}, layout={props.tensorLayout}, '
                                         f'scale={props.scale}, shift={props.shift}')
                    except Exception:
                        pass

            if outputs is not None:
                for i, out in enumerate(outputs):
                    try:
                        props = out.properties
                        self.logger.info(f'BPU output[{i}] properties: shape={props.validShape}, '
                                         f'type={props.tensorType}')
                    except Exception:
                        pass

        except Exception:
            pass

    def _try_call_input_api(self, infer, input_tensor: np.ndarray):
        input_tensor = np.ascontiguousarray(input_tensor, dtype=np.float32)

        methods = [
            'read_numpy_arr_float32',
            'read_input_float32',
            'read_input',
        ]

        last_error = None

        for method_name in methods:
            if not hasattr(infer, method_name):
                continue

            method = getattr(infer, method_name)

            call_patterns = [
                lambda m=method, t=input_tensor: m(t, 0),
                lambda m=method, t=input_tensor: m(0, t),
                lambda m=method, t=input_tensor: m(t),
            ]

            for call in call_patterns:
                try:
                    call()
                    return
                except TypeError:
                    continue
                except Exception:
                    continue

        raise KWSBackendError(f'No working BPU input API found')

    def _try_call_output_api(self, infer) -> np.ndarray:
        if hasattr(infer, 'get_infer_res_np_float32'):
            method = getattr(infer, 'get_infer_res_np_float32')

            for call in [
                lambda: method(0),
                lambda: method(),
            ]:
                try:
                    output = call()
                    output = np.asarray(output, dtype=np.float32).reshape(-1)
                    if output.size > 0:
                        return output
                except TypeError:
                    continue
                except Exception:
                    continue

        infer.get_output()
        return np.asarray(infer.outputs[0].data, dtype=np.float32).reshape(-1)

    def _do_bpu_infer(self, input_tensor: np.ndarray) -> float:
        fresh_infer = None
        output = None

        try:
            fresh_infer = self._bpu_infer_lib.Infer(False)
            fresh_infer.load_model(self.kws_model_path)

            self._print_model_io_info(fresh_infer)

            input_tensor = np.ascontiguousarray(input_tensor, dtype=np.float32)

            if DEBUG_RANDOM_INPUT_EVERY > 0:
                self._debug_infer_count += 1
                if self._debug_infer_count % DEBUG_RANDOM_INPUT_EVERY == 0:
                    input_tensor = np.random.randn(
                        1, 1, self.feature_dim, self._target_frames
                    ).astype(np.float32)
                    input_tensor = np.ascontiguousarray(input_tensor, dtype=np.float32)

                    if self.logger:
                        self.logger.warning(
                            f'RANDOM input: shape={input_tensor.shape}, '
                            f'mean={float(input_tensor.mean()):.4f}, '
                            f'std={float(input_tensor.std()):.4f}'
                        )

            self._try_call_input_api(fresh_infer, input_tensor)

            fresh_infer.forward()

            output = self._try_call_output_api(fresh_infer)

        except Exception as exc:
            raise KWSBackendError(f'BPU inference failed: {exc}') from exc
        finally:
            try:
                del fresh_infer
            except Exception:
                pass

        if output is None or output.size == 0:
            raise KWSBackendError('BPU inference output is empty')

        return self._parse_score(output)

    def _parse_score(self, output: np.ndarray) -> float:
        output = np.asarray(output, dtype=np.float32).reshape(-1)

        if output.size == 0:
            raise KWSBackendError('BPU inference output is empty')

        if output.size == 1:
            return float(output[0])

        if output.size == 2:
            logits = output - np.max(output)
            probs = np.exp(logits)
            probs = probs / np.sum(probs)
            return float(probs[1])

        return float(np.max(output))

    def _create_fbank_extractor(self):
        try:
            from scipy.signal import stft as _stft
            self._stft = _stft
        except ImportError:
            raise KWSBackendError('Missing scipy. Install: pip install scipy')

        self._mel_basis = self._build_mel_filterbank(
            n_fft=512,
            sr=self.sample_rate,
            n_mels=self.feature_dim,
            fmin=20.0,
            fmax=8000.0,
        )

    @staticmethod
    def _hz_to_mel(hz):
        return 2595.0 * np.log10(1.0 + hz / 700.0)

    @staticmethod
    def _mel_to_hz(mel):
        return 700.0 * (10.0 ** (mel / 2595.0) - 1.0)

    def _build_mel_filterbank(self, n_fft, sr, n_mels, fmin, fmax):
        n_freqs = n_fft // 2 + 1
        mel_min = self._hz_to_mel(fmin)
        mel_max = self._hz_to_mel(fmax)
        mel_points = np.linspace(mel_min, mel_max, n_mels + 2)
        hz_points = self._mel_to_hz(mel_points)
        bin_points = np.floor((n_fft + 1) * hz_points / sr).astype(int)

        filterbank = np.zeros((n_mels, n_freqs), dtype=np.float32)

        for m in range(n_mels):
            left = bin_points[m]
            center = bin_points[m + 1]
            right = bin_points[m + 2]

            for k in range(left, center):
                if center != left and 0 <= k < n_freqs:
                    filterbank[m, k] = (k - left) / (center - left)

            for k in range(center, right):
                if right != center and 0 <= k < n_freqs:
                    filterbank[m, k] = (right - k) / (right - center)

        return filterbank

    def _pcm16_to_float32(self, pcm: bytes) -> np.ndarray:
        if not pcm:
            return np.zeros((0,), dtype=np.float32)

        audio = np.frombuffer(pcm, dtype=np.int16).astype(np.float32)
        return audio / 32768.0

    def _audio_trunc(self, waveform: np.ndarray) -> np.ndarray:
        if waveform.ndim != 1:
            waveform = waveform.reshape(-1)

        if waveform.size >= self._expected_samples:
            return waveform[-self._expected_samples:]

        padded = np.zeros((self._expected_samples,), dtype=np.float32)
        padded[-waveform.size:] = waveform
        return padded

    def _extract_fbank(self, waveform: np.ndarray) -> np.ndarray:
        if self._stft is None:
            self._create_fbank_extractor()

        n_fft = 512
        hop_length = int(self.sample_rate * self.fbank_frame_shift_ms / 1000)
        win_length = int(self.sample_rate * self.fbank_frame_length_ms / 1000)

        if hop_length <= 0 or win_length <= 0 or win_length <= hop_length:
            raise KWSBackendError(f'Invalid STFT: win={win_length}, hop={hop_length}')

        _, _, Zxx = self._stft(
            waveform,
            fs=self.sample_rate,
            nperseg=win_length,
            noverlap=win_length - hop_length,
            nfft=n_fft,
            window='hann',
            padded=False,
            boundary=None,
        )

        power_spec = (np.abs(Zxx) ** 2).T
        mel_spec = np.dot(power_spec, self._mel_basis.T)
        log_fbank = np.log(mel_spec + 1e-10).astype(np.float32)

        mean = np.mean(log_fbank)
        std = np.std(log_fbank)

        if std < 1e-6:
            std = 1.0

        log_fbank = (log_fbank - mean) / std

        return log_fbank.astype(np.float32)

    def _prepare_input_tensor(self, features: np.ndarray) -> np.ndarray:
        if features.ndim != 2:
            raise KWSBackendError(f'Unexpected feature rank: {features.ndim}')

        if features.shape[1] != self.feature_dim:
            raise KWSBackendError(
                f'Unexpected feature dim {features.shape[1]}, expected {self.feature_dim}'
            )

        if features.shape[0] >= self._target_frames:
            features = features[-self._target_frames:, :]
        else:
            padded = np.zeros((self._target_frames, self.feature_dim), dtype=np.float32)
            padded[-features.shape[0]:, :] = features
            features = padded

        # features: (T, F) = (100, 40)
        # 模型要求 NCHW: (1, 1, 40, 100)
        features = features.T

        input_tensor = features.reshape(
            1,
            1,
            self.feature_dim,
            self._target_frames,
        ).astype(np.float32, copy=False)

        return np.ascontiguousarray(input_tensor, dtype=np.float32)

    def score(self, pcm: bytes) -> float:
        if not self._ready:
            raise KWSBackendError('BPU KWS engine is not ready')

        waveform = self._pcm16_to_float32(pcm)
        waveform = self._audio_trunc(waveform)
        features = self._extract_fbank(waveform)
        input_tensor = self._prepare_input_tensor(features)

        with self._counter_lock:
            self._request_counter += 1
            request_id = self._request_counter

        event = threading.Event()
        self._result_events[request_id] = event

        try:
            self._infer_queue.put_nowait((request_id, input_tensor))
        except queue.Full:
            self._result_events.pop(request_id, None)
            if self.logger:
                self.logger.warning('KWS infer queue full, skip this frame')
            return 0.0

        if event.wait(timeout=3.0):
            score = self._result_map.pop(request_id, 0.0)
            self._result_events.pop(request_id, None)
            return float(score)

        self._result_map.pop(request_id, None)
        self._result_events.pop(request_id, None)

        return 0.0


def create_kws_engine(
    engine_name: str,
    sample_rate: int,
    score_threshold: float,
    wake_words: Sequence[str],
    kws_model_path: str,
    feature_dim: int,
    window_ms: int,
    fbank_frame_length_ms: float,
    fbank_frame_shift_ms: float,
    fbank_dither: float,
    logger=None,
) -> BaseKWSEngine:
    name = (engine_name or 'energy_gate').strip().lower()

    if name == 'bpu_kws':
        return BPUKwsEngine(
            sample_rate=sample_rate,
            score_threshold=score_threshold,
            wake_words=wake_words,
            kws_model_path=kws_model_path,
            feature_dim=feature_dim,
            window_ms=window_ms,
            fbank_frame_length_ms=fbank_frame_length_ms,
            fbank_frame_shift_ms=fbank_frame_shift_ms,
            fbank_dither=fbank_dither,
            logger=logger,
        )

    return EnergyGateEngine(score_threshold=score_threshold)