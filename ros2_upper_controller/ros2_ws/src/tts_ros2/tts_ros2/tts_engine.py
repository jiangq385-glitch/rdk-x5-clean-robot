import asyncio
import hashlib
import os
import subprocess
from pathlib import Path
from uuid import uuid4


class TTSEngine:
    def __init__(self, voice: str, rate: str, volume: str, tmp_dir: str, proxy: str = ''):
        self.voice = voice
        self.rate = rate
        self.volume = volume
        self.tmp_dir = Path(tmp_dir)
        self.proxy = self._resolve_proxy(proxy)
        self.tmp_dir.mkdir(parents=True, exist_ok=True)
        self._cache: dict[str, str] = {}

    def _resolve_proxy(self, proxy: str) -> str:
        candidate = str(proxy).strip()
        if candidate:
            return candidate

        for env_name in ('HTTPS_PROXY', 'https_proxy', 'HTTP_PROXY', 'http_proxy', 'ALL_PROXY', 'all_proxy'):
            candidate = os.environ.get(env_name, '').strip()
            if candidate:
                return candidate

        return ''

    async def _synthesize_async(self, text: str, mp3_path: str):
        try:
            import edge_tts
        except ImportError as exc:
            raise RuntimeError('未安装 edge-tts，请先 pip install edge-tts') from exc

        communicate_kwargs = {
            'text': text,
            'voice': self.voice,
            'rate': self.rate,
            'volume': self.volume,
        }
        if self.proxy:
            communicate_kwargs['proxy'] = self.proxy

        communicate = edge_tts.Communicate(
            **communicate_kwargs,
        )
        await communicate.save(mp3_path)

    def synthesize(self, text: str) -> str:
        text = text.strip()
        if not text:
            raise ValueError('text must not be empty')

        cached_path = self._cache.get(text)
        if cached_path and Path(cached_path).is_file():
            return cached_path

        cache_key = hashlib.sha1(
            f'{self.voice}\n{self.rate}\n{self.volume}\n{self.proxy}\n{text}'.encode('utf-8')
        ).hexdigest()
        mp3_path = self.tmp_dir / f'{cache_key}.mp3'
        wav_path = self.tmp_dir / f'{cache_key}.wav'

        if wav_path.is_file():
            self._cache[text] = str(wav_path)
            return str(wav_path)

        asyncio.run(self._synthesize_async(text, str(mp3_path)))

        cmd = [
            'ffmpeg',
            '-y',
            '-i', str(mp3_path),
            '-ar', '16000',
            '-ac', '1',
            '-f', 'wav',
            str(wav_path),
        ]
        completed = subprocess.run(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        if completed.returncode != 0:
            details = (completed.stderr or completed.stdout or '').strip()
            raise RuntimeError(f'ffmpeg convert failed rc={completed.returncode}: {details}')

        try:
            mp3_path.unlink(missing_ok=True)
        except Exception:
            pass

        self._cache[text] = str(wav_path)
        return str(wav_path)

    def warmup(self, texts: list[str]):
        for text in texts:
            cleaned = text.strip()
            if cleaned:
                self.synthesize(cleaned)