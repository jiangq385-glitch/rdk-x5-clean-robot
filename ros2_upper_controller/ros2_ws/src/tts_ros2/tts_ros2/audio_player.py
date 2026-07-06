import subprocess
from pathlib import Path


class AudioPlayer:
    def __init__(self, output_device: str = 'default'):
        self.output_device = output_device

    def play(self, audio_path: str):
        suffix = Path(audio_path).suffix.lower()

        if suffix == '.mp3':
            cmd = ['ffplay', '-nodisp', '-autoexit', '-loglevel', 'quiet', audio_path]
        else:
            cmd = ['aplay']
            if self.output_device != 'default':
                cmd += ['-D', self.output_device]
            cmd += [audio_path]

        completed = subprocess.run(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        if completed.returncode != 0:
            details = (completed.stderr or completed.stdout or '').strip()
            raise RuntimeError(f'Audio player failed rc={completed.returncode}: {details}')