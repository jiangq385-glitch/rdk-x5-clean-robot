#!/usr/bin/env python3
import argparse
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

from .llm_client import CloudAPIClient


DEFAULT_API_BASE_URL = 'https://ark.cn-beijing.volces.com/api/v3'
DEFAULT_MODEL = 'doubao-seed-2-0-mini-260428'
DEFAULT_API_KEY = 'ark-f9a1b4f7-50f5-4f43-9fc4-cf8b7cc3d320-7e34b'
DEFAULT_PROMPT = '请用中文简短描述这张图片里的主要内容，并判断画面是否正常可识别。'


def _latest_dump(output_dir: Path, started_at: float) -> Path | None:
    candidates = []
    for path in output_dir.glob('dump_codec_output*.jpeg'):
        try:
            stat = path.stat()
        except OSError:
            continue
        if stat.st_mtime >= started_at - 1.0 and stat.st_size > 0:
            candidates.append((stat.st_mtime, path))
    if not candidates:
        return None
    return max(candidates, key=lambda item: item[0])[1]


def _terminate(proc: subprocess.Popen) -> None:
    if proc.poll() is not None:
        return
    try:
        proc.send_signal(signal.SIGINT)
        proc.wait(timeout=5)
    except Exception:
        proc.terminate()
        try:
            proc.wait(timeout=3)
        except Exception:
            proc.kill()


def _cleanup_old_capture_container() -> None:
    pattern = 'component_container_isolated.*__node:=tros_container'
    found = subprocess.run(
        ['pgrep', '-f', pattern],
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
        check=False,
    )
    pids = [pid for pid in found.stdout.split() if pid.isdigit()]
    if not pids:
        return

    print(f'[vision_test] cleaning old capture container: {" ".join(pids)}', flush=True)
    subprocess.run(['pkill', '-INT', '-f', pattern], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False)
    deadline = time.time() + 3.0
    while time.time() < deadline:
        still_running = subprocess.run(
            ['pgrep', '-f', pattern],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )
        if still_running.returncode != 0:
            return
        time.sleep(0.2)

    subprocess.run(['pkill', '-KILL', '-f', pattern], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False)
    time.sleep(0.5)


def capture_frame(output_dir: Path, timeout_sec: float) -> Path:
    output_dir.mkdir(parents=True, exist_ok=True)
    started_at = time.time()
    cmd = [
        'ros2',
        'launch',
        'hobot_codec',
        'hobot_mipi_encoder_component.launch.py',
        'codec_out_format:=jpeg',
        'codec_pub_topic:=/image_jpeg',
        'codec_dump_output:=True',
        'codec_dump_frame_count:=1',
    ]

    _cleanup_old_capture_container()
    print('[vision_test] starting MIPI capture...', flush=True)
    proc = subprocess.Popen(
        cmd,
        cwd=str(output_dir),
        stdout=None,
        stderr=None,
    )

    deadline = time.time() + timeout_sec
    try:
        while time.time() < deadline:
            image_path = _latest_dump(output_dir, started_at)
            if image_path:
                print(f'[vision_test] captured: {image_path}', flush=True)
                return image_path

            if proc.poll() is not None:
                raise RuntimeError(f'capture process exited early with code {proc.returncode}')

            time.sleep(0.2)
    finally:
        _terminate(proc)
        _cleanup_old_capture_container()

    raise TimeoutError(f'no JPEG frame dumped within {timeout_sec:.1f}s')


def ask_llm(args: argparse.Namespace, image_path: Path) -> str:
    client = CloudAPIClient(
        api_base_url=args.api_base_url,
        api_key=args.api_key,
        model=args.model,
        timeout_sec=args.llm_timeout,
        num_predict=args.max_tokens,
        temperature=args.temperature,
        enable_vision=True,
        image_path=str(image_path),
    )
    return client.generate(args.system_prompt, args.prompt)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='Capture one MIPI frame and test multimodal LLM vision.')
    parser.add_argument('--output-dir', default=os.environ.get('VISION_TEST_OUTPUT_DIR', '/home/sunrise'))
    parser.add_argument('--capture-timeout', type=float, default=30.0)
    parser.add_argument('--api-base-url', default=os.environ.get('LLM_API_BASE_URL', DEFAULT_API_BASE_URL))
    parser.add_argument('--api-key', default=os.environ.get('LLM_API_KEY', DEFAULT_API_KEY))
    parser.add_argument('--model', default=os.environ.get('LLM_MODEL', DEFAULT_MODEL))
    parser.add_argument('--llm-timeout', type=float, default=90.0)
    parser.add_argument('--max-tokens', type=int, default=256)
    parser.add_argument('--temperature', type=float, default=0.2)
    parser.add_argument('--system-prompt', default='你是机器人视觉测试助手。')
    parser.add_argument('--prompt', default=DEFAULT_PROMPT)
    args, _ = parser.parse_known_args()
    return args


def main() -> int:
    args = parse_args()
    if not args.api_key:
        print('[vision_test] missing api key: set LLM_API_KEY or pass --api-key', file=sys.stderr)
        return 2

    try:
        image_path = capture_frame(Path(args.output_dir), args.capture_timeout)
        print('[vision_test] asking LLM...', flush=True)
        reply = ask_llm(args, image_path)
    except Exception as exc:
        print(f'[vision_test] failed: {exc}', file=sys.stderr)
        return 1

    print('\n========== LLM RESULT ==========')
    print(reply)
    print('================================')
    print(f'[vision_test] image: {image_path}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
