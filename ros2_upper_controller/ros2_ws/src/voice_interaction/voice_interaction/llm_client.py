import base64
import json
import mimetypes
import urllib.error
import urllib.request
from pathlib import Path


class LLMClient:
    def generate(self, system_prompt: str, user_text: str) -> str:
        raise NotImplementedError


class CloudAPIClient(LLMClient):
    def __init__(
        self,
        api_base_url: str,
        api_key: str,
        model: str,
        timeout_sec: float,
        num_predict: int,
        temperature: float,
        enable_vision: bool = False,
        image_path: str = '',
    ):
        self.api_base_url = api_base_url.rstrip('/')
        self.api_key = api_key.strip()
        self.model = model.strip()
        self.timeout_sec = timeout_sec
        self.num_predict = num_predict
        self.temperature = temperature
        self.enable_vision = enable_vision
        self.image_path = image_path.strip()

    def _load_image_data_url(self) -> str:
        if not self.enable_vision or not self.image_path:
            return ''

        path = Path(self.image_path)
        if not path.exists() or not path.is_file():
            return ''

        mime_type, _ = mimetypes.guess_type(path.name)
        if not mime_type:
            mime_type = 'image/jpeg'

        raw = path.read_bytes()
        encoded = base64.b64encode(raw).decode('utf-8')
        return f'data:{mime_type};base64,{encoded}'

    def _build_user_message(self, user_text: str):
        image_data_url = self._load_image_data_url()
        if not image_data_url:
            return user_text

        return [
            {'type': 'text', 'text': user_text},
            {'type': 'image_url', 'image_url': {'url': image_data_url}},
        ]

    def generate(self, system_prompt: str, user_text: str) -> str:
        if not self.api_base_url:
            raise RuntimeError('云端模型缺少 api_base_url 配置')
        if not self.api_key:
            raise RuntimeError('云端模型缺少 api_key 配置')
        if not self.model:
            raise RuntimeError('云端模型缺少 model 配置')

        payload = {
            'model': self.model,
            'messages': [
                {'role': 'system', 'content': system_prompt},
                {'role': 'user', 'content': self._build_user_message(user_text)},
            ],
            'max_tokens': self.num_predict,
            'temperature': self.temperature,
            'stream': False,
        }
        data = json.dumps(payload).encode('utf-8')
        req = urllib.request.Request(
            url=f'{self.api_base_url}/chat/completions',
            data=data,
            headers={
                'Content-Type': 'application/json',
                'Authorization': f'Bearer {self.api_key}',
            },
            method='POST',
        )

        opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))

        try:
            with opener.open(req, timeout=self.timeout_sec) as resp:
                body = resp.read().decode('utf-8')
        except urllib.error.HTTPError as e:
            err = e.read().decode('utf-8', errors='ignore')
            raise RuntimeError(f'云端模型 HTTPError: {e.code} {err}') from e
        except urllib.error.URLError as e:
            raise RuntimeError(f'无法连接云端模型: {e}') from e

        result = json.loads(body)
        choices = result.get('choices', [])
        if not choices:
            raise RuntimeError(f'云端模型返回为空: {body}')

        text = choices[0].get('message', {}).get('content', '')
        if isinstance(text, list):
            text = ''.join(item.get('text', '') for item in text if isinstance(item, dict))
        text = str(text).strip()
        if not text:
            raise RuntimeError(f'云端模型返回内容为空: {body}')
        return text
