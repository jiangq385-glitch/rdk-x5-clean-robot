from collections import deque


class AudioBuffer:
    def __init__(self, max_bytes: int):
        if max_bytes <= 0:
            raise ValueError('max_bytes must be positive')
        self.max_bytes = max_bytes
        self._chunks = deque()
        self._size = 0

    def __len__(self) -> int:
        return self._size

    def clear(self):
        self._chunks.clear()
        self._size = 0

    def append(self, data: bytes):
        if not data:
            return

        self._chunks.append(data)
        self._size += len(data)
        self._trim_to_max()

    def drop_prefix(self, size: int):
        remaining = max(0, int(size))
        while remaining > 0 and self._chunks:
            chunk = self._chunks[0]
            if len(chunk) <= remaining:
                self._chunks.popleft()
                self._size -= len(chunk)
                remaining -= len(chunk)
                continue

            self._chunks[0] = chunk[remaining:]
            self._size -= remaining
            remaining = 0

    def get_bytes(self) -> bytes:
        return b''.join(self._chunks)

    def _trim_to_max(self):
        overflow = self._size - self.max_bytes
        if overflow > 0:
            self.drop_prefix(overflow)
