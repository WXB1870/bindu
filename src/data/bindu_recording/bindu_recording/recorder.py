import json
from pathlib import Path
from queue import Queue, Empty, Full
from threading import Thread


class AsyncRecorder:
    """Bounded nonblocking ingress; disk I/O occurs only in the writer thread."""
    def __init__(self, directory, manifest, capacity=4096):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=False)
        (self.directory / 'manifest.json').write_text(json.dumps(manifest, indent=2))
        self.queue = Queue(maxsize=capacity)
        self.dropped = self.written = 0
        self.error = ''
        self.closing = False
        self.worker = Thread(target=self._write, daemon=True)
        self.worker.start()

    def submit(self, record):
        if self.closing or self.error:
            self.dropped += 1
            return False
        try:
            self.queue.put_nowait(record)
            return True
        except Full:
            self.dropped += 1
            return False

    def _write(self):
        try:
            with (self.directory / 'episode.jsonl').open('w') as stream:
                while not self.closing or not self.queue.empty():
                    try:
                        record = self.queue.get(timeout=.05)
                    except Empty:
                        continue
                    stream.write(json.dumps(record, allow_nan=False) + '\n')
                    self.written += 1
                    stream.flush()
        except (OSError, ValueError) as exc:
            self.error = type(exc).__name__

    def close(self):
        self.closing = True
        self.worker.join(timeout=3)
        if self.worker.is_alive():
            self.error = 'WRITER_SHUTDOWN_TIMEOUT'
        (self.directory / 'summary.json').write_text(json.dumps({
            'written': self.written, 'dropped': self.dropped, 'error': self.error,
            'coverage': 'received_messages_only',
            'writer_complete': not self.error and not self.dropped and self.queue.empty()}))
