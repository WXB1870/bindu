import json
import gzip
import time
from pathlib import Path
from queue import Queue, Empty, Full
from threading import Thread


class AsyncRecorder:
    """Bounded nonblocking ingress; disk I/O occurs only in the writer thread."""
    def __init__(self, directory, manifest, capacity=4096, *, mode='normal'):
        if mode not in ('normal', 'compact', 'off'):
            raise ValueError('INVALID_RECORDING_MODE')
        self.mode = mode
        self.directory = Path(directory)
        self.queue = Queue(maxsize=capacity)
        self.dropped = self.written = 0
        self.sampled_out = 0
        self._last = {}
        self.error = ''
        self.closing = False
        self.worker = None
        if mode == 'off':
            return
        self.directory.mkdir(parents=True, exist_ok=False)
        (self.directory / 'manifest.json').write_text(json.dumps({**manifest,
            'recording_mode': mode, 'data_file': 'episode.jsonl.gz',
            'sampling': 'feedback/vr_input at most 10 Hz plus state changes; routine success events omitted'
                        if mode == 'compact' else 'all received messages'}, indent=2))
        self.worker = Thread(target=self._write, daemon=True)
        self.worker.start()

    def _keep(self, record):
        if self.mode != 'compact':
            return True
        kind, data = record.get('kind'), record.get('data', {})
        if kind == 'event':
            if data.get('state') in ('ACCEPTED', 'SUPERSEDED'):
                return False  # Full accepted commands remain in 'reference'.
            if data.get('state') == 'TELEOP_IK_RESULT':
                try:
                    if json.loads(data['code']).get('code') == 'OK':
                        return False
                except (ValueError, TypeError, KeyError, AttributeError):
                    pass  # Keep malformed diagnostics for investigation.
        if kind in ('feedback', 'vr_input'):
            now = record['received_at']
            key = (kind, data.get('instance_id'), data.get('source_id'), data.get('side'))
            flags = tuple(data.get(k) for k in ('state', 'code', 'valid', 'init', 'stop', 'clutch_seq'))
            previous = self._last.get(key)
            if previous and previous[1] == flags and 0 <= now-previous[0] < .1:
                return False
            self._last[key] = (now, flags)
        return True

    def submit(self, record):
        if self.closing or self.error:
            self.dropped += 1
            return False
        if self.mode == 'off' or not self._keep(record):
            self.sampled_out += 1
            return True
        try:
            self.queue.put_nowait(record)
            return True
        except Full:
            self.dropped += 1
            return False

    def _write(self):
        try:
            with gzip.open(self.directory / 'episode.jsonl.gz', 'wt', encoding='utf-8', compresslevel=1) as stream:
                last_flush = time.monotonic()
                flushed = 0
                while not self.closing or not self.queue.empty():
                    if self.written != flushed and time.monotonic()-last_flush >= .5:
                        stream.flush()
                        last_flush, flushed = time.monotonic(), self.written
                    try:
                        record = self.queue.get(timeout=.05)
                    except Empty:
                        continue
                    stream.write(json.dumps(record, allow_nan=False, ensure_ascii=False, separators=(',', ':')) + '\n')
                    self.written += 1
        except (OSError, ValueError) as exc:
            self.error = type(exc).__name__

    def close(self):
        self.closing = True
        if self.worker is None:
            return
        self.worker.join(timeout=3)
        if self.worker.is_alive():
            self.error = 'WRITER_SHUTDOWN_TIMEOUT'
        (self.directory / 'summary.json').write_text(json.dumps({
            'written': self.written, 'dropped': self.dropped, 'error': self.error,
            'recording_mode': self.mode, 'sampled_out': self.sampled_out,
            'coverage': 'selected_received_messages' if self.mode == 'compact' else 'received_messages_only',
            'writer_complete': not self.error and not self.dropped and self.queue.empty()}))
