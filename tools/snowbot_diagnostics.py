"""Bounded asynchronous diagnostic writer, outside the control processes."""
import json
import math
from pathlib import Path
import queue
import sys
import threading
import time


def finite_json(value):
    """Keep malformed numeric input inspectable without invalid JSON numbers."""
    if isinstance(value, float) and not math.isfinite(value):return str(value)
    if isinstance(value, dict):return {k:finite_json(v) for k,v in value.items()}
    if isinstance(value, (list,tuple)):return [finite_json(v) for v in value]
    return value


class DiagnosticWriter:
    def __init__(self, path, *, capacity=8192, chunk_bytes=128*1024*1024):
        self.path=Path(path);self.path.parent.mkdir(parents=True,exist_ok=True)
        self.queue=queue.Queue(capacity);self.chunk_bytes=chunk_bytes
        self.stop=threading.Event();self.error='';self.dropped=0;self.written=0
        self.thread=threading.Thread(target=self.run,name='diagnostic-writer',daemon=True)
        self.thread.start()

    def record(self, stream, value, *, ros=False):
        if self.error or self.stop.is_set():
            self.dropped+=1;return
        item=(stream,dict(received_at=time.time(),received_monotonic_ns=time.monotonic_ns()),value,ros)
        try:self.queue.put_nowait(item)
        except queue.Full:self.dropped+=1

    def status(self):
        return dict(active=self.thread.is_alive() and not self.error and not self.stop.is_set(),dropped=self.dropped,
                    written=self.written,queued=self.queue.qsize(),error=self.error)

    def run(self):
        handles={};sizes={};chunks={};last_flush=time.monotonic()
        def flush():
            for handle in handles.values():handle.flush()
            health=self.path.parent/'diagnostics-health.json'
            temporary=health.with_suffix('.tmp');temporary.write_text(json.dumps(dict(self.status(),stamp=time.time())))
            temporary.replace(health)
        try:
            while not self.stop.is_set() or not self.queue.empty():
                try:stream,envelope,value,ros=self.queue.get(timeout=.1)
                except queue.Empty:stream=None
                if stream is not None:
                    if ros:
                        from rosidl_runtime_py.convert import message_to_ordereddict
                        value=message_to_ordereddict(value)
                    envelope['data']=finite_json(value)
                    line=json.dumps(envelope,ensure_ascii=False,allow_nan=False)+'\n'
                    if stream not in handles or sizes[stream]>=self.chunk_bytes:
                        if stream in handles:handles[stream].close()
                        index=chunks.get(stream,0);chunks[stream]=index+1
                        base=self.path if stream=='diagnostics' else self.path.with_name(stream+'.jsonl')
                        target=base if index==0 else base.with_name(f'{base.stem}-{index:04d}.jsonl')
                        handles[stream]=target.open('a',buffering=65536);sizes[stream]=target.stat().st_size
                    handles[stream].write(line);sizes[stream]+=len(line.encode());self.written+=1
                if time.monotonic()-last_flush>=1.:
                    flush();last_flush=time.monotonic()
        except Exception as exc:
            self.error=type(exc).__name__+':'+str(exc)
            print('DIAGNOSTICS_FAILED '+self.error,file=sys.stderr,flush=True)
        finally:
            try:flush()
            finally:
                for handle in handles.values():handle.close()

    def close(self):
        self.stop.set();self.thread.join(timeout=5.)
        if self.thread.is_alive():raise RuntimeError('DIAGNOSTICS_FLUSH_TIMEOUT')
