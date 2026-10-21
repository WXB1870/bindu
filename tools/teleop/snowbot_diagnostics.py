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


def _field(value, name, default=None):
    return value.get(name, default) if isinstance(value, dict) else getattr(value, name, default)


def _fields(value, names):
    return tuple(_field(value, name) for name in names.split())


def _sequence(value, name):
    result = _field(value, name)
    return tuple(result) if result is not None else ()


def _analog_state(value):
    # Preserve the actual input contract's release/engage threshold crossings.
    if not isinstance(value, (int, float)) or not math.isfinite(value):return 'invalid'
    return (value < .3, value > .7)


def sampling_policy(stream, value):
    """Small state keys only; do not serialize ROS arrays before sampling."""
    key = stream
    period = .1
    state = None
    if stream in ('vr-raw', 'vr-projected'):
        # A reconnect changes state, rather than creating unbounded sampler keys.
        side = _field(value, 'side')
        key = (stream, side if side in ('left', 'right') else 'unknown')
        state = (_fields(value, 'source_id runtime_mode valid init stop clutch_seq mapping buttons virtual_grip'),
                 _sequence(value, 'button_edges'),
                 _analog_state(_field(value, 'grip')), _analog_state(_field(value, 'trigger')))
    elif stream == 'execution-state':
        state = (_fields(value, 'instance_id state code lease_active lease_epoch'),
                 _sequence(value, 'missing_feedback_groups'),
                 _sequence(value, 'stop_failures'))
    elif stream == 'teleop-state':
        state = (_fields(value, 'source_id generation phase reason teleop_mode load_state pending recording'),
                 tuple((_fields(hand, 'valid buttons'), _sequence(hand, 'button_edges'),
                        _analog_state(hand.get('squeeze')), _analog_state(hand.get('trigger')))
                       for hand in (value.get('controller_states', {}).get(side, {}) for side in ('left', 'right'))))
    elif stream == 'gripper-state':
        side = _field(value, 'side')
        key = (stream, side if side in ('left', 'right') else 'unknown')
        state = _fields(value, 'instance_id state code calibrated feedback_valid lease_active owner epoch loading_authorized trigger_armed position_reached grasp_state')
    elif stream == 'mode-state':
        state = _fields(value, 'generation mode side phase code')
    elif stream in ('vr-health', 'physics-health', 'pdo-health'):
        period = 1.
        state = _fields(value, 'ready valid code reason reasons fault_mask valid_mask operation_enabled_mask')
    elif stream == 'diagnostics':
        state = _fields(value, 'valid execution code')
    elif stream == 'runtime-events':
        event = _field(value, 'state')
        # Keep command/session mapping and all other discrete events in full.
        if event not in ('TELEOP_IK_RESULT', 'BIMANUAL_QP_RESULT'):return 0., key, None
        try:
            code = json.loads(_field(value, 'code', ''))
            state = (event, code.get('code'))
        except (ValueError, TypeError, AttributeError):return 0., key, None
    else:
        return 0., key, None
    return period, key, state


class DiagnosticSampler:
    def __init__(self):
        self.last = {}
        self.sampled_out = 0

    def keep(self, stream, value, now):
        period, key, state = sampling_policy(stream, value)
        if not period:return True
        previous = self.last.get(key)
        if previous and previous[1] == state and 0 <= now-previous[0] < period:
            self.sampled_out += 1
            return False
        self.last[key] = (now, state)
        return True


class DiagnosticWriter:
    def __init__(self, path, *, capacity=8192, chunk_bytes=128*1024*1024, sampling=True):
        self.path=Path(path);self.path.parent.mkdir(parents=True,exist_ok=True)
        self.queue=queue.Queue(capacity);self.chunk_bytes=chunk_bytes
        self.stop=threading.Event();self.error='';self.dropped=0;self.written=0
        self.sampler=DiagnosticSampler() if sampling else None
        self.thread=threading.Thread(target=self.run,name='diagnostic-writer',daemon=True)
        self.thread.start()

    def record(self, stream, value, *, ros=False):
        if self.error or self.stop.is_set():
            self.dropped+=1;return
        if self.sampler and not self.sampler.keep(stream,value,time.monotonic()):return
        item=(stream,dict(received_at=time.time(),received_monotonic_ns=time.monotonic_ns()),value,ros)
        try:self.queue.put_nowait(item)
        except queue.Full:self.dropped+=1

    def status(self):
        return dict(active=self.thread.is_alive() and not self.error and not self.stop.is_set(),dropped=self.dropped,
                    written=self.written,queued=self.queue.qsize(),error=self.error,
                    sampled_out=self.sampler.sampled_out if self.sampler else 0,
                    sampling='10Hz_state_changes_1Hz_health' if self.sampler else 'all_received')

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
