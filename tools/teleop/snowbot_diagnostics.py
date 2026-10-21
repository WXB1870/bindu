"""Bounded asynchronous diagnostic writer, outside the control processes."""
import json
import math
from collections import OrderedDict
from pathlib import Path
import queue
import re
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


_ANSI = re.compile(r'\x1b\[[0-?]*[ -/]*[@-~]')
_CONSOLE_LOG = re.compile(
    r'^(?P<prefix>\[[^\]]+\]\s+)?\[(?P<level>WARN(?:ING)?|ERROR|FATAL)\]\s*'
    r'(?:\[(?P<stamp>\d+(?:\.\d+)?)\]\s*)?'
    r'(?:\[(?P<name>[^\]]+)\]\s*:?\s*)?(?P<message>.*)$')
_LEVELS = dict(WARN=30, WARNING=30, ERROR=40, FATAL=50)


def console_log(line, source):
    """Recognize ROS console records only; traceback continuations stay verbatim."""
    match = _CONSOLE_LOG.match(_ANSI.sub('', line).rstrip('\r\n'))
    if not match:return None
    return dict(name=match['name'] or match['prefix'] or source, level=_LEVELS[match['level']],
                message=match['message'], source_stamp=match['stamp'], text=line)


class RepeatedLogs:
    """Bounded exact-message aggregation; silence never means fault recovery."""
    def __init__(self, emit, *, interval=5., capacity=128, idle=30.):
        if interval <= 0 or capacity < 1 or idle < interval:raise ValueError('LOG_REPEAT_BOUNDS')
        self.emit, self.interval, self.capacity, self.idle = emit, interval, capacity, idle
        self.entries = OrderedDict()
        self.lock = threading.Lock()
        self.coalesced = self.evicted = 0

    def _summary(self, entry, reason):
        if entry['pending']:
            value = dict(entry['value'], kind='repeat_summary', suppressed_repeats=entry['pending'],
                         observed_count=entry['count'], first_observed_at=entry['first'],
                         last_observed_at=entry['last'], first_monotonic=entry['first_steady'],
                         last_monotonic=entry['last_steady'], last_source_stamp=entry['last_source_stamp'],
                         summary_reason=reason)
            self.emit(entry['stream'], value, entry['text'])
            entry['pending'] = 0

    def add(self, stream, value, *, text=False, now=None, wall=None):
        now = time.monotonic() if now is None else now
        wall = time.time() if wall is None else wall
        key = (stream, value.get('name'), value.get('level'), value.get('message'),
               value.get('file'), value.get('function'), value.get('line'))
        with self.lock:
            entry = self.entries.get(key)
            if entry is not None and now-entry['last_steady'] >= self.idle:
                self._summary(entry, 'idle');del self.entries[key];entry = None
            if entry is None:
                if len(self.entries) == self.capacity:
                    _, old = self.entries.popitem(last=False)
                    self._summary(old, 'capacity_eviction');self.evicted += 1
                entry = dict(value=dict(value), stream=stream, text=text, count=1, pending=0,
                             first=wall, last=wall, first_steady=now, last_steady=now, emitted=now,
                             last_source_stamp=value.get('source_stamp_ns', value.get('source_stamp')))
                self.entries[key] = entry
                self.emit(stream, dict(value, kind='first', observed_count=1,
                          first_observed_at=wall, last_observed_at=wall,
                          first_monotonic=now, last_monotonic=now), text)
            else:
                self.entries.move_to_end(key)
                entry.update(count=entry['count']+1, pending=entry['pending']+1, last=wall, last_steady=now,
                             last_source_stamp=value.get('source_stamp_ns', value.get('source_stamp')))
                self.coalesced += 1
                if now-entry['emitted'] >= self.interval:
                    self._summary(entry, 'periodic');entry['emitted'] = now

    def flush(self, now=None, *, final=False):
        now = time.monotonic() if now is None else now
        with self.lock:
            for key, entry in list(self.entries.items()):
                if final or now-entry['emitted'] >= self.interval:
                    self._summary(entry, 'close' if final else 'periodic');entry['emitted'] = now
                if final or now-entry['last_steady'] >= self.idle:del self.entries[key]


class RosLogObserver:
    """Observe vendor ROS warnings only; log text is never a control/health input."""
    LOGGERS = ('servo_node', 'dual_arm_control_node', 'robot_upperMonitor')

    def __init__(self, node, writer, run_id):
        from rcl_interfaces.msg import Log
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        self.writer, self.run_id = writer, run_id
        self.started_ns = time.time_ns()
        self.received = 0
        # A volatile, best-effort observer must not request history or backpressure publishers.
        qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self.subscription = node.create_subscription(Log, '/rosout', self.receive, qos)
        writer.record('rosout-log-events', dict(kind='capture_started', run_id=run_id,
                      loggers=self.LOGGERS, minimum_level=Log.WARN, scope='observed_rosout_only',
                      reliability='best_effort', durability='volatile', queue_depth=100))

    def receive(self, msg):
        if msg.level < 30 or not any(msg.name == name or msg.name.startswith(name+'.') for name in self.LOGGERS):return
        self.received += 1
        stamp_ns = msg.stamp.sec*1000000000+msg.stamp.nanosec
        self.writer.record_log('rosout-log-events', dict(name=msg.name, level=int(msg.level), message=msg.msg,
            file=msg.file, function=msg.function, line=int(msg.line), source_stamp_ns=stamp_ns,
            source_stamp_before_capture=stamp_ns < self.started_ns, run_id=self.run_id, origin='rosout'))

    def status(self):
        return dict(scope='observed_rosout_warn_plus', received=self.received,
                    loggers=self.LOGGERS, plain_stdio_captured=False)


class DiagnosticWriter:
    def __init__(self, path, *, capacity=8192, chunk_bytes=128*1024*1024, sampling=True,
                 health_filename='diagnostics-health.json', echo=False):
        self.path=Path(path);self.path.parent.mkdir(parents=True,exist_ok=True)
        self.queue=queue.Queue(capacity);self.chunk_bytes=chunk_bytes
        self.stop=threading.Event();self.error='';self.dropped=0;self.written=0
        self.sampler=DiagnosticSampler() if sampling else None
        self.repeats=RepeatedLogs(self._emit_log)
        self.health_filename, self.echo, self.console_error = health_filename, echo, ''
        self.thread=threading.Thread(target=self.run,name='diagnostic-writer',daemon=True)
        self.thread.start()

    def record(self, stream, value, *, ros=False):
        if self.error or self.stop.is_set():
            self.dropped+=1;return
        if self.sampler and not self.sampler.keep(stream,value,time.monotonic()):return
        self._enqueue(stream, value, ros)

    def _enqueue(self, stream, value, mode):
        if self.error or self.stop.is_set():self.dropped+=1;return
        item=(stream,dict(received_at=time.time(),received_monotonic_ns=time.monotonic_ns()),value,mode)
        try:self.queue.put_nowait(item)
        except queue.Full:self.dropped+=1

    def record_log(self, stream, value, *, text=False):
        if self.error or self.stop.is_set():self.dropped+=1;return
        self.repeats.add(stream, value, text=text)

    def write_text(self, stream, line):
        self._enqueue(stream, line, 'text')

    def write_json_line(self, stream, value):
        self._enqueue(stream, value, 'json_line')

    def _emit_log(self, stream, value, text):
        if text:
            line = value['text'] if value['kind'] == 'first' else '[LOG_REPEAT] '+json.dumps(
                {k:v for k,v in value.items() if k != 'text'}, ensure_ascii=False)+'\n'
            self.write_text(stream, line)
        else:self.record(stream, value)

    def status(self):
        return dict(active=self.thread.is_alive() and not self.error and not self.stop.is_set(),dropped=self.dropped,
                    written=self.written,queued=self.queue.qsize(),error=self.error,
                    sampled_out=self.sampler.sampled_out if self.sampler else 0,
                    sampling='10Hz_state_changes_1Hz_health' if self.sampler else 'all_received',
                    coalesced=self.repeats.coalesced, repeat_keys=len(self.repeats.entries),
                    repeat_evicted=self.repeats.evicted, console_error=self.console_error)

    def run(self):
        handles={};sizes={};chunks={};last_flush=time.monotonic();last_repeat=last_flush
        def flush():
            for handle in handles.values():handle.flush()
            if self.echo:
                try:sys.stdout.flush()
                except (OSError,ValueError) as exc:self.console_error=str(exc);self.echo=False
            health=self.path.parent/self.health_filename
            temporary=health.with_suffix('.tmp');temporary.write_text(json.dumps(dict(self.status(),stamp=time.time())))
            temporary.replace(health)
        try:
            while not self.stop.is_set() or not self.queue.empty():
                if not self.stop.is_set() and time.monotonic()-last_repeat >= 1.:
                    self.repeats.flush();last_repeat=time.monotonic()
                try:stream,envelope,value,mode=self.queue.get(timeout=.1)
                except queue.Empty:stream=None
                if stream is not None:
                    if mode is True:
                        from rosidl_runtime_py.convert import message_to_ordereddict
                        value=message_to_ordereddict(value)
                    if mode == 'text':line=value
                    elif mode == 'json_line':line=json.dumps(finite_json(value),ensure_ascii=False,allow_nan=False)+'\n'
                    else:
                        envelope['data']=finite_json(value)
                        line=json.dumps(envelope,ensure_ascii=False,allow_nan=False)+'\n'
                    if stream not in handles or sizes[stream]>=self.chunk_bytes:
                        if stream in handles:handles[stream].close()
                        index=chunks.get(stream,0);chunks[stream]=index+1
                        base=self.path if stream=='diagnostics' else self.path.with_name(stream+('.log' if mode=='text' else '.jsonl'))
                        target=base if index==0 else base.with_name(f'{base.stem}-{index:04d}{base.suffix}')
                        handles[stream]=target.open('a',buffering=65536);sizes[stream]=target.stat().st_size
                    handles[stream].write(line);sizes[stream]+=len(line.encode());self.written+=1
                    if mode == 'text' and self.echo:
                        try:sys.stdout.write(line)
                        except (OSError,ValueError) as exc:self.console_error=str(exc);self.echo=False
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
        self.repeats.flush(final=True)
        self.stop.set();self.thread.join(timeout=5.)
        if self.thread.is_alive():raise RuntimeError('DIAGNOSTICS_FLUSH_TIMEOUT')
