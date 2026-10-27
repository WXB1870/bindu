"""Diagnostic browser clocks are untrusted and never determine control freshness."""
import math
import time
from collections import deque


class WebSocketTiming:
    """Optional observation of complete WS messages, never kernel packet arrival."""
    def __init__(self, websocket):
        self.websocket = websocket
        self.last_message = None
        self.sequence = 0

    def __getattr__(self, name):
        return getattr(self.websocket, name)

    def __aiter__(self):
        return self

    async def __anext__(self):
        started = time.monotonic()
        message = await self.websocket.__anext__()
        received = time.monotonic()
        self.sequence += 1
        self.last_message = dict(message_seq=self.sequence, received_monotonic_s=received,
            read_wait_s=received-started,
            payload_bytes=len(message.data) if isinstance(message.data, bytes) else None)
        return message


class TransportWindow:
    """Eight preceding reports and four recovery frames; no extra normal traffic."""
    def __init__(self):
        self.history = deque(maxlen=8)
        self.source = None
        self.remaining = 0
        self.trigger_seq = None

    def observe(self, report):
        if report['source_id'] != self.source:
            self.history.clear()
            self.remaining = 0
            self.source = report['source_id']
        gap = report.get('server_controller_interval_s', 0.) > .1
        if gap:
            samples = list(self.history)+[report]
            self.trigger_seq = report['seq']
            self.remaining = 4
        elif self.remaining:
            samples = [report]
            self.remaining -= 1
        else:
            self.history.append(report)
            return None
        self.history.append(report)
        return dict(stage='transport_window', stamp=report['stamp'], source_id=self.source,
            seq=report['seq'], trigger_seq=self.trigger_seq, phase='gap' if gap else 'recovery',
            remaining_recovery_frames=self.remaining, samples=samples,
            clock_semantics='host_monotonic; client intervals only; not packet arrival')


class InputPipelineTiming:
    """Optional one-Hz bounded summaries; never used for control freshness."""
    def __init__(self):
        self.previous_poll=self.reported=self.started=self.last_frame=None
        self.polls=self.frames=0
        self.published=0
        self.maxima={}
        self.latest=None

    def maximum(self,name,value,identity=None):
        if math.isfinite(value) and value>=0 and value>self.maxima.get(name,{}).get('seconds',-1.):
            self.maxima[name]=dict(seconds=value,**(identity or {}))

    def begin(self,now):
        if self.reported is None:self.reported=now
        if self.previous_poll is not None:self.maximum('ros_poll_interval',now-self.previous_poll)
        self.previous_poll=self.started=now
        self.polls+=1

    def finish(self,now,delivery,published_messages=0):
        self.published+=published_messages
        self.maximum('ros_poll_callback',now-self.started)
        if delivery:
            self.frames+=1;self.last_frame=delivery['dequeued']
            identity=dict(source_id=delivery['source_id'],seq=delivery['seq'])
            self.latest=identity
            self.maximum('controller_handler',delivery['frame_ready']-delivery['handler_started'],identity)
            if delivery.get('dispatched') is not None:
                self.maximum('server_task_dispatch',delivery['handler_started']-delivery['dispatched'],identity)
                websocket=delivery.get('websocket')
                if websocket:
                    self.maximum('websocket_read_wait',websocket['read_wait_s'],identity)
                    self.maximum('websocket_to_dispatch',
                        delivery['dispatched']-websocket['received_monotonic_s'],identity)
            self.maximum('ros_after_dequeue',now-delivery['dequeued'],identity)
            if delivery['shared_monotonic']:
                self.maximum('frame_queue_wait',delivery['dequeued']-delivery['frame_ready'],identity)
                self.maximum('handler_to_ros_poll_end',now-delivery['handler_started'],identity)
        if self.reported is None:self.reported=now
        if now-self.reported<1.:return None
        report=dict(stage='input_pipeline',clock='host_monotonic_diagnostic_only',
            poll_count=self.polls,frame_count=self.frames,published_messages=self.published,maxima=self.maxima,
            last_dequeued_age_s=None if self.last_frame is None else now-self.last_frame,
            latest=self.latest,window_s=now-self.reported)
        self.reported=now;self.polls=self.frames=self.published=0;self.maxima={}
        return report


class ControllerTiming:
    def __init__(self):
        self.previous = None

    def observe(self, value, received_at, received_monotonic, source_id, seq):
        report = dict(stage='controller_timing', stamp=received_at, source_id=source_id, seq=seq,
                      server_monotonic_s=received_monotonic, client_timing_available=False,
                      stamp_semantics='controller_handler_entry_wall_clock')
        required = ('version', 'seq', 'perf_ms', 'wall_ms', 'buffered_bytes', 'loop_max_ms',
                    'raf_max_ms', 'xr_max_ms', 'long_task_max_ms', 'long_task_count')
        valid = (isinstance(value, dict) and value.get('version') == 1 and all(
            type(value.get(k)) in (int, float) and math.isfinite(value[k]) and value[k] >= 0
            for k in required))
        if valid:
            report['client_timing_available'] = True
            report['client'] = {k: value[k] for k in required}
            for k in ('page_visibility', 'xr_visibility'):
                if isinstance(value.get(k), str):report['client'][k] = value[k][:32]
            previous_send=value.get('previous_send')
            fields=('seq','prepare_ms','send_call_ms','buffered_after_bytes','wire_bytes')
            if isinstance(previous_send,dict) and all(type(previous_send.get(k)) in (int,float)
                    and math.isfinite(previous_send[k]) and previous_send[k]>=0 for k in fields) and (
                    previous_send['seq']<value['seq']):
                report['client']['previous_send']={k:previous_send[k] for k in fields}
        previous = self.previous
        if previous and previous[0] == source_id:
            report['server_controller_interval_s'] = received_monotonic - previous[1]
            if valid and previous[2] is not None:
                interval = (value['perf_ms'] - previous[2]) * .001
                if interval > 0:
                    report['client_controller_interval_s'] = interval
                    # Difference of intervals, not one-way network latency:
                    # headset and server clocks are deliberately not synchronized.
                    report['transport_interval_excess_s'] = report['server_controller_interval_s'] - interval
        self.previous = (source_id, received_monotonic, value['perf_ms'] if valid else None)
        return report
