"""Diagnostic browser clocks are untrusted and never determine control freshness."""
import math


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
