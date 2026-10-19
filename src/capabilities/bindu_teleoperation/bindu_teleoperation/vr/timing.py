"""Diagnostic browser clocks are untrusted and never determine control freshness."""
import math


class ControllerTiming:
    def __init__(self):
        self.previous = None

    def observe(self, value, received_at, received_monotonic, source_id, seq):
        report = dict(stage='controller_timing', stamp=received_at, source_id=source_id, seq=seq,
                      server_monotonic_s=received_monotonic, client_timing_available=False)
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
