"""Validate the existing SOEM bridge's measured 48-channel ROS feedback.

This module has no command publisher or hardware access. The ROS wrapper may
forward a frame only while the bridge status and the original frame are fresh.
"""
import math

from .position_mapping import PositionMapping


class SoemFeedbackGate:
    def __init__(self, config, max_age=0.2, future_tolerance=0.05):
        if not 0 < max_age <= 1 or not 0 <= future_tolerance <= max_age:
            raise ValueError('SOEM_FEEDBACK_TIME_CONFIG')
        self.mapping = PositionMapping(config, config['joints'])
        self.max_age = max_age
        self.future_tolerance = future_tolerance
        self.status_time = None
        self.status_healthy = False
        self.sample = None
        self.last_stamp = 0.
        self.last_published_stamp = 0.

    def observe_status(self, report, received_at):
        self.status_healthy = False
        if not math.isfinite(received_at):
            raise ValueError('SOEM_STATUS_TIME')
        fields = {}
        for token in report.split():
            if '=' in token:
                key, value = token.split('=', 1)
                if key in fields:
                    self.sample = None
                    raise ValueError('SOEM_STATUS_DUPLICATE')
                fields[key] = value
        self.status_time = received_at
        self.status_healthy = all(fields.get(k) == v for k, v in (
            ('alive', '1'), ('ready', '1'), ('error', '0'), ('hand_held_mask', '0')))
        if not self.status_healthy:
            self.sample = None

    def observe_joint_state(self, names, positions, stamp, received_at):
        if (not isinstance(stamp, (int, float)) or not math.isfinite(stamp) or
                not isinstance(received_at, (int, float)) or not math.isfinite(received_at) or
                stamp <= 0 or stamp <= self.last_stamp or
                stamp - received_at > self.future_tolerance or
                received_at - stamp > self.max_age or
                len(names) != len(positions) or len(set(names)) != len(names) or
                set(names) != set(self.mapping.channels)):
            self.sample = None
            raise ValueError('SOEM_JOINT_STATE_INVALID')
        try:
            mapped = self.mapping.decode(dict(zip(names, positions)))
        except ValueError as exc:
            self.sample = None
            raise ValueError('SOEM_JOINT_STATE_INVALID') from exc
        self.last_stamp = stamp
        self.sample = (stamp, mapped)

    def take(self, now):
        if not math.isfinite(now):
            raise ValueError('SOEM_FEEDBACK_TIME')
        if (not self.status_healthy or self.status_time is None or
                now - self.status_time > self.max_age or
                self.status_time - now > self.future_tolerance or
                self.sample is None):
            return None
        stamp, positions = self.sample
        if (stamp <= self.last_published_stamp or now - stamp > self.max_age or
                stamp - now > self.future_tolerance):
            return None
        self.last_published_stamp = stamp
        return stamp, positions
