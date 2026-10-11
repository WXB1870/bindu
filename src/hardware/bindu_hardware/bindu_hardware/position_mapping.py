"""Validated model-radian <-> vendor-position mapping; no transport or control.

One frame is assembled atomically, including explicitly held extra channels.
Calibration is applied only here, never again in the motion solver.
"""
import math


class PositionMapping:
    def __init__(self, config, joint_names):
        if (set(config) != {'schema_version', 'channels', 'joints', 'hold_channels'} or
                type(config['schema_version']) is not int or config['schema_version'] != 1):
            raise ValueError('POSITION_MAPPING_SCHEMA')
        self.channels = tuple(config['channels'])
        self.joints = dict(config['joints'])
        self.hold_channels = tuple(config['hold_channels'])
        if (not self.channels or any(not isinstance(n, str) or not n for n in self.channels) or
                len(set(self.channels)) != len(self.channels) or set(self.joints) != set(joint_names)):
            raise ValueError('POSITION_MAPPING_LAYOUT')
        destinations, self.transforms = [], {}
        for name, item in self.joints.items():
            if set(item) != {'channel', 'unit', 'direction', 'offset'}:
                raise ValueError('POSITION_MAPPING_JOINT:' + name)
            if (item['unit'] not in ('rad', 'deg') or type(item['direction']) is not int or
                    item['direction'] not in (-1, 1) or type(item['offset']) not in (float, int) or
                    not math.isfinite(item['offset']) or item['channel'] not in self.channels):
                raise ValueError('POSITION_MAPPING_JOINT:' + name)
            scale = item['direction'] * (180/math.pi if item['unit'] == 'deg' else 1.)
            self.transforms[name] = (item['channel'], scale, float(item['offset']))
            destinations.append(item['channel'])
        combined = destinations + list(self.hold_channels)
        if len(combined) != len(set(combined)) or set(combined) != set(self.channels):
            raise ValueError('POSITION_MAPPING_COVERAGE')
        self._held = None

    def _feedback(self, feedback):
        if (set(feedback) != set(self.channels) or
                any(type(v) not in (float, int) or not math.isfinite(v) for v in feedback.values())):
            raise ValueError('POSITION_MAPPING_FEEDBACK')

    def decode(self, feedback):
        try:
            self._feedback(feedback)
        except ValueError:
            self.invalidate()
            raise
        return {n: (feedback[channel]-offset)/scale
                for n, (channel, scale, offset) in self.transforms.items()}

    def capture(self, feedback):
        # A failed recapture must invalidate the previous session's hold values.
        self.invalidate()
        decoded = self.decode(feedback)
        self._held = {n: feedback[n] for n in self.hold_channels}
        return decoded

    def invalidate(self):
        self._held = None

    def encode(self, positions):
        if self._held is None:
            raise ValueError('POSITION_MAPPING_NOT_CAPTURED')
        if (set(positions) != set(self.joints) or
                any(type(v) not in (float, int) or not math.isfinite(v) for v in positions.values())):
            raise ValueError('POSITION_MAPPING_TARGET')
        values = dict(self._held)
        for name, (channel, scale, offset) in self.transforms.items():
            values[channel] = positions[name]*scale+offset
        if not all(math.isfinite(v) for v in values.values()):
            raise ValueError('POSITION_MAPPING_TARGET')
        return tuple(values[n] for n in self.channels)
