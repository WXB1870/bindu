"""Two-axis, source-fenced VR base control; no ROS or device side effects."""
import math


ZERO = (0., 0.)


def validate_base_config(cfg):
    if not isinstance(cfg, dict) or type(cfg.get('schema_version')) is not int or cfg['schema_version'] != 1:
        raise ValueError('BASE_CONFIG_SCHEMA')
    for key in ('cmd_vel_topic', 'odom_topic', 'feedback_frame'):
        value = cfg.get(key)
        if not isinstance(value, str) or not value or (key.endswith('_topic') and
                (not value.startswith('/') or '//' in value or value.endswith('/'))):
            raise ValueError('BASE_CONFIG:'+key)
    bounds = dict(max_linear_m_s=1., max_angular_rad_s=2., linear_acceleration_m_s2=2.,
        angular_acceleration_rad_s2=4., deadzone=.5, input_max_age_s=.5,
        feedback_max_age_s=.5, max_tick_gap_s=.2, stop_linear_m_s=.05,
        stop_angular_rad_s=.1, stop_window_s=1.)
    for key, maximum in bounds.items():
        value = cfg.get(key)
        if type(value) not in (int, float) or not 0 < value <= maximum:
            raise ValueError('BASE_CONFIG:'+key)
    watchdog = cfg.get('driver_watchdog_s')
    if watchdog is not None and (type(watchdog) not in (int, float) or
            not 0 < watchdog <= .5):
        raise ValueError('BASE_CONFIG:driver_watchdog_s')
    return dict(cfg, driver_watchdog_s=watchdog)


class BaseTeleop:
    """PICO owns this base output; a lost permit always requires neutral rearm.

    Input/feedback stamps remain in the node clock; receipt ages and rate
    limiting use an independent monotonic clock. Published zero is not a stop
    acknowledgement: require subsequent, sustained measured zero velocity.
    """
    def __init__(self, config):
        self.cfg = validate_base_config(config)
        self.source = None
        self.retired = set()
        self.seq = -1
        self.input_stamp = self.received = float('-inf')
        self.feedback_stamp = self.feedback_received = float('-inf')
        self.feedback_velocity = None
        self.target = self.command = ZERO
        self.neutral = self.armed = False
        self.last_tick = None
        self.zero_requested = None
        self.stationary_since = None
        self.reason = 'BASE_WAITING_FOR_INPUT'

    def inhibit(self, reason, now):
        if self.zero_requested is None or self.command != ZERO:
            self.zero_requested = now
            self.stationary_since = None
        self.command = ZERO
        self.armed = False
        self.neutral = False
        self.reason = reason
        return ZERO

    def ingest(self, left, right, now, clock):
        pair = (left, right)
        if (left.side != 'left' or right.side != 'right' or not left.source_id or
                left.source_id != right.source_id or left.seq != right.seq or
                left.stamp != right.stamp or any(not f.valid or f.mapping != 'pico_snowbot_v1' or
                f.stop or len(f.stick) != 2 or any(type(x) not in (int, float) or
                not -1 <= x <= 1 for x in f.stick) for f in pair)):
            self.target = ZERO
            self.neutral = False
            self.inhibit('BASE_INPUT_INVALID', now)
            return
        if left.source_id != self.source:
            if left.source_id in self.retired:
                self.inhibit('BASE_RETIRED_SOURCE', now)
                return
            if self.source is not None:
                self.retired.add(self.source)
            self.source, self.seq, self.input_stamp = left.source_id, -1, float('-inf')
            self.inhibit('BASE_SOURCE_CHANGED', now)
        if (left.seq <= self.seq or left.stamp <= self.input_stamp or
                not 0 <= now-left.stamp <= self.cfg['input_max_age_s']):
            self.inhibit('BASE_INPUT_STALE', now)
            return
        self.seq, self.input_stamp, self.received = left.seq, left.stamp, clock
        # Preserve the reference script's WebXR signs and deadzone, but only
        # use left Y for forward/reverse and right X for yaw. No button remap.
        def axis(value):
            return 0. if abs(value) < self.cfg['deadzone'] else -value
        self.target = (axis(left.stick[1])*self.cfg['max_linear_m_s'],
                       axis(right.stick[0])*self.cfg['max_angular_rad_s'])
        self.neutral = (abs(left.stick[1]) < self.cfg['deadzone'] and
                        abs(right.stick[0]) < self.cfg['deadzone'])

    def observe(self, stamp, velocity, now, clock):
        if (len(velocity) != 2 or not math.isfinite(stamp) or
                any(not math.isfinite(x) for x in velocity) or
                not 0 <= now-stamp <= self.cfg['feedback_max_age_s']):
            self.feedback_velocity = None
            self.stationary_since = None
            return
        if stamp <= self.feedback_stamp:
            return  # Repeated/late samples never refresh the feedback watchdog.
        if not self.healthy(now, clock):
            self.stationary_since = None
        self.feedback_stamp, self.feedback_received = stamp, clock
        self.feedback_velocity = tuple(velocity)
        if (self.command == ZERO and self.zero_requested is not None and stamp > self.zero_requested and
                abs(velocity[0]) <= self.cfg['stop_linear_m_s'] and
                abs(velocity[1]) <= self.cfg['stop_angular_rad_s']):
            if self.stationary_since is None:
                self.stationary_since = stamp
        else:
            self.stationary_since = None

    def healthy(self, now, clock):
        return (self.feedback_velocity is not None and
                0 <= now-self.feedback_stamp <= self.cfg['feedback_max_age_s'] and
                0 <= clock-self.feedback_received <= self.cfg['feedback_max_age_s'])

    def stopped(self, now, clock):
        return bool(self.healthy(now, clock) and self.command == ZERO and
                    self.stationary_since is not None and
                    self.feedback_stamp-self.stationary_since >= self.cfg['stop_window_s'])

    def tick(self, now, clock, *, allowed, reason='BASE_NOT_PERMITTED'):
        dt = 0. if self.last_tick is None else clock-self.last_tick
        self.last_tick = clock
        if not allowed:
            return self.inhibit(reason, now)
        if not 0 < dt <= self.cfg['max_tick_gap_s']:
            return self.inhibit('BASE_TICK_GAP', now)
        if not self.healthy(now, clock):
            return self.inhibit('BASE_FEEDBACK_UNAVAILABLE', now)
        if not (0 <= now-self.input_stamp <= self.cfg['input_max_age_s'] and
                0 <= clock-self.received <= self.cfg['input_max_age_s']):
            return self.inhibit('BASE_INPUT_TIMEOUT', now)
        if not self.armed:
            if not self.neutral or not self.stopped(now, clock):
                return self.inhibit('BASE_CENTRE_STICKS_AND_WAIT_FOR_STOP', now)
            self.armed = True
        previous = self.command
        self.command = tuple(v+max(-a*dt, min(a*dt, target-v)) for v, target, a in zip(
            self.command, self.target, (self.cfg['linear_acceleration_m_s2'],
                                      self.cfg['angular_acceleration_rad_s2'])))
        if self.command != ZERO:
            self.zero_requested = self.stationary_since = None
        elif previous != ZERO or self.zero_requested is None:
            self.zero_requested, self.stationary_since = now, None
        self.reason = 'BASE_ACTIVE' if self.command != ZERO else 'BASE_READY'
        return self.command
