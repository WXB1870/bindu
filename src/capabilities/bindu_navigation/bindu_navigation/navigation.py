"""Navigation configuration and per-goal input validation; no ROS dependencies."""
from dataclasses import dataclass
import json
import math
from pathlib import Path


def finite(values):
    return all(type(v) in (int, float) and math.isfinite(v) for v in values)


@dataclass(frozen=True)
class Site:
    name: str
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class NavigationConfig:
    map_id: str
    frame: str
    base_frame: str
    sites: dict
    timeout: float = 20.
    input_timeout: float = .25
    pose_timeout: float = .3
    position_tolerance: float = .08
    yaw_tolerance: float = .1
    stop_speed: float = .01
    stop_duration: float = .15
    stop_timeout: float = 2.

    @classmethod
    def load(cls, path):
        data = json.loads(Path(path).read_text())
        sites = {}
        for row in data.pop('sites'):
            if (set(row) != {'name', 'x', 'y', 'yaw'} or not isinstance(row['name'], str)
                    or not row['name'] or row['name'] in sites or not finite((row['x'], row['y'], row['yaw']))):
                raise ValueError('INVALID_OR_DUPLICATE_SITE')
            sites[row['name']] = Site(**row)
        cfg = cls(sites=sites, **data)
        if not sites or any(not isinstance(v, str) or not v for v in (cfg.map_id, cfg.frame, cfg.base_frame)):
            raise ValueError('INVALID_NAVIGATION_CONFIG')
        limits = (cfg.timeout, cfg.input_timeout, cfg.pose_timeout, cfg.position_tolerance,
                  cfg.yaw_tolerance, cfg.stop_speed, cfg.stop_duration, cfg.stop_timeout)
        if not finite(limits) or min(limits) <= 0 or not cfg.stop_duration < cfg.stop_timeout:
            raise ValueError('INVALID_NAVIGATION_LIMIT')
        if not cfg.input_timeout < 1. or not cfg.pose_timeout < cfg.timeout:
            raise ValueError('INVALID_NAVIGATION_TIMING')
        return cfg


def planar_pose(position, quaternion):
    if not finite((*position, *quaternion)) or len(position) != 3 or len(quaternion) != 4:
        raise ValueError('INVALID_NAVIGATION_POSE')
    x, y, z, w = quaternion
    if abs(sum(v*v for v in quaternion)-1.) > 1e-3:
        raise ValueError('INVALID_NAVIGATION_QUATERNION')
    return position[0], position[1], math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


class NavigationGate:
    """A velocity is usable only for its original goal and publishing process.

    The producer must attach the actual action UUID when computing a command;
    a generic Twist subscriber stamping the currently active UUID is unsafe.
    """
    def __init__(self, config, goal_id, start):
        self.config, self.goal_id, self.start = config, goal_id, start
        self.source_id = ''
        self.sequence = -1
        self.velocity = None
        self.pose = None
        self.fault = ''
        self.closed = False
        self.last_now = start
        self.stopped_since = None
        self.last_stop_stamp = None

    def close(self):
        self.closed = True
        self.velocity = None

    def accept_velocity(self, goal_id, source_id, sequence, stamp, frame, velocity, now):
        if self.closed or goal_id != self.goal_id:
            return False  # Late data from other goals must not disturb this goal.
        if self.source_id and source_id != self.source_id:
            self.fault = 'NAV_SOURCE_CHANGED'
        elif (not source_id or type(sequence) is not int or sequence < 0 or
              len(velocity) != 2 or not finite((*velocity, stamp, now)) or
              frame != self.config.base_frame):
            self.fault = 'NAV_INVALID_VELOCITY'
        elif sequence <= self.sequence:
            return False  # Duplicate/old sequence cannot refresh the watchdog.
        elif (stamp < self.start or stamp > now or now-stamp >= self.config.input_timeout
              or (self.velocity and stamp <= self.velocity[0])):
            self.fault = 'NAV_STALE_VELOCITY'
        if self.fault:
            self.close()
            return False
        self.source_id, self.sequence = source_id, sequence
        self.velocity = (stamp, tuple(velocity))
        return True

    def accept_pose(self, map_id, frame, stamp, pose, now):
        if self.closed:
            return
        if (map_id != self.config.map_id or frame != self.config.frame or
                len(pose) != 3 or not finite((*pose, stamp, now))):
            self.fault = 'NAV_INVALID_POSE'
        elif stamp > now or now-stamp >= self.config.pose_timeout:
            self.fault = 'NAV_POSE_STALE'
        elif not self.pose or stamp > self.pose[0]:
            self.pose = (stamp, tuple(pose))

    def check(self, now, require_velocity=True):
        if not math.isfinite(now) or now < self.last_now:
            self.fault = 'NAV_CLOCK_RESET'
        self.last_now = now
        if self.fault:
            raise RuntimeError(self.fault)
        if self.closed:
            raise RuntimeError('NAV_GATE_CLOSED')
        if not self.pose or not 0 <= now-self.pose[0] < self.config.pose_timeout:
            raise RuntimeError('NAV_POSE_STALE')
        if require_velocity and (not self.velocity or not 0 <= now-self.velocity[0] < self.config.input_timeout):
            raise RuntimeError('NAV_INPUT_TIMEOUT')

    def at_site(self, site):
        if not self.pose:
            return False
        x, y, yaw = self.pose[1]
        dyaw = math.atan2(math.sin(yaw-site.yaw), math.cos(yaw-site.yaw))
        return math.hypot(x-site.x, y-site.y) <= self.config.position_tolerance and abs(dyaw) <= self.config.yaw_tolerance

    def stopped(self, stamp, velocity, now, requested_at):
        valid = (finite((stamp, now, *velocity)) and stamp >= requested_at and
                 0 <= now-stamp < self.config.pose_timeout and len(velocity) == 2 and
                 max(abs(v) for v in velocity) <= self.config.stop_speed)
        if not valid:
            self.stopped_since = self.last_stop_stamp = None
            return False
        if self.last_stop_stamp is not None and (stamp < self.last_stop_stamp or
                                                stamp-self.last_stop_stamp >= self.config.pose_timeout):
            self.stopped_since = None
        self.last_stop_stamp = stamp
        if self.stopped_since is None:
            self.stopped_since = stamp
        # Re-reading a frozen message never advances the evidence interval.
        return stamp-self.stopped_since >= self.config.stop_duration


class SessionVelocityFence:
    """Immutable goal/process binding used by isolated native Nav2 sessions.

    A new process publisher on the same topic fails closed. Neither timestamps
    nor the goal ID are replaced at reception. This is isolation, not DDS auth.
    """
    def __init__(self, goal_id, source_id, started):
        self.goal_id, self.source_id, self.started = goal_id, source_id, started
        self.publishers = frozenset()
        self.closed = False
        self.sequence = 0
        self.last_stamp = None

    def bind_publishers(self, publishers):
        if self.publishers or not publishers or self.closed:
            raise ValueError('NAV2_PUBLISHER_BINDING_INVALID')
        self.publishers = frozenset(publishers)

    def accept(self, publisher, stamp, now, timeout):
        if self.closed:
            return 'NAV2_SESSION_CLOSED'
        if publisher not in self.publishers:
            return 'NAV2_PUBLISHER_CHANGED'
        if (not finite((stamp, now)) or stamp < self.started or stamp > now or
                now-stamp >= timeout or (self.last_stamp is not None and stamp <= self.last_stamp)):
            return 'NAV2_COMMAND_STALE'
        self.last_stamp = stamp
        return ''

    def next_sequence(self):
        self.sequence += 1
        return self.sequence

    def close(self):
        self.closed = True
