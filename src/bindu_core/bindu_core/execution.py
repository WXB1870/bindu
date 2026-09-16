"""Single-owner reference executor. Call under one serialized callback group.

This Python implementation validates contracts, not hard real-time performance.
Joint targets use the simulated backend's bounded tracking; finite trajectories
retain their own timeline. Interpolation belongs to execution; device adapters only accept references.
"""
from collections import OrderedDict, deque
import math
import uuid
from .contracts import Event


class Rejected(ValueError):
    pass


class Executor:
    def __init__(self, profile, robot_io):
        self.profile, self.io = profile, robot_io
        self.lease_id = ''
        self.epoch = 0
        self.owner = ''
        self.groups = ()
        self.expires = 0.
        self.active = None
        self.started = 0.
        self.initial = {}
        self.last_stamp = 0.
        self.feedback = robot_io.last_feedback
        self.events = deque(maxlen=2048)
        self.results = OrderedDict()
        self.last_tick = None

    def emit(self, now, motion, state, code):
        event = Event(now, motion.task_id if motion else '', motion.command_id if motion else '',
                      state, code, motion.observation_id if motion else '')
        self.events.append(event)
        if motion:
            self.results[motion.command_id] = event
            self.results.move_to_end(motion.command_id)
            while len(self.results) > 256:
                self.results.popitem(last=False)

    def acquire(self, owner, groups, ttl, now):
        self.tick(now)
        if self.lease_id:
            raise Rejected('BUSY')
        if not owner or not 0 < ttl <= 5 or not groups:
            raise Rejected('INVALID_LEASE')
        if any(g not in set(self.profile.groups) | {'base'} for g in groups):
            raise Rejected('UNSUPPORTED_RESOURCE')
        if 'base' in groups and 'base_velocity' not in self.profile.capabilities:
            raise Rejected('UNSUPPORTED_CAPABILITY')
        self.epoch += 1
        self.lease_id = uuid.uuid4().hex
        self.owner, self.groups, self.expires = owner, tuple(groups), now + ttl
        self.last_stamp = 0.
        return self.lease_id, self.epoch

    def check_lease(self, lease, epoch, now):
        if not lease or lease != self.lease_id or epoch != self.epoch or now >= self.expires:
            raise Rejected('INVALID_LEASE')

    def renew(self, lease, epoch, now):
        self.check_lease(lease, epoch, now)
        self.expires = now + 2.

    def halt(self, now, code='CANCELED', revoke=False):
        self.io.stop()
        if self.io.stop_failures:
            code = 'STOP_REQUEST_FAILED:' + ','.join(self.io.stop_failures)
            revoke = True
        if self.active:
            self.emit(now, self.active, 'CANCELED' if code == 'CANCELED' else 'FAILED', code)
        self.active = None
        if revoke:
            self.lease_id = ''
            self.owner = ''
            self.epoch += 1

    def submit(self, m, now):
        self.tick(now)
        self.check_lease(m.lease_id, m.epoch, now)
        if m.schema_version != 1 or m.profile_hash != self.profile.digest:
            raise Rejected('SCHEMA_OR_PROFILE_MISMATCH')
        if not m.command_id or not m.task_id or m.command_id in self.results:
            raise Rejected('DUPLICATE_OR_EMPTY_ID')
        if m.group not in self.groups:
            raise Rejected('RESOURCE_NOT_OWNED')
        if not all(math.isfinite(x) for x in (m.stamp, m.valid_for, m.duration)):
            raise Rejected('INVALID_TIME')
        if not (0 < m.valid_for <= 10) or m.stamp > now + .05 or m.stamp < self.last_stamp or now >= m.stamp + m.valid_for:
            raise Rejected('STALE_OR_UNORDERED')
        if now - self.feedback.stamp > .2:
            raise Rejected('FEEDBACK_STALE')
        if m.mode == 'base_velocity':
            if m.group != 'base' or 'base_velocity' not in self.profile.capabilities:
                raise Rejected('UNSUPPORTED_CAPABILITY')
            if m.names or m.positions or m.points or not 0 < m.duration < m.valid_for:
                raise Rejected('INVALID_BASE_COMMAND')
            if len(m.velocity) != 2 or not all(math.isfinite(x) and abs(x) <= .4 for x in m.velocity):
                raise Rejected('BASE_LIMIT')
        elif m.mode in ('joint_target', 'finite_trajectory'):
            if m.group not in self.profile.groups or 'joint_position' not in self.profile.capabilities:
                raise Rejected('UNSUPPORTED_CAPABILITY')
            if (m.velocity != (0., 0.) or m.duration or
                    (m.mode == 'joint_target' and (m.points or m.offsets)) or
                    (m.mode == 'finite_trajectory' and m.positions)):
                raise Rejected('MIXED_COMMAND_PAYLOAD')
            if tuple(self.profile.groups.get(m.group, ())) != m.names:
                raise Rejected('JOINT_LAYOUT_MISMATCH')
            values = (m.positions,) if m.mode == 'joint_target' else m.points
            if not values:
                raise Rejected('EMPTY_TRAJECTORY')
            for point in values:
                if len(point) != len(m.names):
                    raise Rejected('DIMENSION_MISMATCH')
                for joint, value in zip(m.names, point):
                    lo, hi = self.profile.limits[joint]
                    if not math.isfinite(value) or not lo <= value <= hi:
                        raise Rejected('JOINT_LIMIT')
            if m.mode == 'finite_trajectory':
                if (len(m.offsets) != len(m.points) or not all(math.isfinite(t) for t in m.offsets) or
                        not 0 < m.offsets[0] or not all(a < b for a, b in zip(m.offsets, m.offsets[1:])) or
                        m.stamp + m.offsets[-1] >= m.stamp + m.valid_for or m.stamp < now - .1):
                    raise Rejected('INVALID_TRAJECTORY_TIME')
                previous = tuple(self.feedback.positions[j] for j in m.names)
                previous_t = 0.
                for t, point in zip(m.offsets, m.points):
                    if any(abs(q - p) / (t - previous_t) > self.profile.max_speed for p, q in zip(previous, point)):
                        raise Rejected('TRAJECTORY_SPEED_LIMIT')
                    previous, previous_t = point, t
        else:
            raise Rejected('UNSUPPORTED_MODE')
        if self.active:
            self.io.stop()
            if self.io.stop_failures:
                self.halt(now, 'STOP_REQUEST_FAILED', revoke=True)
                raise Rejected('STOP_REQUEST_FAILED')
            self.emit(now, self.active, 'SUPERSEDED', 'REPLACED')
        self.active, self.started = m, m.stamp
        self.last_stamp = m.stamp
        self.initial = dict(self.feedback.positions)
        self.emit(now, m, 'ACCEPTED', 'OK')

    def tick(self, now):
        if self.last_tick is not None and now < self.last_tick:
            self.halt(now, 'CLOCK_RESET', revoke=True)
        dt = 0. if self.last_tick is None else max(0., now - self.last_tick)
        self.last_tick = now
        try:
            self.feedback = self.io.read_feedback(now, dt)
        except RuntimeError:
            self.halt(now, 'DEVICE_FEEDBACK_ERROR', revoke=True)
            return
        if self.lease_id and now >= self.expires:
            self.halt(now, 'LEASE_EXPIRED', revoke=True)
        m = self.active
        if not m:
            return
        if now - self.feedback.stamp > .2:
            self.halt(now, 'FEEDBACK_STALE', revoke=True)
            return
        if now >= m.stamp + m.valid_for:
            self.halt(now, 'COMMAND_TIMEOUT')
            return
        if now < m.stamp:
            return
        elapsed = max(0., now - self.started)
        try:
            if m.mode == 'base_velocity':
                self.io.write_base(m.velocity if elapsed < m.duration else (0., 0.))
                done = elapsed >= m.duration and self.feedback.base_velocity == (0., 0.)
            else:
                goal = m.positions
                if m.mode == 'finite_trajectory':
                    times = (0.,) + m.offsets
                    points = (tuple(self.initial[j] for j in m.names),) + m.points
                    goal = points[-1]
                    for i in range(1, len(times)):
                        if elapsed < times[i]:
                            alpha = (elapsed - times[i-1]) / (times[i] - times[i-1])
                            goal = tuple(a + alpha * (b - a) for a, b in zip(points[i-1], points[i]))
                            break
                self.io.write_joints(dict(zip(m.names, goal)))
                target = m.points[-1] if m.mode == 'finite_trajectory' else m.positions
                done = all(abs(self.feedback.positions[j] - q) < .015 for j, q in zip(m.names, target))
                done = done and (m.mode != 'finite_trajectory' or elapsed >= m.offsets[-1])
                # Online targets keep their session alive until replaced or explicitly halted.
                if m.mode == 'joint_target':
                    done = False
            if done:
                self.emit(now, m, 'SUCCEEDED', 'FEEDBACK_CONFIRMED')
                self.active = None
        except RuntimeError:
            self.halt(now, 'DEVICE_COMMAND_REJECTED', revoke=True)
