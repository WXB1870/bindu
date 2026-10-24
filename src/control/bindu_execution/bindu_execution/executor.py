"""Single-owner reference executor. Call under one serialized callback group.

This Python implementation validates contracts, not hard real-time performance.
Joint references are generated and bounded here; timed trajectories retain their
own timeline. Device tracking limits are independent of reference limits.
Device adapters only accept references.
"""
from collections import OrderedDict, deque
import math
import time
import uuid
from bindu_contracts.contracts import Event
from bindu_contracts.ports import ExecutionIO
from bindu_contracts.runtime_mode import resolve_mode, command_mode, SIMULATION
from .interpolation import State, Timeline, fit_online, fit_target, fit_timed, fit_stop, join, trim_before


class Rejected(ValueError):
    pass


class Executor:
    def __init__(self, profile, robot_io: ExecutionIO, *, runtime_mode=None, steady=time.monotonic):
        self.profile, self.io = profile, robot_io
        self.runtime_mode = resolve_mode('' if runtime_mode is None else runtime_mode, default=(
            SIMULATION if robot_io.last_feedback.simulated else 'real'))
        if type(robot_io.last_feedback.simulated) is not bool or robot_io.last_feedback.simulated != (self.runtime_mode == SIMULATION):
            raise ValueError('FEEDBACK_RUNTIME_MODE_MISMATCH')
        self.lease_id = ''
        self.epoch = 0
        self.owner = ''
        self.groups = ()
        self.expires = 0.
        self.active = None
        self.started = 0.
        self.last_stamp = 0.
        self.feedback = robot_io.last_feedback
        self.events = deque(maxlen=2048)
        self.results = OrderedDict()
        self.last_tick = None
        self.path = None
        self.reference = None
        self.reference_names = ()
        self.stopping = None
        self.stop_deadline = 0.
        self.steady = steady
        self.stop_steady_deadline = 0.
        self.device_stop_pending = False
        self.stop_after_stamp = 0.
        self.stop_fault = ''
        self.revision = 1
        self.held_references = {}
        self.base_reference = (0., 0.)
        self.base_zero_requested_at = None

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
        if self.stop_fault or self.io.stop_failures:
            raise Rejected('STOP_UNCONFIRMED')
        if self.lease_id or self.stopping:
            raise Rejected('BUSY')
        if not 0 <= now-self.feedback.stamp <= .2:
            raise Rejected('FEEDBACK_STALE')
        measured_names = (tuple(n for g in groups for n in self.profile.groups.get(g, ()))
                          if getattr(self.io, 'group_feedback', False) else tuple(self.profile.limits))
        if any(j not in self.feedback.joint_velocities for j in measured_names):
            raise Rejected('FEEDBACK_VELOCITY_UNAVAILABLE')
        if (any(not self.joint_at_rest(j) for j in measured_names) or
                any(abs(v) > self.profile.feedback_tolerances.get('base_'+axis+'_velocity', 0.)
                    for axis,v in zip(('linear','angular'),self.feedback.base_velocity))):
            raise Rejected('FEEDBACK_NOT_STATIONARY')
        if not owner or not 0 < ttl <= 5 or not groups or len(groups) != len(set(groups)):
            raise Rejected('INVALID_LEASE')
        if any(g not in set(self.profile.groups) | {'base'} for g in groups):
            raise Rejected('UNSUPPORTED_RESOURCE')
        if 'base' in groups and 'base_velocity' not in self.profile.capabilities:
            raise Rejected('UNSUPPORTED_CAPABILITY')
        self.epoch += 1
        self.lease_id = uuid.uuid4().hex
        self.owner, self.groups, self.expires = owner, tuple(groups), now + ttl
        self.last_stamp = 0.
        self.held_references.clear()
        self.revision += 1
        return self.lease_id, self.epoch

    def check_lease(self, lease, epoch, now):
        if not lease or lease != self.lease_id or epoch != self.epoch or now >= self.expires:
            raise Rejected('INVALID_LEASE')

    def renew(self, lease, epoch, now):
        self.check_lease(lease, epoch, now)
        self.expires = now + 2.

    def halt(self, now, code='CANCELED', revoke=False, emergency=False, effective_at=None):
        if (revoke or emergency) and self.lease_id:
            self.lease_id = ''
            self.owner = ''
            self.epoch += 1
        if self.stopping and not emergency:
            return
        if self.stop_fault:
            return
        if self.device_stop_pending:
            if self.stop_expired(now):
                self.fail_stop(now, 'STOP_FEEDBACK_TIMEOUT')
            return
        motion = self.active or (self.stopping[0] if self.stopping else None)
        if not self.stopping:
            self.stop_deadline = now+self.profile.stop_timeout
            self.stop_steady_deadline = self.steady()+self.profile.stop_timeout
        self.last_stamp = max(self.last_stamp, now)
        self.revision += 1
        stop_start = now if effective_at is None else effective_at
        if not emergency and self.path:
            try:
                self.reference = self.path.sample(stop_start)
            except ValueError:
                code += ':REFERENCE_GENERATION_FAILED'
                emergency = True
                if self.lease_id:
                    self.lease_id = ''
                    self.owner = ''
                    self.epoch += 1
        if not emergency and self.path and self.reference and (any(
                abs(x) > 1e-8 for x in self.reference.v+self.reference.a) or any(
                abs(self.feedback.positions[j]-q) > 1e-5 for j, q in zip(self.reference_names, self.reference.q))):
            try:
                self.path = fit_stop(self.profile, self.reference_names, self.reference, stop_start)
                self.stopping = (motion, code)
                self.active = None
                self.emit(now, motion, 'STOPPING', code)
                return
            except ValueError:
                # Feedback/driver faults and an uncertifiable braking envelope
                # use the device stop. Do not report this as a smooth stop.
                code += ':EMERGENCY_STOP_NO_FEASIBLE_BRAKE'
                self.lease_id = ''
                self.owner = ''
                self.epoch += 1
        elif not emergency and self.held_references and not self.references_arrived():
            # A finite goal may satisfy its arrival tolerance while its device
            # is still converging. Keep that resting reference until feedback
            # confirms it instead of truncating it at the measured position.
            try:
                self.io.write_base((0., 0.))
                self.stopping = (motion, code)
                self.active = None
                self.emit(now, motion, 'STOPPING', code)
                return
            except RuntimeError:
                code = 'DEVICE_COMMAND_REJECTED'
        self.finish_stop(now, motion, code)

    def finish_stop(self, now, motion, code):
        """Request a device stop once, then await independent measured rest."""
        try:
            self.io.stop()
        except RuntimeError:
            self.io.stop_failures = tuple(sorted(set(self.io.stop_failures) | {'device_stop'}))
        self.device_stop_pending = True
        self.stop_after_stamp = max(now, self.feedback.stamp)
        self.stopping = (motion, code)
        self.base_reference = (0., 0.)
        self.active = self.path = self.reference = None
        self.reference_names = ()
        self.held_references.clear()
        if self.io.stop_failures:
            self.fail_stop(now, 'STOP_REQUEST_FAILED:'+','.join(self.io.stop_failures))
        elif self.stop_expired(now):
            self.fail_stop(now, 'STOP_FEEDBACK_TIMEOUT')
        else:
            self.emit(now, motion, 'STOPPING', code)

    def stop_expired(self, now):
        # Preserve both budgets across repeated faults; ROS time can stall/reset.
        return now >= self.stop_deadline or self.steady() >= self.stop_steady_deadline

    def fail_stop(self, now, code):
        motion = self.active or (self.stopping[0] if self.stopping else None)
        self.stop_fault = code
        if not self.io.stop_failures:
            self.io.stop_failures = ('stop_feedback',)
        if self.lease_id:
            self.epoch += 1
        self.lease_id = self.owner = ''
        self.emit(now, motion, 'FAILED', code)
        self.active = self.stopping = self.path = self.reference = None
        self.device_stop_pending = False
        self.reference_names = ()
        self.held_references.clear()

    def prepare_path(self, m, now):
        switch = max(now, m.stamp)
        initial = (self.path.sample(switch) if self.path and self.reference_names == m.names
                   else State.rest(self.held_references.get(j, self.feedback.positions[j]) for j in m.names))
        if m.mode == 'joint_target':
            if (self.active and self.active.mode == m.mode and self.active.positions == m.positions
                    and self.reference_names == m.names):
                return self.path
            # Coupled tools can select the existing synchronized group curve;
            # ordinary position streams retain the adaptive scalar executor.
            fit = fit_target if self.profile.joint_target_interpolation == 'synchronized' else fit_online
            suffix = fit(self.profile, m.names, initial, m.positions, switch)
        else:
            # A positional waypoint means rest there unless explicit v/a are
            # supplied. Never silently discard a planner's derivatives.
            zero = (0.,)*len(m.names)
            velocities = m.velocities or (zero,)*len(m.points)
            accelerations = m.accelerations or (zero,)*len(m.points)
            if len(velocities) != len(m.points) or len(accelerations) != len(m.points):
                raise Rejected('DERIVATIVE_LAYOUT_MISMATCH')
            for rows in (velocities, accelerations):
                if any(len(row) != len(m.names) or not all(math.isfinite(x) for x in row) for row in rows):
                    raise Rejected('DERIVATIVE_LAYOUT_MISMATCH')
            states = tuple(State(q, v, a) for q, v, a in zip(m.points, velocities, accelerations))
            suffix = fit_timed(self.profile, m.names, initial, states,
                tuple(m.stamp+offset for offset in m.offsets), switch,
                chunk=m.mode == 'joint_reference_segment')
        return join(self.path, suffix, switch) if self.path and switch > now else suffix

    def submit(self, m, now):
        try:
            command_mode(m.runtime_mode, self.runtime_mode)
        except ValueError as exc:
            raise Rejected(str(exc)) from exc
        self.tick(now)
        self.check_lease(m.lease_id, m.epoch, now)
        if self.stopping:
            raise Rejected('STOPPING')
        if (m.expected_revision and m.expected_revision != self.revision) or (
                m.mode == 'joint_reference_segment' and not m.expected_revision):
            raise Rejected('STALE_REVISION')
        if m.schema_version not in (1, 2) or m.profile_hash != self.profile.digest:
            raise Rejected('SCHEMA_OR_PROFILE_MISMATCH')
        if (type(m.rigid_reference) is not bool or (m.rigid_reference and
                (m.schema_version != 2 or m.mode != 'joint_target' or
                 len(m.resource_groups) < 2))):
            raise Rejected('RIGID_REFERENCE_SCOPE')
        if m.rigid_reference and not getattr(self.io, 'supports_rigid_reference', False):
            raise Rejected('RIGID_REFERENCE_UNSUPPORTED')
        if not m.command_id or not m.task_id or m.command_id in self.results:
            raise Rejected('DUPLICATE_OR_EMPTY_ID')
        if m.schema_version == 1:
            if m.resource_groups or m.group not in self.groups:
                raise Rejected('RESOURCE_NOT_OWNED')
            command_groups = (m.group,)
        else:
            command_groups = m.resource_groups
            if (m.mode == 'base_velocity' or m.group or len(command_groups) < 2 or
                    len(set(command_groups)) != len(command_groups) or
                    not set(command_groups).issubset(self.groups)):
                raise Rejected('RESOURCE_NOT_OWNED')
        if not all(math.isfinite(x) for x in (m.stamp, m.valid_for, m.duration)):
            raise Rejected('INVALID_TIME')
        if not (0 < m.valid_for <= 10) or m.stamp > now + .05 or m.stamp < self.last_stamp or now >= m.stamp + m.valid_for:
            raise Rejected('STALE_OR_UNORDERED')
        if now - self.feedback.stamp > .2:
            raise Rejected('FEEDBACK_STALE')
        if m.mode == 'base_velocity':
            if command_groups != ('base',) or 'base_velocity' not in self.profile.capabilities:
                raise Rejected('UNSUPPORTED_CAPABILITY')
            if m.names or m.positions or m.points or m.offsets or m.velocities or m.accelerations or not 0 < m.duration < m.valid_for:
                raise Rejected('INVALID_BASE_COMMAND')
            if len(m.velocity) != 2 or not all(math.isfinite(x) and abs(x) <= self.profile.base_speed(axis)
                    for axis, x in zip(("linear", "angular"), m.velocity)):
                raise Rejected('BASE_LIMIT')
        elif m.mode in ('joint_target', 'finite_trajectory', 'joint_reference_segment'):
            if (any(g not in self.profile.groups for g in command_groups) or
                    'joint_position' not in self.profile.capabilities):
                raise Rejected('UNSUPPORTED_CAPABILITY')
            if (m.velocity != (0., 0.) or m.duration or
                    (m.mode == 'joint_target' and (m.points or m.offsets or m.velocities or m.accelerations)) or
                    (m.mode != 'joint_target' and m.positions)):
                raise Rejected('MIXED_COMMAND_PAYLOAD')
            if tuple(j for g in command_groups for j in self.profile.groups[g]) != m.names:
                raise Rejected('JOINT_LAYOUT_MISMATCH')
            values = (m.positions,) if m.mode == 'joint_target' else m.points
            if not values:
                raise Rejected('EMPTY_TRAJECTORY')
            if len(values) > 1024:
                raise Rejected('TRAJECTORY_TOO_LARGE')
            for point in values:
                if len(point) != len(m.names):
                    raise Rejected('DIMENSION_MISMATCH')
                for joint, value in zip(m.names, point):
                    lo, hi = self.profile.limits[joint]
                    if not math.isfinite(value) or not lo <= value <= hi:
                        raise Rejected('JOINT_LIMIT')
            if m.mode != 'joint_target':
                if (len(m.offsets) != len(m.points) or not all(math.isfinite(t) for t in m.offsets) or
                        not 0 <= m.offsets[0] or not all(a < b for a, b in zip(m.offsets, m.offsets[1:])) or
                        m.offsets[-1] >= m.valid_for or (m.mode == 'finite_trajectory' and m.stamp < now - .1)):
                    raise Rejected('INVALID_TRAJECTORY_TIME')
        else:
            raise Rejected('UNSUPPORTED_MODE')
        if self.path and m.names != self.reference_names:
            raise Rejected('TRANSITION_REQUIRES_STOP')
        if self.active and (self.active.mode == 'base_velocity') != (m.mode == 'base_velocity'):
            raise Rejected('TRANSITION_REQUIRES_STOP')
        try:
            candidate = None if m.mode == 'base_velocity' else self.prepare_path(m, now)
        except ValueError as exc:
            raise Rejected(str(exc)) from exc
        # All checks and preparation precede this serialized commit. Rejected
        # replacements leave the existing timeline and revision intact.
        if self.active:
            self.emit(now, self.active, 'SUPERSEDED', 'REPLACED')
        self.active, self.started = m, m.stamp
        self.base_zero_requested_at = None
        self.last_stamp = m.stamp
        self.path = candidate
        self.reference_names = m.names
        for name in m.names:
            self.held_references.pop(name, None)
        if candidate:
            self.reference = candidate.sample(now)
        self.revision += 1
        self.emit(now, m, 'ACCEPTED', 'OK')

    def tick(self, now):
        if not math.isfinite(now) or (self.last_tick is not None and now < self.last_tick):
            self.halt(now if math.isfinite(now) else (self.last_tick or 0.), 'CLOCK_RESET', revoke=True, emergency=True)
            self.last_tick = now if math.isfinite(now) else None
            return
        dt = 0. if self.last_tick is None else max(0., now-self.last_tick)
        self.last_tick = now
        try:
            feedback = self.io.read_feedback(now, dt)
            if type(feedback.simulated) is not bool or feedback.simulated != (self.runtime_mode == SIMULATION):
                self.halt(now, 'FEEDBACK_RUNTIME_MODE_MISMATCH', revoke=True, emergency=True)
                return
            self.feedback = feedback
        except (RuntimeError, ValueError) as exc:
            self.emit(now, self.active, 'DEVICE_FEEDBACK_DETAIL', str(exc))
            self.halt(now, 'DEVICE_FEEDBACK_ERROR', revoke=True, emergency=True)
            return
        if (self.active or self.stopping) and now-self.feedback.stamp > .2:
            self.halt(now, 'FEEDBACK_STALE', revoke=True, emergency=True)
            return
        measured_names = self.reference_names if getattr(self.io, 'group_feedback', False) else self.profile.limits
        if self.active and any(j not in self.feedback.joint_velocities for j in measured_names):
            self.halt(now, 'FEEDBACK_VELOCITY_UNAVAILABLE', revoke=True, emergency=True)
            return
        if (self.active or self.stopping) and not self.device_stop_pending and dt > self.profile.max_tick_gap+1e-9:
            self.halt(now, 'EXECUTION_OVERRUN', revoke=True, emergency=True)
            return
        if self.lease_id and now >= self.expires:
            deadline = min(self.expires, self.active.stamp+self.active.valid_for) if self.active else self.expires
            self.halt(now, 'LEASE_EXPIRED', revoke=True, effective_at=deadline)
        if self.stopping:
            self.tick_stop(now)
            return
        m = self.active
        if not m:
            return
        if now >= m.stamp+m.valid_for:
            self.halt(now, 'COMMAND_TIMEOUT', effective_at=m.stamp+m.valid_for)
            if self.stopping:
                self.tick_stop(now)
            return
        if now < m.stamp and not isinstance(self.path, Timeline):
            return
        elapsed = max(0., now-self.started)
        try:
            if m.mode == 'base_velocity':
                target = m.velocity if elapsed < m.duration else (0., 0.)
                self.base_reference = tuple(v + max(-self.profile.base_acceleration(axis)*dt,
                    min(self.profile.base_acceleration(axis)*dt, t-v))
                    for axis, v, t in zip(("linear", "angular"), self.base_reference, target))
                self.io.write_base(self.base_reference)
                if elapsed >= m.duration and self.base_reference == (0., 0.) and self.base_zero_requested_at is None:
                    self.base_zero_requested_at = now
                done = (elapsed >= m.duration and self.base_reference == (0., 0.) and
                        self.base_zero_requested_at is not None and self.feedback.stamp > self.base_zero_requested_at and
                        all(abs(v) <= self.profile.feedback_tolerances.get('base_'+axis+'_velocity', 0.)
                            for axis, v in zip(('linear', 'angular'), self.feedback.base_velocity)))
            else:
                self.path = trim_before(self.path, now)
                self.reference = self.path.sample(now)
                self.io.write_joints(dict(zip(m.names, self.reference.q)))
                if m.mode == 'joint_reference_segment' and elapsed >= m.offsets[-1]:
                    self.stopping = (m, 'BUFFER_EXHAUSTED')
                    self.stop_deadline = now+self.profile.stop_timeout
                    self.stop_steady_deadline = self.steady()+self.profile.stop_timeout
                    self.active = None
                    self.emit(now, m, 'STOPPING', 'BUFFER_EXHAUSTED')
                    return
                target = m.points[-1] if m.mode == 'finite_trajectory' else m.positions
                done = (m.mode == 'finite_trajectory' and now >= self.path.end and self.feedback.stamp >= self.path.end and
                        all(abs(self.feedback.positions[j]-q) < self.profile.feedback_tolerances.get('joint_position', .015)
                            and self.joint_at_rest(j)
                            for j, q in zip(m.names, target)))
            if done:
                self.emit(now, m, 'SUCCEEDED', 'FEEDBACK_CONFIRMED')
                if self.reference:
                    self.held_references.update(zip(self.reference_names, self.reference.q))
                self.active = None
                # Keep the reference at rest for the next command. A different
                # resource may now start from measured feedback.
                self.path = self.reference = None
                self.reference_names = ()
        except ValueError:
            self.halt(now, 'REFERENCE_GENERATION_FAILED', revoke=True, emergency=True)
        except RuntimeError as exc:
            self.halt(now, 'DEVICE_COMMAND_REJECTED'+
                      (':'+str(exc) if str(exc).startswith('PARTIAL_WRITE:') else ''),
                      revoke=True, emergency=True)

    def tick_stop(self, now):
        motion, code = self.stopping
        if self.stop_expired(now):
            if self.device_stop_pending:
                self.fail_stop(now, 'STOP_FEEDBACK_TIMEOUT')
            else:
                self.finish_stop(now, motion, 'STOP_FEEDBACK_TIMEOUT')
            return
        if self.device_stop_pending:
            if (self.feedback.stamp > self.stop_after_stamp and 0 <= now-self.feedback.stamp <= .2
                    and all(self.joint_at_rest(j) for j in self.profile.limits)
                    and all(abs(v) <= self.profile.feedback_tolerances.get('base_'+axis+'_velocity', 0.)
                            for axis, v in zip(('linear','angular'), self.feedback.base_velocity))):
                self.emit(now, motion, 'CANCELED' if code == 'CANCELED' else 'FAILED', code)
                self.stopping = None
                self.device_stop_pending = False
            return
        try:
            if self.path:
                self.reference = self.path.sample(now)
                self.io.write_joints(dict(zip(self.reference_names, self.reference.q)))
            if (not self.path or now >= self.path.end) and self.references_arrived():
                self.finish_stop(now, motion, code)
        except ValueError:
            self.halt(now, 'REFERENCE_GENERATION_FAILED', revoke=True, emergency=True)
        except RuntimeError as exc:
            self.halt(now, 'DEVICE_COMMAND_REJECTED'+
                      (':'+str(exc) if str(exc).startswith('PARTIAL_WRITE:') else ''),
                      revoke=True, emergency=True)

    def references_arrived(self):
        targets = dict(self.held_references)
        if self.reference:
            targets.update(zip(self.reference_names, self.reference.q))
        tolerance = self.profile.feedback_tolerances.get('joint_position', 1e-5)
        return all(abs(self.feedback.positions[j]-q) < tolerance and
                   self.joint_at_rest(j) for j, q in targets.items())

    def joint_at_rest(self, name):
        velocity = self.feedback.joint_velocities.get(name)
        return (velocity is not None and math.isfinite(velocity) and
                abs(velocity) <= self.profile.feedback_tolerances.get('joint_velocity', .01))
