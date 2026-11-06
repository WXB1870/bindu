"""Position target execution when the device owns interpolation and braking.

Retains Executor's lease, layout, limit, freshness and revision validation.
Untimed targets retain device interpolation. A capable dated-session adapter
can also accept one immutable prepared trajectory plus source TTL heartbeats.
"""
import json
import math
from dataclasses import dataclass, replace

from .executor import Executor, Rejected
from .interpolation import State, PreparedSegment


@dataclass(frozen=True)
class DirectTarget:
    state: State

    def sample(self, now):
        return self.state


@dataclass(frozen=True)
class PreparedReplay:
    effective: float
    description: tuple
    lease_id: str
    epoch: int
    inner_reference: bool
    motion: object
    hold: object = None

    @classmethod
    def create(cls, motion, profile):
        if (not 2 <= len(motion.points) <= 257 or
                len(motion.offsets) != len(motion.points) or
                len(motion.velocities) != len(motion.points) or
                len(motion.accelerations) != len(motion.points) or
                not all(math.isfinite(t) for t in motion.offsets) or
                motion.offsets[0] != 0. or
                any(a >= b for a, b in zip(motion.offsets, motion.offsets[1:]))):
            raise Rejected('PREPARED_TRAJECTORY_LAYOUT')
        states = tuple(State(tuple(q), tuple(v), tuple(a)) for q, v, a in zip(
            motion.points, motion.velocities, motion.accelerations))
        if any(len(row) != len(motion.names) or not all(math.isfinite(x) for x in row)
               for s in states for row in (s.q, s.v, s.a)):
            raise Rejected('PREPARED_TRAJECTORY_LAYOUT')
        if any(abs(x) > 1e-8 for s in (states[0], states[-1]) for row in (s.v, s.a) for x in row):
            raise Rejected('PREPARED_TRAJECTORY_ENDPOINT_NOT_REST')
        pieces = []
        for a, b, first, last in zip(motion.offsets, motion.offsets[1:], states, states[1:]):
            h = b-a
            controls = tuple((q0, q0+v0*h/5, q0+2*v0*h/5+a0*h*h/20,
                              q1-2*v1*h/5+a1*h*h/20, q1-v1*h/5, q1)
                             for q0,v0,a0,q1,v1,a1 in zip(first.q,first.v,first.a,last.q,last.v,last.a))
            pieces.append((a, h, h, 0., controls))
        # Import/certify the supplied polynomials once. No target fitting or
        # retiming; heartbeats below carry only this immutable identity.
        PreparedSegment.from_description(profile, motion.names, tuple(pieces))
        return cls(0., tuple(pieces), '', 0,
                   motion.mode.removesuffix('_impedance') == 'prepared_trajectory', motion)


class DelegatedPositionExecutor(Executor):
    replay = None
    replay_wait = None
    replay_rest = None
    replay_result = None

    def prepare_replay(self, motion, now):
        """Certify the immutable payload before granting any control authority."""
        from bindu_contracts.runtime_mode import command_mode
        command_mode(motion.runtime_mode, self.runtime_mode)
        self.tick(now)
        if self.stop_fault or self.io.stop_failures:
            raise Rejected('STOP_UNCONFIRMED')
        if self.results and next(reversed(self.results.values())).state == 'FAILED':
            raise Rejected('EXECUTION_FAILED:'+next(reversed(self.results.values())).code)
        if self.lease_id or self.active or self.stopping:
            raise Rejected('BUSY')
        if motion.mode.endswith('_impedance') != (getattr(self.io, 'execution_mode', 'position') == 'impedance'):
            raise Rejected('EXECUTION_MODE_MISMATCH')
        if (not getattr(self.io, 'supports_prepared_replay', False) or
                motion.schema_version != 2 or motion.profile_hash != self.profile.digest):
            raise Rejected('SCHEMA_OR_PROFILE_MISMATCH')
        groups = motion.resource_groups
        if (not motion.command_id or not motion.task_id or motion.lease_id or motion.epoch or
                motion.positions or motion.group or motion.rigid_reference or motion.duration or
                motion.velocity != (0., 0.) or len(groups) != len(set(groups)) or
                not groups or not set(groups) <= self.io.writable_groups or
                any(g not in self.profile.groups for g in groups) or
                any(c & set(groups) and not c <= set(groups) for c in self.io.coupled_group_sets) or
                tuple(n for g in groups for n in self.profile.groups[g]) != motion.names):
            raise Rejected('PREPARED_TRAJECTORY_LAYOUT')
        if motion.expected_revision != self.revision:
            raise Rejected('STALE_REVISION')
        self.io.validate_motion(motion, now)
        prepared = PreparedReplay.create(motion, self.profile)
        if motion.observation_id:
            held = self.io.held_reference()
            if motion.observation_id != held[0]:
                raise Rejected('REPLAY_REFERENCE_INSTANCE_CHANGED')
            prepared = replace(prepared, hold=held)
        elif getattr(self.io, 'execution_mode', 'position') == 'impedance':
            raise Rejected('PREPARED_REFERENCE_REQUIRED')
        self.io.prepare_replay(prepared)
        self.replay, self.replay_wait, self.replay_rest = prepared, None, None
        self.replay_result = None
        self.emit(now, motion, 'PREPARED', 'REPLAY_PREPARED')

    def acquire_replay(self, owner, groups, now):
        prepared = self.replay
        if (prepared is None or owner != prepared.motion.task_id or
                tuple(groups) != prepared.motion.resource_groups):
            raise Rejected('REPLAY_NOT_PREPARED')
        self.tick(now)
        if prepared.lease_id:
            self.check_lease(prepared.lease_id, prepared.epoch, now)
            return prepared.lease_id, prepared.epoch
        if self.stop_fault or self.io.stop_failures:
            raise Rejected('STOP_UNCONFIRMED')
        if self.lease_id or self.stopping:
            raise Rejected('BUSY')
        if prepared.motion.expected_revision != self.revision:
            raise Rejected('STALE_REVISION')
        if prepared.hold is not None and self.io.held_reference() != prepared.hold:
            raise Rejected('REPLAY_HELD_REFERENCE_CHANGED')
        # This deadline belongs to the request, including all polls and swings.
        if self.replay_wait is None:
            self.replay_wait = (self.steady(), None)
        began, rest = self.replay_wait
        timing = self.io.replay_timing
        if self.steady()-began >= timing['completion_timeout_s']:
            raise Rejected('REPLAY_NOT_SETTLED')
        self.io.check_health(groups, acquiring=True)
        names, feedback = prepared.motion.names, self.feedback
        quiet = all(n in feedback.joint_velocities and
                    math.isfinite(feedback.joint_velocities[n]) and
                    abs(feedback.joint_velocities[n]) <= timing['velocity_tolerance_rad_s'] for n in names)
        if not quiet:
            rest = None
        elif (rest is None or feedback.stamp < rest[1] or
              feedback.stamp-rest[1] > self.io.stop_confirmation_max_gap):
            rest = (self.steady(), feedback.stamp)
        elif feedback.stamp > rest[1] and self.steady()-rest[0] >= timing['settle_time_s']:
            lease, epoch = self.acquire(owner, groups, 2., now)
            self.replay = replace(prepared, lease_id=lease, epoch=epoch)
            self.replay_result = dict(completed=False, final=False, stop_confirmed=False,
                measured_arrival_confirmed=False, nominal_arrival_confirmed=False,
                measured_end_error_rad=None, error=None, details=[])
            return lease, epoch
        elif feedback.stamp > rest[1]:
            rest = (rest[0], feedback.stamp)
        self.replay_wait = (began, rest)
        raise Rejected('WAIT_REST:'+('MEASURED_SPEED' if not quiet else 'REST_WINDOW'))

    def replay_status(self, lease, epoch, now, *, stop=False):
        prepared = self.replay
        if prepared is None or not lease or (lease, epoch) != (prepared.lease_id, prepared.epoch):
            raise Rejected('INVALID_LEASE')
        if stop and not self.replay_result['final'] and not self.stopping:
            self.halt(now, revoke=True)
        return self.replay_result

    def emit(self, now, motion, state, code):
        prepared, result = self.replay, self.replay_result
        if (prepared is not None and result is not None and not result['final'] and motion is not None
                and motion.task_id == prepared.motion.task_id and state in ('FAILED', 'CANCELED', 'SUCCEEDED')):
            result.update(final=True, stop_confirmed=not bool(self.stop_fault or self.io.stop_failures),
                          completed=state == 'SUCCEEDED', state=state, code=code)
            if state in ('FAILED', 'CANCELED'):
                result['error'] = code
            result['details'].extend(str(x) for x in self.io.stop_failures)
            super().emit(now, prepared.motion, 'REPLAY_RESULT', json.dumps(result, allow_nan=False))
        super().emit(now, motion, state, code)

    def tick(self, now):
        super().tick(now)
        prepared = self.replay
        if (prepared is None or not prepared.effective or self.replay_result is None or self.replay_result['final']
                or not self.active or self.stopping):
            return
        end = prepared.effective+prepared.motion.offsets[-1]
        if now < end:
            return
        timing, names = self.io.replay_timing, prepared.motion.names
        q, v = self.feedback.positions, self.feedback.joint_velocities
        goal = prepared.motion.points[-1]
        errors = {n:q[n]-x for n,x in zip(names, goal)}
        tolerance = self.io.replay_position_tolerances
        if not prepared.inner_reference:
            tolerance = {n:max(t, self.io.replay_prepare_tolerance) for n,t in tolerance.items()}
        measured = all(abs(errors[n]) <= tolerance[n] for n in names)
        nominal = getattr(self.io, 'execution_mode', 'position') == 'impedance'
        reference = self.io.sampled_reference()[1] if nominal else q
        arrived = (all(n in reference and abs(reference[n]-x) <= tolerance[n]
                       for n,x in zip(names, goal)) and
                   all(n in v and math.isfinite(v[n]) and abs(v[n]) <= timing['velocity_tolerance_rad_s'] for n in names)
                   and self.feedback.stamp >= end)
        previous = self.replay_rest
        if not arrived:
            self.replay_rest = None
        elif (previous is None or self.feedback.stamp < previous[1] or
              self.feedback.stamp-previous[1] > self.io.stop_confirmation_max_gap):
            self.replay_rest = (self.steady(), self.feedback.stamp)
        elif self.feedback.stamp > previous[1]:
            self.replay_rest = (previous[0], self.feedback.stamp)
            if self.steady()-previous[0] >= timing['settle_time_s']:
                self.replay_result.update(measured_arrival_confirmed=measured,
                    nominal_arrival_confirmed=nominal, measured_end_error_rad=errors)
                self.halt(now, 'REPLAY_COMPLETED', revoke=True)
                return
        if now-end >= timing['completion_timeout_s']:
            self.halt(now, 'REPLAY_ARRIVAL_UNCONFIRMED', revoke=True)

    def acquire(self, owner, groups, ttl, now):
        if self.stop_fault or self.io.stop_failures:
            raise Rejected('STOP_UNCONFIRMED')
        if not set(groups) <= self.io.writable_groups:
            raise Rejected('VENDOR_RESOURCE_UNAVAILABLE')
        for coupled in self.io.coupled_group_sets:
            if coupled & set(groups) and not coupled <= set(groups):
                raise Rejected('VENDOR_COUPLED_OWNERSHIP_REQUIRED')
        if self.lease_id or self.stopping:
            raise Rejected('BUSY')
        try:
            self.io.capture(groups)
        except RuntimeError as exc:
            raise Rejected(str(exc)) from exc
        return super().acquire(owner, groups, ttl, now)

    def submit(self, motion, now):
        if motion.mode in ('prepared_trajectory', 'prepared_entry', 'prepared_trajectory_impedance', 'prepared_entry_impedance'):
            try:
                self.prepare_replay(motion, now)
            except RuntimeError as exc:
                raise Rejected(str(exc)) from exc
            return
        if motion.mode == 'trajectory_heartbeat':
            from bindu_contracts.runtime_mode import command_mode
            command_mode(motion.runtime_mode, self.runtime_mode)
            self.tick(now)
            self.check_lease(motion.lease_id, motion.epoch, now)
            if self.stopping:
                raise Rejected('STOPPING')
            prepared = self.replay
            if (prepared is None or motion.observation_id != prepared.motion.command_id or
                    motion.task_id != prepared.motion.task_id or motion.schema_version != 2 or
                    motion.profile_hash != self.profile.digest or motion.names != prepared.motion.names or
                    motion.resource_groups != prepared.motion.resource_groups or motion.group or
                    motion.lease_id != prepared.lease_id or motion.epoch != prepared.epoch or
                    motion.points or motion.offsets or motion.velocities or motion.accelerations):
                raise Rejected('PREPARED_TRAJECTORY_IDENTITY')
            if motion.positions or motion.rigid_reference or motion.duration or motion.velocity != (0., 0.):
                raise Rejected('MIXED_COMMAND_PAYLOAD')
            if not motion.command_id or motion.command_id in self.results:
                raise Rejected('DUPLICATE_OR_EMPTY_ID')
            if motion.expected_revision and motion.expected_revision != self.revision:
                raise Rejected('STALE_REVISION')
            if motion.stamp < self.last_stamp or motion.valid_for > self.io.replay_timing['command_ttl_s']:
                raise Rejected('VENDOR_TARGET_TIME')
            try:
                self.io.validate_motion(motion, now)
            except RuntimeError as exc:
                raise Rejected(str(exc)) from exc
            source = motion
            if not prepared.effective:
                prepared = self.replay = replace(prepared,
                    effective=motion.stamp+self.io.replay_timing['start_lead_s'])
                source = replace(prepared.motion, command_id=motion.command_id,
                    stamp=motion.stamp, valid_for=motion.valid_for,
                    lease_id=motion.lease_id, epoch=motion.epoch)
                self.path = DirectTarget(State.rest(prepared.motion.points[0]))
                self.reference_names = prepared.motion.names
                self.reference = self.path.sample(now)
            self.active = replace(motion, mode='joint_target', positions=prepared.motion.points[0])
            self.started, self.last_stamp = motion.stamp, motion.stamp
            self.io.source_motion, self.io.prepared_replay = source, prepared
            self.revision += 1
            self.emit(now, motion, 'ACCEPTED', 'OK')
            return
        if self.replay_result is not None and not self.replay_result['final']:
            raise Rejected('REPLAY_REQUIRES_HEARTBEAT')
        if motion.mode != 'joint_target':
            raise Rejected('VENDOR_REQUIRES_UNTIMED_JOINT_TARGET')
        if motion.stamp > now or motion.valid_for > .4:
            raise Rejected('VENDOR_TARGET_TIME')
        validate = getattr(self.io, 'validate_motion', None)
        if validate is not None:
            try:
                validate(motion, now)
            except RuntimeError as exc:
                raise Rejected(str(exc)) from exc
        result = super().submit(motion, now)
        # Ordinary target streams retain the original source deadline too.
        if hasattr(self.io, 'source_motion'):
            self.io.source_motion, self.io.prepared_replay = motion, None
        return result

    def prepare_path(self, motion, now):
        return DirectTarget(State.rest(motion.positions))

    def halt(self, now, code='CANCELED', revoke=False, emergency=False, effective_at=None):
        if self.replay_result is not None and not self.replay_result['final']:
            revoke = True
        if revoke or emergency:
            if self.lease_id:
                self.epoch += 1
            self.lease_id = self.owner = ''
        if self.stop_fault:
            return
        if self.stopping and self.stop_expired(now):
            self.io.stop_failures = tuple(sorted(set(self.io.stop_failures) | {'vendor_hold'}))
            self.fail_stop(now, 'STOP_UNCONFIRMED:'+','.join(self.io.stop_failures))
            return
        if self.stopping and not emergency:
            return
        if self.stopping and self.stopping[1] not in ('CANCELED', 'REPLAY_COMPLETED'):
            code = self.stopping[1]
        # Keep both original budgets across repeated faults and clock changes.
        if not self.stopping:
            self.stop_deadline = now+self.profile.stop_timeout
            self.stop_steady_deadline = self.steady()+self.profile.stop_timeout
            self.stop_rest_window = None
        motion = self.active or (self.stopping[0] if self.stopping else None)
        if motion is None and self.replay_result is not None and not self.replay_result['final']:
            motion = self.replay.motion
        self.last_stamp = max(self.last_stamp, now if math.isfinite(now) else self.last_stamp)
        self.revision += 1
        try:
            self.io.stop()
        except RuntimeError:
            self.io.stop_failures = tuple(sorted(set(self.io.stop_failures) | {'vendor_hold'}))
        self.active = self.path = None
        self.held_references.clear()
        self.reference_names = tuple(self.io.stop_targets)
        self.reference = State.rest(self.io.stop_targets.values()) if self.reference_names else None
        if self.reference_names and self.stop_expired(now):
            self.io.stop_failures = tuple(sorted(set(self.io.stop_failures) | {'vendor_hold'}))
        if self.io.stop_failures:
            self.stopping = (motion, code)
            self.fail_stop(now, 'STOP_UNCONFIRMED:'+','.join(self.io.stop_failures))
        elif self.reference_names:
            self.stopping = (motion, code)
            self.emit(now, motion, 'STOPPING', code)
        else:
            self.stopping = None
            self.emit(now, motion, 'SUCCEEDED' if code == 'REPLAY_COMPLETED' else 'CANCELED' if code == 'CANCELED' else 'FAILED', code)

    def tick_stop(self, now):
        motion, code = self.stopping
        # Late feedback must not erase an expired stop budget.
        if self.stop_expired(now):
            self.io.stop_failures = tuple(sorted(set(self.io.stop_failures) | {'vendor_hold'}))
            self.fail_stop(now, 'STOP_UNCONFIRMED:'+','.join(self.io.stop_failures))
            return
        # Some position devices acknowledge the actual braking/hold point.
        # Its ACK fixes the braking reference; it does not prove physical rest.
        confirm = getattr(self.io, 'confirmed_stop_targets', None)
        if confirm is not None:
            targets = confirm()
            if targets is None:
                self.stop_rest_window = None
                return
            if (set(targets) != set(self.reference_names) or
                    any(not math.isfinite(q) for q in targets.values())):
                self.fail_stop(now, 'STOP_REFERENCE_INVALID')
                return
            self.reference = State.rest(targets[n] for n in self.reference_names)
        # A sent hold is only a request. Require feedback newer than that request.
        require_arrival = getattr(self.io, 'stop_requires_arrival', True)
        arrival_names = None if require_arrival else tuple(getattr(self.io, 'stop_arrival_joints', ()))
        if arrival_names is not None and not set(arrival_names) <= set(self.reference_names):
            self.fail_stop(now, 'STOP_REFERENCE_INVALID')
            return
        resting = all(n in self.feedback.positions and math.isfinite(self.feedback.positions[n])
                      and self.joint_at_rest(n) for n in self.reference_names)
        arrived = (self.feedback.stamp > self.io.stop_requested_at and
                   0 <= now-self.feedback.stamp <= .2 and resting and
                   self.references_arrived(arrival_names,
                       position_tolerances=self.profile.stop_position_tolerances))
        window = getattr(self.io, 'stop_confirmation_window', 0.)
        if arrived and window:
            positions = {n: self.feedback.positions[n] for n in self.reference_names}
            previous = self.stop_rest_window
            tolerance = self.profile.feedback_tolerances.get('joint_position', 1e-5)
            max_gap = getattr(self.io, 'stop_confirmation_max_gap', window)
            if (previous is None or self.feedback.stamp < previous[1] or
                    self.feedback.stamp-previous[1] > max_gap or
                    any(abs(q-previous[2][n]) >= tolerance for n, q in positions.items())):
                self.stop_rest_window = (self.steady(), self.feedback.stamp, positions)
                return
            if self.feedback.stamp <= previous[1]:
                return  # Repeated feedback cannot complete a measured-rest window.
            self.stop_rest_window = (previous[0], self.feedback.stamp, previous[2])
            if self.steady()-previous[0] < window:
                return
        elif not arrived:
            self.stop_rest_window = None
        if arrived:
            self.io.stop_targets.clear()
            self.stopping = self.reference = None
            self.reference_names = ()
            self.emit(now, motion, 'SUCCEEDED' if code == 'REPLAY_COMPLETED' else 'CANCELED' if code == 'CANCELED' else 'FAILED', code)
