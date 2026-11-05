"""Position target execution when the device owns interpolation and braking.

Retains Executor's lease, layout, limit, freshness and revision validation.
Untimed targets retain device interpolation. A capable dated-session adapter
can also accept one immutable prepared trajectory plus source TTL heartbeats.
"""
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
    identity: str
    effective: float
    description: tuple
    initial: tuple
    lease_id: str
    epoch: int
    inner_reference: bool

    @classmethod
    def create(cls, motion, profile):
        if (not 2 <= len(motion.points) <= 257 or
                len(motion.offsets) != len(motion.points) or
                len(motion.velocities) != len(motion.points) or
                len(motion.accelerations) != len(motion.points) or
                not all(math.isfinite(t) for t in motion.offsets) or
                not 0 < motion.offsets[0] <= .15 or
                any(a >= b for a, b in zip(motion.offsets, motion.offsets[1:]))):
            raise Rejected('PREPARED_TRAJECTORY_LAYOUT')
        states = tuple(State(tuple(q), tuple(v), tuple(a)) for q, v, a in zip(
            motion.points, motion.velocities, motion.accelerations))
        if any(len(row) != len(motion.names) or not all(math.isfinite(x) for x in row)
               for s in states for row in (s.q, s.v, s.a)):
            raise Rejected('PREPARED_TRAJECTORY_LAYOUT')
        if any(abs(x) > 1e-8 for s in (states[0], states[-1]) for row in (s.v, s.a) for x in row):
            raise Rejected('PREPARED_TRAJECTORY_ENDPOINT_NOT_REST')
        origin = motion.offsets[0]
        pieces = []
        for a, b, first, last in zip(motion.offsets, motion.offsets[1:], states, states[1:]):
            h = b-a
            controls = tuple((q0, q0+v0*h/5, q0+2*v0*h/5+a0*h*h/20,
                              q1-2*v1*h/5+a1*h*h/20, q1-v1*h/5, q1)
                             for q0,v0,a0,q1,v1,a1 in zip(first.q,first.v,first.a,last.q,last.v,last.a))
            pieces.append((a-origin, h, h, 0., controls))
        # Import/certify the supplied polynomials once. No target fitting or
        # retiming; heartbeats below carry only this immutable identity.
        PreparedSegment.from_description(profile, motion.names, tuple(pieces))
        return cls(motion.command_id, motion.stamp+origin, tuple(pieces), states[0].q,
                   motion.lease_id, motion.epoch, motion.mode == 'prepared_trajectory')


class DelegatedPositionExecutor(Executor):
    def acquire(self, owner, groups, ttl, now):
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
        original, prepared = motion, None
        if motion.mode in ('prepared_trajectory', 'prepared_entry', 'trajectory_heartbeat'):
            if not getattr(self.io, 'supports_prepared_replay', False):
                raise Rejected('PREPARED_TRAJECTORY_UNSUPPORTED')
            if motion.mode != 'trajectory_heartbeat':
                try:
                    prepared = PreparedReplay.create(motion, self.profile)
                except ValueError as exc:
                    raise Rejected(str(exc)) from exc
            else:
                prepared = getattr(self.io, 'prepared_replay', None)
                if (prepared is None or motion.observation_id != prepared.identity or
                        motion.lease_id != prepared.lease_id or motion.epoch != prepared.epoch or
                        motion.points or motion.offsets or motion.velocities or motion.accelerations):
                    raise Rejected('PREPARED_TRAJECTORY_IDENTITY')
            if motion.positions:
                raise Rejected('MIXED_COMMAND_PAYLOAD')
            motion = replace(motion, mode='joint_target', positions=prepared.initial,
                             points=(), offsets=(), velocities=(), accelerations=())
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
        # Devices with a dated session protocol retain the accepted source
        # identity. Repeated executor ticks must not mint fresh motion packets.
        if hasattr(self.io, 'source_motion'):
            self.io.source_motion = original
            self.io.prepared_replay = prepared
        return result

    def prepare_path(self, motion, now):
        return DirectTarget(State.rest(motion.positions))

    def halt(self, now, code='CANCELED', revoke=False, emergency=False, effective_at=None):
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
        # Keep both original budgets across repeated faults and clock changes.
        if not self.stopping:
            self.stop_deadline = now+self.profile.stop_timeout
            self.stop_steady_deadline = self.steady()+self.profile.stop_timeout
            self.stop_rest_window = None
        motion = self.active or (self.stopping[0] if self.stopping else None)
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
            self.emit(now, motion, 'CANCELED' if code == 'CANCELED' else 'FAILED', code)

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
            self.emit(now, motion, 'CANCELED' if code == 'CANCELED' else 'FAILED', code)
