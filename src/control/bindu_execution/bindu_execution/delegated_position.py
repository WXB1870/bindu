"""Position target execution when the device owns interpolation and braking.

Retains Executor's lease, layout, limit, freshness and revision validation.
Only untimed joint_target is supported: timed derivative trajectories cannot
be honored by an un-timestamped position interface.
"""
import math
from dataclasses import dataclass

from .executor import Executor, Rejected
from .interpolation import State


@dataclass(frozen=True)
class DirectTarget:
    state: State

    def sample(self, now):
        return self.state


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
        if motion.mode != 'joint_target':
            raise Rejected('VENDOR_REQUIRES_UNTIMED_JOINT_TARGET')
        if motion.stamp > now or motion.valid_for > .4:
            raise Rejected('VENDOR_TARGET_TIME')
        return super().submit(motion, now)

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
        # Its ACK changes the expected position, never the stop budget or the
        # required measured position/velocity tolerances.
        confirm = getattr(self.io, 'confirmed_stop_targets', None)
        if confirm is not None:
            targets = confirm()
            if targets is None:
                return
            if (set(targets) != set(self.reference_names) or
                    any(not math.isfinite(q) for q in targets.values())):
                self.fail_stop(now, 'STOP_REFERENCE_INVALID')
                return
            self.reference = State.rest(targets[n] for n in self.reference_names)
        # A sent hold is only a request. Require feedback newer than that request.
        if (self.feedback.stamp > self.io.stop_requested_at and
                0 <= now-self.feedback.stamp <= .2 and self.references_arrived()):
            self.io.stop_targets.clear()
            self.stopping = self.reference = None
            self.reference_names = ()
            self.emit(now, motion, 'CANCELED' if code == 'CANCELED' else 'FAILED', code)
