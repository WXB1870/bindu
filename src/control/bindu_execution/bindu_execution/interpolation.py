"""Python contracts for the shared C++ reference engine; no numerical fallback.

The native module owns all curve, limit, adaptive-stream and braking math.
Python keeps immutable timeline selection and the ROS-independent public state.
Build with colcon (or setup.py build_ext --inplace for local tests) before import.
"""
from dataclasses import dataclass
import math
from . import _native


@dataclass(frozen=True)
class State:
    q: tuple
    v: tuple
    a: tuple
    j: tuple = ()

    @classmethod
    def rest(cls, q):
        q = tuple(q)
        return cls(q, (0.,)*len(q), (0.,)*len(q), (0.,)*len(q))


def _state(state):
    return state.q, state.v, state.a


def _limits(profile, names):
    return tuple((*profile.limits[n], profile.speed(n), profile.acceleration(n), profile.jerk(n)) for n in names)


@dataclass(frozen=True)
class Curve:
    _handle: object
    start: float
    duration: float

    @classmethod
    def _wrap(cls, handle):
        return cls(handle, *_native.curve_info(handle))

    @classmethod
    def between(cls, start, duration, initial, final):
        return cls._wrap(_native.between(start, duration, _state(initial), _state(final)))

    @property
    def end(self):
        return self.start+self.duration

    def sample(self, now):
        return State(*_native.curve_sample(self._handle, now))

    def valid(self, profile, names):
        return _native.curve_valid(self._handle, _limits(profile, names))


@dataclass(frozen=True)
class Path:
    curves: tuple

    @classmethod
    def _wrap(cls, handles):
        return cls(tuple(Curve._wrap(h) for h in handles))

    @property
    def end(self):
        return self.curves[-1].end

    def sample(self, now):
        if not math.isfinite(now):
            raise ValueError('INVALID_SAMPLE_TIME')
        for curve in self.curves:
            if now < curve.end:
                return curve.sample(now)
        final = self.curves[-1].sample(self.end)
        return State(final.q, final.v, final.a, (0.,)*len(final.q))


def join(prefix, suffix, switch):
    # A replacement at the same (or earlier) boundary makes these suffixes
    # unreachable. Retain only the part that can execute before the new switch.
    while isinstance(prefix, Timeline) and switch <= prefix.switch:
        prefix = prefix.before
    return Timeline(prefix, suffix, switch) if prefix else suffix


def trim_before(path, now):
    """Release elapsed branches, preserving samples at/after now and snapshots.

    Walk inside pending switches too: the outermost switch may stay in the
    future forever during streaming. Rebuild only changed immutable ancestors.
    Call after expiry handling, which may need to sample an earlier deadline.
    """
    if not isinstance(path, Timeline):
        return path
    pending = []
    while isinstance(path, Timeline):
        if now >= path.switch:
            path = path.after
        else:
            pending.append(path)
            path = path.before
    for node in reversed(pending):
        path = node if path is node.before else Timeline(path, node.after, node.switch)
    return path


@dataclass(frozen=True)
class Timeline:
    before: object
    after: object
    switch: float

    @property
    def end(self):
        return self.after.end

    def sample(self, now):
        path = self
        while isinstance(path, Timeline):
            path = path.before if now < path.switch else path.after
        return path.sample(now)


@dataclass(frozen=True)
class Online:
    _handle: object
    # A continuous target is not a finite task, even after reference arrival.
    end = float('inf')

    def sample(self, now):
        return State(*_native.online_sample(self._handle, now))


def fit_online(profile, names, state, target, start):
    return Online(_native.online(_limits(profile, names), _state(state), target, start,
                                 profile.control_period, profile.max_transition_duration))


def fit_target(profile, names, state, target, start):
    """Non-timed synchronized P2P; adaptive streaming uses fit_online instead."""
    return Path._wrap(_native.fit_target(_limits(profile, names), _state(state), target, start,
                                         profile.max_transition_duration, profile.stop_timeout))


def fit_stop(profile, names, state, start):
    return Path._wrap(_native.fit_stop(_limits(profile, names), _state(state), start, profile.stop_timeout))


def fit_timed(profile, names, initial, states, times, start, chunk=False):
    return Path._wrap(_native.fit_timed(_limits(profile, names), _state(initial),
        tuple(_state(s) for s in states), times, start, chunk, profile.stop_timeout))
