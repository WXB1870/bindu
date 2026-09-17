"""Bounded C2 joint references, independent of ROS and device tracking.

Quintic Bezier curves expose derivatives of the *same* position polynomial.
Subdivision certifies whole-interval bounds using convex hulls (not a sampled
limit check). Conservative rejection is preferable to accepting an overshoot.
This is a reference implementation, not a hard real-time motion controller.
"""
from dataclasses import dataclass
import math


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


def evaluate(values, u):
    values = list(values)
    while len(values) > 1:
        values = [(1-u)*a+u*b for a, b in zip(values, values[1:])]
    return values[0]


def derivative(values, duration):
    n = len(values)-1
    return tuple(n*(b-a)/duration for a, b in zip(values, values[1:]))


def bounded(values, lo, hi, depth=9):
    # The tolerance only absorbs roundoff at exact endpoint limits.
    eps = 1e-9
    if min(values) >= lo-eps and max(values) <= hi+eps:
        return True
    if values[0] < lo-eps or values[0] > hi+eps or values[-1] < lo-eps or values[-1] > hi+eps:
        return False
    if depth == 0:
        return False
    row = list(values)
    left, right = [row[0]], [row[-1]]
    while len(row) > 1:
        row = [(a+b)*.5 for a, b in zip(row, row[1:])]
        left.append(row[0])
        right.append(row[-1])
    return bounded(left, lo, hi, depth-1) and bounded(right[::-1], lo, hi, depth-1)


@dataclass(frozen=True)
class Curve:
    start: float
    duration: float
    controls: tuple

    @classmethod
    def between(cls, start, duration, initial, final):
        if not math.isfinite(duration) or duration < 1e-5:
            raise ValueError('TRAJECTORY_INTERVAL_TOO_SHORT')
        axes = []
        for q, v, a, p, w, b in zip(initial.q, initial.v, initial.a, final.q, final.v, final.a):
            axes.append((q, q+v*duration/5, q+2*v*duration/5+a*duration**2/20,
                         p-2*w*duration/5+b*duration**2/20, p-w*duration/5, p))
        return cls(start, duration, tuple(axes))

    @property
    def end(self):
        return self.start+self.duration

    def sample(self, now):
        u = min(1., max(0., (now-self.start)/self.duration))
        columns = [[], [], [], []]
        for axis in self.controls:
            for order in range(4):
                columns[order].append(evaluate(axis, u))
                axis = derivative(axis, self.duration)
        return State(*(tuple(c) for c in columns))

    def valid(self, profile, names):
        for name, axis in zip(names, self.controls):
            limits = (profile.limits[name], (-profile.speed(name), profile.speed(name)),
                      (-profile.acceleration(name), profile.acceleration(name)),
                      (-profile.jerk(name), profile.jerk(name)))
            for lo, hi in limits:
                if not bounded(axis, lo, hi):
                    return False
                axis = derivative(axis, self.duration)
        return True


@dataclass(frozen=True)
class Path:
    curves: tuple

    @property
    def end(self):
        return self.curves[-1].end

    def sample(self, now):
        for curve in self.curves:
            if now < curve.end:
                return curve.sample(now)
        final = self.curves[-1].sample(self.end)
        # Every executable path ends at rest. Never silently zero a derivative.
        return State(final.q, final.v, final.a, (0.,)*len(final.q))


def join(prefix, suffix, switch):
    # Clip the *selection interval*, preserving the original polynomial.
    return Timeline(prefix, suffix, switch) if prefix else suffix


@dataclass(frozen=True)
class Timeline:
    before: object
    after: object
    switch: float

    @property
    def end(self):
        return self.after.end

    def sample(self, now):
        return (self.before if now < self.switch else self.after).sample(now)


def fit_target(profile, names, state, target, start):
    if any(v*(p-q) < -1e-8 for v, p, q in zip(state.v, target, state.q)):
        # Opposite-side retargeting first removes the current momentum. A long
        # single polynomial can otherwise keep moving the wrong way too long.
        brake = fit_stop(profile, names, state, start)
        continuation = fit_target(profile, names, brake.sample(brake.end), target, brake.end)
        return Path(brake.curves+continuation.curves)
    distance = max(abs(p-q)/profile.speed(n) for n, p, q in zip(names, target, state.q))
    duration = max(.02, distance)
    # Duration dilation is only allowed for online goals, never timed paths.
    for _ in range(70):
        if duration > profile.max_transition_duration:
            break
        curve = Curve.between(start, duration, state, State.rest(target))
        if curve.valid(profile, names):
            return Path((curve,))
        duration *= 1.12
        if duration > profile.max_transition_duration:
            break
    # A single quintic can be infeasible near a position boundary during a
    # reversal even though a bounded brake followed by a new move is feasible.
    # Do not keep chasing an obsolete goal merely because one polynomial fails.
    if any(abs(x) > 1e-8 for x in state.v+state.a):
        brake = fit_stop(profile, names, state, start)
        continuation = fit_target(profile, names, brake.sample(brake.end), target, brake.end)
        return Path(brake.curves+continuation.curves)
    raise ValueError('NO_FEASIBLE_CONTINUATION')


def fit_stop(profile, names, state, start):
    duration = .02
    for _ in range(70):
        if duration > profile.stop_timeout:
            break
        target = tuple(q+v*duration/2+a*duration**2/12 for q, v, a in zip(state.q, state.v, state.a))
        curve = Curve.between(start, duration, state, State.rest(target))
        if curve.valid(profile, names):
            return Path((curve,))
        duration *= 1.12
        if duration > profile.stop_timeout:
            break
    raise ValueError('NO_FEASIBLE_STOP')
