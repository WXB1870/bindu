from dataclasses import dataclass, field
import hashlib
import json
import math
from pathlib import Path
from typing import Dict, List, Tuple


@dataclass(frozen=True)
class Profile:
    name: str
    groups: Dict[str, List[str]]
    limits: Dict[str, List[float]]
    max_speed: float
    capabilities: Tuple[str, ...]
    digest: str
    max_acceleration: float = 20.
    max_jerk: float = 400.
    control_period: float = .01
    max_tick_gap: float = .1
    stop_timeout: float = 5.
    joint_dynamics: dict = field(default_factory=dict)
    max_transition_duration: float = 30.

    def speed(self, name):
        return self.joint_dynamics.get(name, {}).get('velocity', self.max_speed)

    def acceleration(self, name):
        return self.joint_dynamics.get(name, {}).get('acceleration', self.max_acceleration)

    def jerk(self, name):
        return self.joint_dynamics.get(name, {}).get('jerk', self.max_jerk)

    @classmethod
    def load(cls, path):
        raw = json.loads(Path(path).read_text())
        names = [j for group in raw['groups'].values() for j in group]
        if len(names) != len(set(names)) or not names:
            raise ValueError('joint names must be unique and nonempty')
        for name in names:
            lo, hi = raw['limits'][name]
            if not all(math.isfinite(x) for x in (lo, hi)) or not lo <= 0 <= hi or lo >= hi:
                raise ValueError('simulation limits must contain zero')
        if not math.isfinite(raw['max_speed']) or raw['max_speed'] <= 0:
            raise ValueError('invalid maximum speed')
        digest = hashlib.sha256(json.dumps(raw, sort_keys=True).encode()).hexdigest()
        cfg = raw.get('execution', {})
        allowed = {'max_acceleration', 'max_jerk', 'control_period', 'max_tick_gap', 'stop_timeout', 'joint_dynamics', 'max_transition_duration'}
        if set(cfg)-allowed:
            raise ValueError('unknown execution setting')
        profile = cls(raw['name'], raw['groups'], raw['limits'], raw['max_speed'],
                      tuple(raw['capabilities']), digest, **cfg)
        for value in (profile.max_acceleration, profile.max_jerk, profile.control_period,
                      profile.max_tick_gap, profile.stop_timeout, profile.max_transition_duration):
            if type(value) not in (float, int) or not math.isfinite(value) or value <= 0:
                raise ValueError('invalid execution limit')
        if not profile.control_period <= profile.max_tick_gap < profile.stop_timeout:
            raise ValueError('invalid execution timing')
        if set(profile.joint_dynamics)-set(names):
            raise ValueError('unknown dynamics joint')
        for limits in profile.joint_dynamics.values():
            if set(limits)-{'velocity', 'acceleration', 'jerk'} or any(
                    type(v) not in (float, int) or not math.isfinite(v) or v <= 0 for v in limits.values()):
                raise ValueError('invalid joint dynamics')
        return profile
