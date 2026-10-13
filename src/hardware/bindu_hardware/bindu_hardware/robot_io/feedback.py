import math
from collections.abc import Mapping
from bindu_contracts.contracts import Feedback


class FeedbackCollector:
    """Combine independently timestamped readings without refreshing stale data."""
    def __init__(self, profile, joint_drivers, base_driver):
        self.groups = profile.groups
        self.joints, self.base = joint_drivers, base_driver
        self.last = Feedback(0., {j: 0. for names in self.groups.values() for j in names}, 0., 0., (0., 0.))
        self.source_stamps = {}

    def read(self, now, dt):
        readings = {name: driver.read(now, dt) for name, driver in self.joints.items()}
        base = self.base.read(now, dt)
        modes = [getattr(base,'simulated',None)]+[getattr(reading,'simulated',None) for reading in readings.values()]
        if any(type(mode) is not bool or mode != modes[0] for mode in modes):
            raise RuntimeError('FEEDBACK_ENVIRONMENT_MISMATCH')
        if not isinstance(base.velocity, (tuple,list)) or len(base.velocity) != 2:
            raise RuntimeError('FEEDBACK_BASE_VELOCITY_LAYOUT')
        for group, reading in readings.items():
            if not isinstance(reading.positions, Mapping) or set(reading.positions) != set(self.groups[group]):
                raise RuntimeError('FEEDBACK_LAYOUT_MISMATCH')
            if not isinstance(reading.velocities, Mapping) or (reading.velocities and set(reading.velocities) != set(reading.positions)):
                raise RuntimeError('FEEDBACK_VELOCITY_LAYOUT_MISMATCH')
        stamps = {name: reading.stamp for name, reading in readings.items()}
        stamps['base'] = base.stamp
        positions = {joint: readings[group].positions[joint] for group, names in self.groups.items() for joint in names}
        velocities = {n: v for reading in readings.values() for n, v in reading.velocities.items()}
        values = list(stamps.values()) + list(positions.values()) + list(velocities.values()) + [base.x, base.y, base.yaw, *base.velocity]
        try:
            valid = all(type(value) is not bool and math.isfinite(value) for value in values)
        except (TypeError, ValueError):
            valid = False
        if not valid or any(t > now + .05 for t in stamps.values()):
            raise RuntimeError('INVALID_DEVICE_FEEDBACK')
        if any(t < self.source_stamps.get(name, t) for name,t in stamps.items()):
            raise RuntimeError('FEEDBACK_TIME_REGRESSION')
        self.source_stamps = stamps
        self.last = Feedback(min(stamps.values()), positions, base.x, base.yaw, base.velocity,
                             base.simulated and all(r.simulated for r in readings.values()), base_y=base.y,
                             joint_velocities=velocities)
        return self.last
