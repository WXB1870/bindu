import math
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
        for group, reading in readings.items():
            if set(reading.positions) != set(self.groups[group]):
                raise RuntimeError('FEEDBACK_LAYOUT_MISMATCH')
        stamps = {name: reading.stamp for name, reading in readings.items()}
        stamps['base'] = base.stamp
        positions = {joint: readings[group].positions[joint] for group, names in self.groups.items() for joint in names}
        values = list(stamps.values()) + list(positions.values()) + [base.x, base.y, base.yaw, *base.velocity]
        if not all(math.isfinite(value) for value in values) or any(t > now + .05 for t in stamps.values()):
            raise RuntimeError('INVALID_DEVICE_FEEDBACK')
        self.source_stamps = stamps
        self.last = Feedback(min(stamps.values()), positions, base.x, base.yaw, base.velocity,
                             base.simulated and all(r.simulated for r in readings.values()), base_y=base.y)
        return self.last
