import math
from bindu_contracts.devices import BaseReading


class SimBaseDriver:
    """Ideal differential-drive SE(2) fixture; no slip, localization or dynamics."""
    def __init__(self):
        self.velocity = (0., 0.)
        self.x = self.y = self.yaw = 0.
        self.fault = ''
        self.reading = BaseReading(0., 0., 0., self.velocity, True)

    def write_velocity(self, velocity):
        if self.fault == 'reject':
            raise RuntimeError('DRIVER_REJECTED')
        self.velocity = velocity

    def request_stop(self):
        self.velocity = (0., 0.)

    def read(self, now, dt):
        dt = max(0., min(dt, .05))
        linear, angular = self.velocity
        turn = angular * dt
        # Exact constant-twist integration, stable also for almost straight motion.
        half = turn / 2
        distance = linear * dt * (math.sin(half) / half if half else 1.)
        self.x += distance * math.cos(self.yaw + half)
        self.y += distance * math.sin(self.yaw + half)
        self.yaw = math.atan2(math.sin(self.yaw + turn), math.cos(self.yaw + turn))
        if self.fault != 'feedback_loss':
            self.reading = BaseReading(now, self.x, self.yaw, self.velocity, True, y=self.y)
        return self.reading

    def inject_fault(self, fault):
        if fault not in ('', 'reject', 'feedback_loss'):
            raise ValueError('UNKNOWN_FAULT')
        self.fault = fault
