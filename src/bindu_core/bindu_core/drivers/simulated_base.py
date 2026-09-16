from .interfaces import BaseReading


class SimBaseDriver:
    """Velocity-controlled wheel fixture; no localization or dynamics model."""
    def __init__(self):
        self.velocity = (0., 0.)
        self.x = self.yaw = 0.
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
        self.x += self.velocity[0] * dt
        self.yaw += self.velocity[1] * dt
        if self.fault != 'feedback_loss':
            self.reading = BaseReading(now, self.x, self.yaw, self.velocity, True)
        return self.reading

    def inject_fault(self, fault):
        if fault not in ('', 'reject', 'feedback_loss'):
            raise ValueError('UNKNOWN_FAULT')
        self.fault = fault
