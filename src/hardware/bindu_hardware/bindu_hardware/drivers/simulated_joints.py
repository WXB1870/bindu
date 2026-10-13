from bindu_contracts.devices import JointReading


class SimJointDriver:
    """Bounded-speed kinematic fixture for one named joint group."""
    def __init__(self, names, max_speed, initial_positions=None):
        self.positions = {name: float((initial_positions or {}).get(name,0.)) for name in names}
        self.target = dict(self.positions)
        self.max_speed = max_speed
        self.fault = ''
        self.velocities = dict.fromkeys(self.positions, 0.)
        self.reading = JointReading(0., dict(self.positions), True, dict(self.velocities))

    def write_positions(self, target):
        if set(target) != set(self.positions):
            raise RuntimeError('DRIVER_JOINT_LAYOUT')
        if self.fault == 'reject':
            raise RuntimeError('DRIVER_REJECTED')
        self.target = dict(target)

    def request_stop(self):
        self.target = dict(self.positions)
        self.velocities = dict.fromkeys(self.positions, 0.)

    def read(self, now, dt):
        dt = max(0., min(dt, .05))
        for joint, target in self.target.items():
            delta = max(-self.max_speed*dt, min(self.max_speed*dt, target-self.positions[joint]))
            self.positions[joint] += delta
            if dt > 0:
                self.velocities[joint] = delta/dt
        if self.fault != 'feedback_loss':
            self.reading = JointReading(now, dict(self.positions), True, dict(self.velocities))
        return self.reading

    def inject_fault(self, fault):
        if fault not in ('', 'reject', 'feedback_loss'):
            raise ValueError('UNKNOWN_FAULT')
        self.fault = fault
