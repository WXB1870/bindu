import math
from bindu_contracts.devices import JointReading


class SynchronizedJointProgress:
    """One bounded vector step across groups, without per-axis time warping.

    This kinematic fixture follows the checked joint segment with one scalar
    progress value. It does not claim a hardware clock or force-level guarantee.
    """
    def __init__(self, drivers, speeds):
        self.drivers=tuple(drivers)
        names=[n for driver in self.drivers for n in driver.positions]
        if len(set(names))!=len(names) or set(names)!=set(speeds):
            raise ValueError('SYNCHRONIZED_JOINT_LAYOUT')
        if any(not math.isfinite(v) or v<=0 for v in speeds.values()):
            raise ValueError('SYNCHRONIZED_JOINT_SPEED')
        self.speeds=dict(speeds);self.last_time=None;self.progress=0.
        for driver in self.drivers:driver.synchronized_progress=self

    def advance(self, now, dt):
        if self.last_time==now:return
        self.last_time=now;dt=max(0.,min(dt,.05))
        steps=[(driver,n,target-driver.positions[n]) for driver in self.drivers
               for n,target in driver.target.items()]
        self.progress=min((min(1.,self.speeds[n]*dt/abs(delta)) for _,n,delta in steps if delta),default=1.)
        for driver,n,delta in steps:
            change=self.progress*delta;driver.positions[n]+=change
            if dt>0:driver.velocities[n]=change/dt


class SimJointDriver:
    """Bounded-speed kinematic fixture for one named joint group."""
    def __init__(self, names, max_speed, initial_positions=None):
        self.positions = {name: float((initial_positions or {}).get(name,0.)) for name in names}
        self.target = dict(self.positions)
        self.max_speed = max_speed
        self.fault = ''
        self.velocities = dict.fromkeys(self.positions, 0.)
        self.reading = JointReading(0., dict(self.positions), True, dict(self.velocities))
        self.synchronized_progress=None

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
        if self.synchronized_progress is not None:
            self.synchronized_progress.advance(now,dt)
        else:
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
