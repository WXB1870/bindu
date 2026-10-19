import math
from bindu_contracts.devices import JointReading


def synchronized_position_step(positions, targets, speeds, dt):
    """A single speed-bounded scalar step; positions are a reference or fixture."""
    deltas={n:q-positions[n] for n,q in targets.items()}
    progress=min((min(1.,speeds[n]*dt/abs(delta)) for n,delta in deltas.items() if delta),default=1.)
    result=dict(positions)
    velocities=dict.fromkeys(positions,0.)
    for n,delta in deltas.items():
        change=progress*delta;result[n]+=change
        if dt>0:velocities[n]=change/dt
    return result,velocities,progress


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
        positions={n:q for d in self.drivers for n,q in d.positions.items()}
        targets={n:q for d in self.drivers for n,q in d.target.items()}
        positions,velocities,self.progress=synchronized_position_step(positions,targets,self.speeds,dt)
        for driver in self.drivers:
            for n in driver.target:
                driver.positions[n]=positions[n]
                if dt>0:driver.velocities[n]=velocities[n]


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
