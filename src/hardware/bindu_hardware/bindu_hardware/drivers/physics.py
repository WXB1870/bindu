"""ROS-independent command gate for a local external physics simulator."""
import math


class PhysicsCommandGate:
    def __init__(self, profile, simulator_id, timeout=.25):
        self.profile, self.simulator_id, self.timeout = profile, simulator_id, timeout
        self.sender = ''
        self.sequence = 0
        self.deadlines = {}

    def accept(self, command, now):
        stamp = command.stamp.sec + command.stamp.nanosec / 1e9
        if command.simulator_id != self.simulator_id or command.profile_hash != self.profile.digest:
            raise ValueError('SIM_ID_OR_PROFILE_MISMATCH')
        if not command.sender_id or (self.sender and command.sender_id != self.sender):
            raise ValueError('SIM_EXECUTOR_CHANGED')
        if command.sequence <= self.sequence:
            raise ValueError('SIM_REPLAY')
        if not all(math.isfinite(v) for v in (stamp, command.valid_for)) or not 0 < command.valid_for <= self.timeout:
            raise ValueError('SIM_INVALID_TTL')
        if stamp > now + .05 or now >= stamp + command.valid_for:
            raise ValueError('SIM_STALE_COMMAND')
        group = command.resource_group
        if group not in set(self.profile.groups) | {'base'} or command.operation not in ('set', 'hold'):
            raise ValueError('SIM_INVALID_OPERATION')
        twist = command.velocity
        values = [twist.linear.x, twist.linear.y, twist.linear.z,
                  twist.angular.x, twist.angular.y, twist.angular.z, *command.positions]
        if not all(math.isfinite(v) for v in values):
            raise ValueError('SIM_NONFINITE_COMMAND')
        if any(v != 0 for v in (twist.linear.y, twist.linear.z, twist.angular.x, twist.angular.y)):
            raise ValueError('SIM_UNSUPPORTED_AXIS')
        if command.operation == 'hold' and any(v != 0 for v in values):
            raise ValueError('SIM_HOLD_HAS_TARGET')
        if group == 'base':
            if command.joint_names or command.positions or abs(twist.linear.x) > self.profile.base_speed('linear') or abs(twist.angular.z) > self.profile.base_speed('angular'):
                raise ValueError('SIM_BASE_LIMIT')
        elif command.operation == 'set':
            if list(command.joint_names) != self.profile.groups[group] or len(command.positions) != len(command.joint_names):
                raise ValueError('SIM_JOINT_LAYOUT')
            if any(not self.profile.limits[n][0] <= q <= self.profile.limits[n][1]
                   for n, q in zip(command.joint_names, command.positions)):
                raise ValueError('SIM_JOINT_LIMIT')
        elif command.joint_names or command.positions:
            raise ValueError('SIM_HOLD_HAS_TARGET')
        if group != 'base' and (twist.linear.x != 0 or twist.angular.z != 0):
            raise ValueError('SIM_CROSS_RESOURCE_COMMAND')
        self.sender, self.sequence = command.sender_id, command.sequence
        self.deadlines[group] = stamp + command.valid_for
        return group

    def expired(self, now):
        expired = [g for g, deadline in self.deadlines.items() if now >= deadline]
        for group in expired:
            del self.deadlines[group]
        return expired


class PhysicsPositionState:
    """Post-step target acceptance and separate measured stable-stop ACKs."""
    def __init__(self, profile, simulator_id, initial, timeout=.25, settle_time=.1):
        self.gate=PhysicsCommandGate(profile,simulator_id,timeout)
        self.profile=profile
        if not math.isfinite(settle_time) or not 0 < settle_time < profile.stop_timeout:
            raise ValueError('PHYSICS_HOLD_SETTLE_TIME')
        self.settle_time=settle_time
        self.targets=dict(initial)
        self.reference=dict(initial)
        self.speeds={n:profile.speed(n) for n in initial}
        self.coupled_names={n for g in ('waist','left_arm','right_arm')
                            for n in profile.groups.get(g,[])}
        self.progress=1.
        self.holding=set()
        self.hold_positions={}
        self.pending={}
        self.applied_sequences={}
        self.hold_requests={}
        self.hold_candidates={}
        self.hold_acknowledgments={}

    def hold(self, group, measured):
        if group not in self.holding:
            for n in self.profile.groups[group]:
                self.reference[n]=self.targets[n]=self.hold_positions[n]=measured[n]
            self.holding.add(group)

    def accept(self, command, now, measured):
        if command.resource_group not in self.profile.groups:
            raise ValueError('PHYSICS_POSITION_RESOURCE')
        group=self.gate.accept(command,now)
        self.hold_candidates.pop(group,None)
        self.hold_acknowledgments.pop(group,None)
        if command.operation=='hold':
            self.hold(group,measured)
            self.hold_requests[group]=command.sequence
        else:
            self.hold_requests.pop(group,None)
            self.holding.discard(group)
            for n,q in zip(command.joint_names,command.positions):
                self.targets[n]=q;self.hold_positions.pop(n,None)
        self.pending[group]=command.sequence

    def expire(self, now, measured):
        groups=self.gate.expired(now)
        for group in groups:self.hold(group,measured)
        return groups

    def stepped(self):
        self.applied_sequences.update(self.pending)
        self.pending.clear()

    def observe_holds(self, measured, velocities, stamp):
        """ACK an explicit hold once its measured physical stop is stable.

        Force drives can rest with a static position bias. The drive target
        remains fixed; the separate stop ACK describes the actual braking point.
        Only measured feedback uses the configured position tolerance at limits;
        all new set targets still pass the strict nominal command gate.
        Neither timeout fallback holds nor a still-moving sample can ACK a stop.
        """
        position_limit=self.profile.feedback_tolerances['joint_position']
        velocity_limit=self.profile.feedback_tolerances['joint_velocity']
        for group,sequence in self.hold_requests.items():
            if group in self.hold_acknowledgments or self.applied_sequences.get(group)!=sequence:
                continue
            names=self.profile.groups[group]
            values=[stamp,*[measured.get(n,float('nan')) for n in names],
                    *[velocities.get(n,float('nan')) for n in names]]
            if (not all(math.isfinite(v) for v in values) or
                    any(abs(velocities[n])>velocity_limit or
                        not self.profile.limits[n][0]-position_limit<=measured[n]<=self.profile.limits[n][1]+position_limit for n in names)):
                self.hold_candidates.pop(group,None)
                continue
            positions={n:measured[n] for n in names}
            candidate=self.hold_candidates.get(group)
            if (candidate is None or not 0 < stamp-candidate[1] <= .2 or
                    any(abs(positions[n]-candidate[2][n])>=position_limit for n in names)):
                self.hold_candidates[group]=(stamp,stamp,positions)
            elif stamp-candidate[0]>=self.settle_time:
                self.hold_acknowledgments[group]=dict(sequence=sequence,stamp=stamp,positions=positions)
            else:
                self.hold_candidates[group]=(candidate[0],stamp,candidate[2])

    def reference_step(self, dt):
        """Device-owned coupled position/velocity reference, never feedback."""
        from .simulated_joints import synchronized_position_step
        if not math.isfinite(dt) or not 0<dt<=.1:
            raise ValueError('PHYSICS_REFERENCE_TIMESTEP')
        reference,velocity,self.progress=synchronized_position_step(
            self.reference,{n:self.targets[n] for n in self.coupled_names},self.speeds,dt)
        # Head and grippers have independent device control. Their speed must
        # neither throttle the coupled waist/arms nor depend on arm motion.
        for n in self.targets.keys()-self.coupled_names:
            q,v,_=synchronized_position_step(reference,{n:self.targets[n]},self.speeds,dt)
            reference[n],velocity[n]=q[n],v[n]
        self.reference=reference
        return dict(self.reference),velocity
