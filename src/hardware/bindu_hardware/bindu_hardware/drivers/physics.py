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
