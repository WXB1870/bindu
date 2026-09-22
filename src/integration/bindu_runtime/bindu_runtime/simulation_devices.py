"""Nonblocking ROS device adapters for an external physics process."""
import math
from bindu_contracts.devices import JointReading, BaseReading
from bindu_interfaces.msg import SimulationCommand, SimulationFeedback
from rclpy.qos import QoSProfile, ReliabilityPolicy
from .common import seconds, stamp


class SimulationDevices:
    def __init__(self, node, profile):
        self.node, self.profile = node, profile
        self.latest = None
        self.simulator_id = ''
        self.failed = False
        self.sequence = 0
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.publisher = node.create_publisher(SimulationCommand, 'simulation/command', 8)
        self.subscription = node.create_subscription(SimulationFeedback, 'simulation/feedback', self.receive, qos)
        self.names = [n for names in profile.groups.values() for n in names]

    def receive(self, msg):
        if self.simulator_id and self.simulator_id != msg.simulator_id:
            self.failed = True  # A simulator restart requires restarting the executor.
            return
        values = [seconds(msg.stamp), *msg.joints.position, *msg.joints.velocity,
                  msg.base_pose.position.x, msg.base_pose.position.y,
                  msg.base_pose.orientation.x, msg.base_pose.orientation.y,
                  msg.base_pose.orientation.z, msg.base_pose.orientation.w,
                  msg.base_velocity.linear.x, msg.base_velocity.angular.z]
        if (not msg.ready or not msg.simulator_id or msg.profile_hash != self.profile.digest
                or list(msg.joints.name) != self.names or len(msg.joints.position) != len(self.names)
                or len(msg.joints.velocity) != len(self.names) or not all(math.isfinite(v) for v in values)
                or seconds(msg.stamp) > self.node.now() + .05):
            return
        if self.latest and (msg.sequence <= self.latest.sequence or seconds(msg.stamp) <= seconds(self.latest.stamp)):
            return
        q = msg.base_pose.orientation
        if abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1) > .001:
            return
        self.simulator_id, self.latest = msg.simulator_id, msg

    def send(self, group, operation='set', positions=None, velocity=(0., 0.)):
        if self.failed or not self.latest or self.node.now() - seconds(self.latest.stamp) > .2:
            raise RuntimeError('SIM_FEEDBACK_UNAVAILABLE')
        self.sequence += 1
        msg = SimulationCommand(stamp=stamp(self.node.now()), simulator_id=self.simulator_id,
            sender_id=self.node.instance_id, sequence=self.sequence, profile_hash=self.profile.digest,
            valid_for=.25, operation=operation, resource_group=group)
        if positions is not None:
            msg.joint_names = list(self.profile.groups[group])
            msg.positions = [positions[n] for n in msg.joint_names]
        msg.velocity.linear.x, msg.velocity.angular.z = velocity
        self.publisher.publish(msg)

    def reading(self):
        if self.failed:
            raise RuntimeError('SIM_INSTANCE_CHANGED')
        return self.latest


class PhysicsJoints:
    def __init__(self, transport, group):
        self.transport, self.group = transport, group

    def write_positions(self, target):
        self.transport.send(self.group, positions=target)

    def request_stop(self):
        self.transport.send(self.group, operation='hold')

    def read(self, now, dt):
        msg = self.transport.reading()
        names = self.transport.profile.groups[self.group]
        positions = dict(zip(msg.joints.name, msg.joints.position)) if msg else dict.fromkeys(names, 0.)
        velocities = dict(zip(msg.joints.name, msg.joints.velocity)) if msg else {}
        return JointReading(seconds(msg.stamp) if msg else 0., {n: positions[n] for n in names}, True,
                            {n: velocities[n] for n in names} if msg else {})


class PhysicsBase:
    def __init__(self, transport):
        self.transport = transport

    def write_velocity(self, velocity):
        self.transport.send('base', velocity=velocity)

    def request_stop(self):
        self.transport.send('base', operation='hold')

    def read(self, now, dt):
        msg = self.transport.reading()
        if msg is None:
            return BaseReading(0., 0., 0., (0., 0.), True)
        q = msg.base_pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        return BaseReading(seconds(msg.stamp), msg.base_pose.position.x, yaw,
            (msg.base_velocity.linear.x, msg.base_velocity.angular.z), True, y=msg.base_pose.position.y)
