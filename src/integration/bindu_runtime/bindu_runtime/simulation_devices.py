"""Nonblocking ROS device adapters for an external physics process."""
import math
from bindu_contracts.devices import JointReading, BaseReading
from bindu_hardware.snowbot import SnowbotIO
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
        from std_msgs.msg import String
        self.health = {}
        node.create_subscription(String,'simulation/health',self.receive_health,1)
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
                or not 0 <= self.node.now()-seconds(msg.stamp) <= .2):
            return
        if self.latest and (msg.sequence <= self.latest.sequence or seconds(msg.stamp) <= seconds(self.latest.stamp)):
            return
        q = msg.base_pose.orientation
        if abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1) > .001:
            return
        self.simulator_id, self.latest = msg.simulator_id, msg

    def receive_health(self, msg):
        import json
        try:
            value=json.loads(msg.data)
            if (value.get('profile_hash')==self.profile.digest and value.get('run_id')==self.node.run_id and
                    value.get('runtime_mode')=='simulation' and value.get('simulator_id')==self.simulator_id and
                    0 <= self.node.now()-value['stamp'] <= .2):
                self.health=value
        except (ValueError,KeyError,TypeError):pass

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
        return self.sequence

    def reading(self):
        if self.failed:
            raise RuntimeError('SIM_INSTANCE_CHANGED')
        return self.latest


class PhysicsPositionIO(SnowbotIO):
    """Confirm an explicit device hold at its acknowledged physical position."""
    def __init__(self, *args, physics, **kwargs):
        super().__init__(*args, **kwargs)
        self.physics = physics
        self.hold_sequences = {}
        self.hold_ack = None

    def capture(self, groups):
        super().capture(groups)
        self.hold_sequences.clear()
        self.hold_ack = None

    def stop(self):
        if not self.targets:
            return super().stop()  # Preserve an outstanding request and deadline.
        self.reference.clear()
        self.stop_requested_at = self.now()
        self.stop_targets = {}
        self.hold_sequences.clear()
        self.hold_ack = None
        groups = self._groups(self.targets)
        try:
            self.require_feedback()
        except RuntimeError:
            self.stop_failures = tuple(sorted(groups))
            self.targets.clear()
            return
        failures = set(self.stop_failures)
        for group in sorted(groups):
            # Reserve the entire footprint even if a send partly fails.
            self.stop_targets.update({n:self.last_feedback.positions[n]
                                      for n in self.profile.groups[group]})
            try:
                self.hold_sequences[group] = self.physics.send(group, operation='hold')
            except RuntimeError:
                failures.add(group)
        self.stop_failures = tuple(sorted(failures))
        self.targets.clear()

    def confirmed_stop_targets(self):
        """Wait for all post-step ACKs and matching fresh joint feedback.

        Acknowledge each request once; never chase drifting measured positions
        or accept the timeout fallback hold as evidence of this explicit hold.
        """
        h = self.physics.health
        at = h.get('stamp')
        if (self.stop_failures or not self.hold_sequences or self.physics.failed or
                not h.get('ready') or h.get('simulator_id') != self.physics.simulator_id or
                type(at) not in (int,float) or not math.isfinite(at) or
                at <= self.stop_requested_at or not 0 <= self.now()-at <= .2):
            return None
        sequences, holds = h.get('applied_sequences'), h.get('hold_positions')
        if not isinstance(sequences,dict) or not isinstance(holds,dict):
            return None
        if any(sequences.get(g) != seq for g,seq in self.hold_sequences.items()):
            return None
        acknowledgments=h.get('hold_acknowledgments')
        if not isinstance(acknowledgments,dict):
            return None
        targets={}
        ack_stamps=[]
        for group,sequence in self.hold_sequences.items():
            ack=acknowledgments.get(group)
            if not isinstance(ack,dict):return None
            positions,ack_at=ack.get('positions'),ack.get('stamp')
            if (ack.get('sequence')!=sequence or not isinstance(positions,dict) or
                    set(positions)!=set(self.profile.groups[group]) or
                    type(ack_at) not in (int,float) or not math.isfinite(ack_at) or
                    not self.stop_requested_at < ack_at <= at):
                return None
            targets.update(positions)
            ack_stamps.append(ack_at)
        if set(targets)!=set(self.stop_targets):return None
        if any(type(q) not in (float,int) or not math.isfinite(q) or
               not self.profile.limits[n][0] <= q <= self.profile.limits[n][1]
               for n,q in targets.items()):
            return None
        if self.hold_ack is None:
            self.hold_ack = (max(ack_stamps), targets)
        ack_stamp, acknowledged = self.hold_ack
        if targets != acknowledged or self.last_source_stamp+1e-6 < ack_stamp:
            return None
        return dict(acknowledged)


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
