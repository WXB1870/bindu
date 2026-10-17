"""ROS binding for the Snowbot position gripper loop, sharing its arm watchdog instance."""
import json
import hashlib
from pathlib import Path
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.qos import qos_profile_sensor_data
from bindu_hardware.snowbot_gripper import GripperController, SIDES, GROUPS
from bindu_contracts.teleoperation import VRFrame
from bindu_interfaces.msg import GripperState, VRInput
from bindu_interfaces.srv import GripperControl, Lease
from .common import seconds, stamp


class SnowbotGrippers:
    def __init__(self, devices):
        self.devices, self.node = devices, devices.node
        node = self.node
        node.declare_parameter('gripper_config', '', descriptor=ParameterDescriptor(read_only=True))
        self.core = None
        path = node.get_parameter('gripper_config').value
        if not path:
            return
        self.config_hash = hashlib.sha256(Path(path).read_bytes()).hexdigest()
        cfg = json.loads(Path(path).read_text())
        self.topic = node.get_parameter('vendor_namespace').value.rstrip('/')+'/bindu/gripper_command'
        self.publisher = (node.create_publisher(devices.array_type, self.topic, 1)
            if node.robot_io.command_enabled and cfg.get('calibrated') else None)
        self.core = GripperController(node.profile, cfg, self.send, self.health, node.now, self.arms_idle,
                                      enabled=node.robot_io.command_enabled)
        self.state_pub = node.create_publisher(GripperState, 'gripper/state', 10)
        node.create_service(Lease, 'gripper/lease', self.lease)
        node.create_service(GripperControl, 'gripper/control', self.control)
        node.create_subscription(VRInput, 'teleop/vr/input', self.input, qos_profile_sensor_data)

    def arms_idle(self):
        engine = getattr(self.node, 'engine', None)
        return bool(engine and not engine.lease_id and not engine.active and not engine.stopping and not engine.stop_fault)

    def health(self):
        self.devices.io.require_feedback()
        self.devices.check_health(set(self.core.groups))
        if self.publisher is None or self.node.count_publishers(self.topic) != 1:
            raise RuntimeError('GRIPPER_COMMAND_PUBLISHER_CONFLICT')
        readers = self.node.get_subscriptions_info_by_topic(self.topic)
        if not any(info.node_namespace.rstrip('/')+'/'+info.node_name == self.devices.receivers['head'] for info in readers):
            raise RuntimeError('GRIPPER_RECEIVER_UNAVAILABLE')
        return self.devices.health.messages['servo']

    def send(self, side, position, hold, sequence):
        self.health()
        cfg = self.core.cfg['grippers'][side]
        session = self.devices.session
        packet = [self.node.now(), float(sequence), float(session >> 32), float(session & 0xffffffff),
                  float(SIDES.index(side)), 0. if hold else 1., float(position), float(cfg['kp']), float(cfg['kd']),
                  self.core.cfg['command_ttl'], float(cfg['max_speed'])]
        self.publisher.publish(self.devices.array_type(data=packet))

    def observe(self, msg, q, v, when):
        if self.core is None:
            return
        try:
            current = {side: float(getattr(msg, prefix+'_iq')[7]) for side, prefix in (('left','l'),('right','r'))}
            self.core.observe(q, v, current, when)
        except (ValueError, IndexError) as exc:
            self.core.invalid = str(exc)

    def lease(self, req, reply):
        try:
            if self.node.closing:
                raise RuntimeError('SHUTDOWN')
            self.core.lease(req.operation, req.owner, req.resources, req.lease_id, req.epoch)
            reply.ok, reply.code = True, 'OK'
        except RuntimeError as exc:
            reply.ok, reply.code = False, str(exc)
        reply.lease_id, reply.epoch, reply.profile_hash = self.core.lease_id, self.core.epoch, self.node.profile.digest
        return reply

    def control(self, req, reply):
        try:
            if self.node.closing:
                raise RuntimeError('SHUTDOWN')
            self.core.command(req.operation, req.side, req.lease_id, req.epoch, req.sequence,
                seconds(req.stamp), req.valid_for, position=req.position,
                load_state=req.load_state, external_support=req.external_support)
            reply.accepted, reply.code = True, 'REQUEST_ACCEPTED'
        except (RuntimeError, ValueError) as exc:
            reply.accepted, reply.code = False, str(exc)
        return reply

    def input(self, msg):
        if self.node.matches_mode(msg):
            frame = VRFrame(msg.source_id, msg.seq, seconds(msg.stamp), msg.side, tuple(msg.pose), msg.grip, msg.trigger,
                msg.init, msg.stop, msg.valid, msg.clutch_seq, msg.mapping, msg.buttons, tuple(int(value) for value in msg.button_edges), msg.virtual_grip)
            self.core.input(frame)

    def tick(self):
        self.core.tick()
        for side in SIDES:
            data = self.core.snapshot(side)
            when = data.pop('feedback_stamp')
            self.state_pub.publish(GripperState(stamp=stamp(self.node.now()), feedback_stamp=stamp(when),
                runtime_mode='real', instance_id=self.node.instance_id, profile_hash=self.node.profile.digest,
                config_hash=self.config_hash, **data))

    def stop(self):
        if self.core:
            self.core.stop('GRIPPER_SHUTDOWN', revoke=True)

    @property
    def stopping(self):
        return bool(self.core and any(c['state'] == 'STOPPING' for c in self.core.channels.values()))
