"""Kinematic devices exposing the same position/gripper services as Snowbot.

The gripper state machine is shared with the hardware adapter. Only device IO
and acknowledgement generation live here; no controller/vendor topics exist.
"""
import hashlib
import json
from pathlib import Path
from rcl_interfaces.msg import ParameterDescriptor
from bindu_execution.delegated_position import DelegatedPositionExecutor
from bindu_hardware.snowbot import SnowbotIO
from bindu_hardware.snowbot_gripper import GripperController, SIDES, JOINTS, GROUPS, MASKS
from .execution_node import ExecutionNode, run_execution
from .snowbot_gripper import SnowbotGrippers
from .device_assembly import assemble_devices


def position_devices(node, profile):
    """Reuse the real IO contract; replace only transport and measured samples."""
    transport, devices = assemble_devices(node, profile)
    node.declare_parameter('command_enabled', False, descriptor=ParameterDescriptor(read_only=True))
    node.commands_enabled = node.get_parameter('command_enabled').value
    io = SnowbotIO(profile, lambda groups, q: transport.write_joints(q), node.now,
                   command_enabled=node.commands_enabled, simulated=True)
    def read(now, dt):
        feedback=transport.read_feedback(now, dt)
        if feedback.stamp > io.last_source_stamp:
            io.observe(feedback.positions,feedback.joint_velocities,feedback.stamp,now)
        io.require_feedback()
        return io.last_feedback
    io.read_feedback=read
    return io, devices


class SimulatedGrippers(SnowbotGrippers):
    def __init__(self, node):
        self.node = node
        node.declare_parameter('gripper_config', '', descriptor=ParameterDescriptor(read_only=True))
        path = Path(node.get_parameter('gripper_config').value)
        cfg = json.loads(path.read_text())
        if cfg.get('simulation_only') is not True:
            raise ValueError('SIM_GRIPPER_CONFIG_REQUIRED')
        self.config_hash = hashlib.sha256(path.read_bytes()).hexdigest()
        self.sample = 0
        self.acks, self.holds, self.hold_mask = [0, 0], [0., 0.], 0
        self.pending = {}
        self.core = GripperController(node.profile, cfg, self.send, self.health,
                                      node.now, self.arms_idle, enabled=node.commands_enabled)
        for side in SIDES:
            node.devices[GROUPS[side]].max_speed = cfg['grippers'][side]['max_speed']
        self.bind_services()

    def health(self):
        feedback = self.node.engine.feedback
        if not 0 <= self.node.now()-feedback.stamp <= self.core.cfg['feedback_max_age']:
            raise RuntimeError('GRIPPER_FEEDBACK_STALE')
        return dict(sample=self.sample, gripper_control_enabled=True,
            gripper_sequence=list(self.acks), gripper_hold_position=list(self.holds),
            gripper_hold_mask=self.hold_mask, gripper_latched_mask=0,
            operation_enabled_mask=sum(MASKS.values()))

    def send(self, side, position, hold, sequence):
        self.node.devices[GROUPS[side]].write_positions({JOINTS[side]: position})
        self.pending[side] = (sequence, position, hold)

    def tick(self):
        feedback = self.node.engine.feedback
        if feedback.stamp > self.core.feedback_stamp:
            self.sample += 1
            for side, (seq, q, hold) in self.pending.items():
                i = SIDES.index(side)
                self.acks[i], self.holds[i] = seq, q
                self.hold_mask = (self.hold_mask | (1 << i)) if hold else (self.hold_mask & ~(1 << i))
            self.pending.clear()
            try:
                self.core.observe(feedback.positions, feedback.joint_velocities,
                                  dict.fromkeys(SIDES, 0.), feedback.stamp)
            except ValueError as exc:
                self.core.invalid = str(exc)
        super().tick()


class PositionSimulation(ExecutionNode):
    def __init__(self):
        super().__init__(device_factory=position_devices, executor_factory=DelegatedPositionExecutor)
        if self.device_backend != 'kinematic':
            raise ValueError('POSITION_SIM_REQUIRES_KINEMATIC')
        self.grippers = SimulatedGrippers(self)

    def lease(self, request, reply):
        if request.operation == 'acquire':
            reason = ('COMMANDS_DISABLED' if not self.commands_enabled else
                      'GRIPPERS_BUSY_OR_FAULT' if self.grippers.core.busy else '')
            if reason:
                reply.ok, reply.code, reply.profile_hash = False, reason, self.profile.digest
                return reply
        return super().lease(request, reply)

    def tick(self):
        super().tick()
        if getattr(self, 'grippers', None):
            self.grippers.tick()

    def begin_shutdown(self):
        self.grippers.stop()
        self.commands_enabled = False
        super().begin_shutdown()

    def shutdown_pending(self):
        return super().shutdown_pending() or self.grippers.stopping

    def shutdown_failed(self):
        return super().shutdown_failed() or bool(self.grippers.core.fault)

    def abort_shutdown(self):
        if self.grippers.stopping:
            self.grippers.core.fail('SHUTDOWN_TIMEOUT')
        super().abort_shutdown()


def main():
    run_execution(PositionSimulation)
