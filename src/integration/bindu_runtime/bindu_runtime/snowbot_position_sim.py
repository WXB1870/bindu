"""Simulation devices exposing the same position/gripper services as Snowbot.

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
    node.physics_transport = (devices['left_arm'].transport if node.device_backend=='external_simulation' else None)
    from bindu_hardware.drivers.simulated_joints import SimJointDriver, SynchronizedJointProgress
    coupled=[devices[g] for g in ('waist','left_arm','right_arm')]
    if all(isinstance(driver,SimJointDriver) for driver in coupled):
        SynchronizedJointProgress(coupled,{n:profile.speed(n) for driver in coupled for n in driver.positions})
        node.position_progress_synchronized=True
    node.declare_parameter('command_enabled', False, descriptor=ParameterDescriptor(read_only=True))
    node.commands_enabled = node.get_parameter('command_enabled').value
    io_type = SnowbotIO
    options = {}
    if node.physics_transport is not None:
        from .simulation_devices import PhysicsPositionIO
        io_type, options = PhysicsPositionIO, dict(physics=node.physics_transport)
    io = io_type(profile, lambda groups, q: transport.write_joints(q), node.now,
                 command_enabled=node.commands_enabled, simulated=True, **options)
    def read(now, dt):
        feedback=transport.read_feedback(now, dt)
        if feedback.stamp > io.last_source_stamp:
            try:
                io.observe(feedback.positions,feedback.joint_velocities,feedback.stamp,now)
            except ValueError as exc:
                raise RuntimeError(str(exc)) from exc
        if node.physics_transport is not None and not io.last_source_stamp:
            return io.last_feedback
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
            if node.physics_transport is None:
                node.devices[GROUPS[side]].max_speed = cfg['grippers'][side]['max_speed']
        self.bind_services()

    def health(self):
        feedback = self.node.engine.feedback
        if not 0 <= self.node.now()-feedback.stamp <= self.core.cfg['feedback_max_age']:
            raise RuntimeError('GRIPPER_FEEDBACK_STALE')
        if self.node.physics_transport is not None:
            health=self.node.physics_transport.health
            if not health.get('ready') or not 0 <= self.node.now()-health.get('stamp',0) <= .2:
                raise RuntimeError('PHYSICS_GRIPPER_HEALTH_STALE')
            for side,pending in list(self.pending.items()):
                sequence,target,hold,wire_sequence=pending
                group=GROUPS[side];i=SIDES.index(side)
                if health.get('applied_sequences',{}).get(group,0) >= wire_sequence:
                    self.acks[i]=sequence
                    actual_hold=health.get('hold_positions',{}).get(JOINTS[side])
                    if hold and actual_hold is not None:
                        self.holds[i]=actual_hold;self.hold_mask|=1<<i
                    else:self.hold_mask&=~(1<<i)
                    del self.pending[side]
        return dict(sample=self.sample, gripper_control_enabled=True,
            gripper_sequence=list(self.acks), gripper_hold_position=list(self.holds),
            gripper_hold_mask=self.hold_mask, gripper_latched_mask=0,
            operation_enabled_mask=sum(MASKS.values()))

    def send(self, side, position, hold, sequence):
        if self.node.physics_transport is not None:
            wire_sequence=self.node.physics_transport.send(GROUPS[side], 'hold' if hold else 'set',
                positions=None if hold else {JOINTS[side]:position})
            self.pending[side]=(sequence,position,hold,wire_sequence)
            return
        self.node.devices[GROUPS[side]].write_positions({JOINTS[side]: position})
        self.pending[side] = (sequence, position, hold)

    def tick(self):
        feedback = self.node.engine.feedback
        if feedback.stamp > self.core.feedback_stamp:
            self.sample += 1
            for side, (seq, q, hold) in (self.pending.items() if self.node.physics_transport is None else ()):
                i = SIDES.index(side)
                self.acks[i], self.holds[i] = seq, q
                self.hold_mask = (self.hold_mask | (1 << i)) if hold else (self.hold_mask & ~(1 << i))
            if self.node.physics_transport is None:self.pending.clear()
            try:
                self.core.observe(feedback.positions, feedback.joint_velocities,
                                  dict.fromkeys(SIDES, 0.), feedback.stamp)
            except ValueError as exc:
                self.core.invalid = str(exc)
        super().tick()


class PositionSimulation(ExecutionNode):
    def __init__(self):
        super().__init__(device_factory=position_devices, executor_factory=DelegatedPositionExecutor)
        if self.device_backend not in ('kinematic','external_simulation'):
            raise ValueError('POSITION_SIM_REQUIRES_SIMULATION_DEVICE')
        synchronization=json.dumps(dict(
            shared_vector_progress=getattr(self,'position_progress_synchronized',False),
            groups=['waist','left_arm','right_arm'],clock='execution_feedback'))
        self.get_logger().info('POSITION_SYNCHRONIZATION '+synchronization)
        self.event('POSITION_SYNCHRONIZATION',synchronization)
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
