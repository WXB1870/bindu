import signal
import time
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from rcl_interfaces.msg import ParameterDescriptor
from bindu_contracts.contracts import Motion
from .device_assembly import assemble_devices, inject_simulated_fault
from bindu_execution.executor import Executor, Rejected
from bindu_interfaces.msg import ExecutionState, MotionCommand
from bindu_interfaces.srv import Lease, SubmitMotion, ControlExecution, InjectFault
from .common import RuntimeNode, seconds, stamp, EVENT_QOS
from bindu_contracts.runtime_mode import command_mode


class ExecutionNode(RuntimeNode):
    def __init__(self, *, allow_hardware=False, device_factory=assemble_devices, executor_factory=Executor):
        super().__init__('execution', allow_hardware=allow_hardware)
        if self.runtime_mode == 'real':
            if self.get_parameter('use_sim_time').value:
                raise ValueError('REAL_EXECUTION_REQUIRES_SYSTEM_TIME')
            self.set_descriptor('use_sim_time', ParameterDescriptor(read_only=True,
                type=self.get_parameter('use_sim_time').type_.value,
                description='Real execution requires the system clock'))
        self.closing = False
        self.robot_io, self.devices = device_factory(self, self.profile)
        self.engine = executor_factory(self.profile, self.robot_io, runtime_mode=self.runtime_mode)
        self.engine.tick(self.now())
        # Default mutually exclusive group serializes every access to the engine.
        self.create_service(Lease, 'execution/lease', self.lease)
        self.create_service(SubmitMotion, 'execution/submit', self.submit)
        self.create_service(ControlExecution, 'execution/control', self.control)
        self.create_service(InjectFault, 'execution/sim_fault', self.fault)
        self.create_subscription(MotionCommand, 'execution/targets', self.stream, 8)
        self.state_pub = self.create_publisher(ExecutionState, 'execution/state', 10)
        self.accepted_pub = self.create_publisher(MotionCommand, 'execution/accepted', EVENT_QOS)
        self.timer = self.create_timer(self.profile.control_period, self.tick)
        self.last_event = None

    def lease(self, request, reply):
        try:
            if self.closing:
                raise Rejected('SHUTDOWN')
            if request.operation == 'acquire':
                self.engine.acquire(request.owner, request.resources, 2., self.now())
            elif request.operation == 'renew':
                self.engine.renew(request.lease_id, request.epoch, self.now())
            elif request.operation == 'release':
                self.engine.check_lease(request.lease_id, request.epoch, self.now())
                self.engine.halt(self.now(), revoke=True)
            else:
                raise Rejected('INVALID_OPERATION')
            reply.ok, reply.code = True, 'OK'
            if request.operation == 'release' and not self.engine.stopping and self.robot_io.stop_failures:
                reply.ok, reply.code = False, 'STOP_REQUEST_FAILED'
        except Rejected as exc:
            reply.ok, reply.code = False, str(exc)
        reply.lease_id, reply.epoch = self.engine.lease_id, self.engine.epoch
        reply.profile_hash = self.profile.digest
        return reply

    def accept(self, msg):
        if self.closing:
            raise Rejected('SHUTDOWN')
        selected_mode = command_mode(msg.runtime_mode, self.runtime_mode)
        if any(v != 0. for v in (msg.velocity.linear.y, msg.velocity.linear.z,
                                msg.velocity.angular.x, msg.velocity.angular.y)):
            raise Rejected('UNSUPPORTED_VELOCITY_AXIS')
        if any(p.effort for p in msg.points):
            raise Rejected('UNSUPPORTED_TRAJECTORY_EFFORT')
        m = Motion(msg.command_id, msg.lease_id, msg.epoch, msg.profile_hash,
                   msg.task_id, msg.observation_id, msg.mode, msg.resource_group,
                   seconds(msg.stamp), msg.valid_for, tuple(msg.joint_names),
                   tuple(msg.positions), tuple(seconds(p.time_from_start) for p in msg.points),
                   tuple(tuple(p.positions) for p in msg.points),
                   (msg.velocity.linear.x, msg.velocity.angular.z), msg.duration, msg.schema_version,
                   tuple(tuple(p.velocities) for p in msg.points) if any(p.velocities for p in msg.points) else (),
                   tuple(tuple(p.accelerations) for p in msg.points) if any(p.accelerations for p in msg.points) else (),
                   msg.expected_revision, tuple(msg.resource_groups), selected_mode)
        self.engine.submit(m, self.now())
        msg.runtime_mode = selected_mode
        self.accepted_pub.publish(msg)
        return max(m.stamp, self.engine.last_tick)

    def submit(self, request, reply):
        try:
            switched = self.accept(request.command)
            reply.switch_stamp = stamp(switched)
            reply.accepted, reply.code = True, 'OK'
        except (Rejected, ValueError) as exc:
            reply.accepted, reply.code = False, str(exc)
            m = request.command
            self.event('REJECTED', reply.code, m.task_id, m.command_id, m.observation_id)
        reply.revision = self.engine.revision
        return reply

    def stream(self, msg):
        try:
            if msg.mode != 'joint_target':
                raise Rejected('STREAM_REQUIRES_JOINT_TARGET')
            self.accept(msg)
        except (Rejected, ValueError) as exc:
            self.event('REJECTED', str(exc), msg.task_id, msg.command_id, msg.observation_id)

    def control(self, request, reply):
        try:
            if self.closing:
                raise Rejected('SHUTDOWN')
            self.engine.check_lease(request.lease_id, request.epoch, self.now())
            if request.operation not in ('stop', 'hold'):
                raise Rejected('INVALID_OPERATION')
            self.engine.halt(self.now())
            # A newly accepted smooth stop has not called the device stop yet.
            # Previous failed stop attempts must not reject this new request;
            # eventual success/failure is still reported by execution state.
            reply.ok = bool(self.engine.stopping) or not bool(self.robot_io.stop_failures)
            reply.code = 'STOP_REQUESTED' if reply.ok else 'STOP_REQUEST_FAILED'
        except Rejected as exc:
            reply.ok, reply.code = False, str(exc)
        return reply

    def fault(self, request, reply):
        reply.ok = inject_simulated_fault(self.devices, request.fault)
        reply.code = 'OK' if reply.ok else 'UNKNOWN_FAULT'
        return reply

    def tick(self):
        now = self.now()
        self.engine.tick(now)
        while self.engine.events:
            self.last_event = e = self.engine.events.popleft()
            self.event(e.state, e.code, e.task_id, e.command_id, e.observation_id)
        fb = self.engine.feedback
        active = self.engine.active
        msg = ExecutionState(stamp=stamp(now), feedback_stamp=stamp(fb.stamp),
                             instance_id=self.instance_id, profile_hash=self.profile.digest,
                             simulated=fb.simulated, runtime_mode=self.runtime_mode, device_backend=self.device_backend,
                             base_x=fb.base_x, base_y=fb.base_y, base_yaw=fb.base_yaw)
        msg.joints.header.stamp = stamp(fb.stamp)
        msg.feedback_sources = list(self.robot_io.feedback.source_stamps)
        msg.source_stamps = [stamp(t) for t in self.robot_io.feedback.source_stamps.values()]
        msg.stop_failures = list(self.robot_io.stop_failures)
        msg.joints.name = list(fb.positions)
        msg.joints.position = list(fb.positions.values())
        msg.reference.header.stamp = stamp(now)
        reference = self.robot_io.reference_positions
        msg.reference.name = list(reference)
        msg.reference.position = list(reference.values())
        # An empty JointState velocity vector explicitly means unavailable.
        # Untagged finite differences must not masquerade as measured rest.
        msg.joints.velocity = ([fb.joint_velocities[j] for j in msg.joints.name]
                               if all(j in fb.joint_velocities for j in msg.joints.name) else [])
        msg.base_velocity.linear.x, msg.base_velocity.angular.z = fb.base_velocity
        msg.revision = self.engine.revision
        msg.lease_active = bool(self.engine.lease_id)
        msg.lease_epoch = self.engine.epoch
        msg.lease_resources = list(self.engine.groups) if self.engine.lease_id else []
        if self.engine.reference:
            values = dict(zip(self.engine.reference_names, zip(self.engine.reference.v,
                          self.engine.reference.a, self.engine.reference.j)))
            msg.reference.velocity = [values.get(j, (0., 0., 0.))[0] for j in msg.reference.name]
            msg.reference_accelerations = [values.get(j, (0., 0., 0.))[1] for j in msg.reference.name]
            msg.reference_jerks = [values.get(j, (0., 0., 0.))[2] for j in msg.reference.name]
        else:
            msg.reference.velocity = [0.]*len(msg.reference.name)
            msg.reference_accelerations = [0.]*len(msg.reference.name)
            msg.reference_jerks = [0.]*len(msg.reference.name)
        if self.engine.stopping:
            e, code = self.engine.stopping
            msg.command_id, msg.task_id = (e.command_id, e.task_id) if e else ('', '')
            msg.state, msg.code = 'STOPPING', code
        elif active:
            msg.command_id, msg.task_id, msg.state, msg.code = active.command_id, active.task_id, 'RUNNING', 'OK'
        elif self.last_event:
            e = self.last_event
            msg.command_id, msg.task_id, msg.state, msg.code = e.command_id, e.task_id, e.state, e.code
        else:
            msg.state, msg.code = 'IDLE', 'OK'
        self.state_pub.publish(msg)

    def destroy_node(self):
        if not self.closing:
            self.begin_shutdown()
        return super().destroy_node()

    def begin_shutdown(self):
        self.closing = True
        self.engine.halt(self.now(), 'SHUTDOWN', revoke=True, emergency=True)

    def abort_shutdown(self):
        self.engine.fail_stop(self.now(), 'STOP_FEEDBACK_TIMEOUT')


def main():
    run_execution(ExecutionNode)


def run_execution(factory):
    """Keep DDS alive for a bounded stop handshake on SIGINT and SIGTERM."""
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    requested = False
    node, deadline, exit_code = None, None, 0
    def request_shutdown(signum, frame):
        nonlocal requested
        requested = True
        if node is not None:
            node.closing = True  # Fence callbacks as soon as the signal arrives.
    previous = {s: signal.signal(s, request_shutdown) for s in (signal.SIGINT, signal.SIGTERM)}
    executor = SingleThreadedExecutor()
    try:
        node = factory()
        executor.add_node(node)
        while rclpy.ok():
            if requested and deadline is None:
                node.begin_shutdown()
                deadline = time.monotonic()+node.profile.stop_timeout+.5
            if requested:
                if not node.engine.stopping:
                    break
                if time.monotonic() >= deadline:
                    node.abort_shutdown()
                    break
            executor.spin_once(timeout_sec=.02)
            if requested:
                if deadline is None:
                    node.begin_shutdown()
                    deadline = time.monotonic()+node.profile.stop_timeout+.5
                # ROS timers may stop when /clock is paused; the stop budget
                # and feedback handshake must still progress during teardown.
                node.tick()
        if node.robot_io.stop_failures or node.engine.stop_fault:
            node.get_logger().error('Shutdown: device stop unconfirmed')
            exit_code = 1
        elif requested:
            node.get_logger().info('Shutdown: stop confirmed or no commanded axes')
    finally:
        executor.shutdown(timeout_sec=1.)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for signum, handler in previous.items():
            signal.signal(signum, handler)
    raise SystemExit(exit_code)
