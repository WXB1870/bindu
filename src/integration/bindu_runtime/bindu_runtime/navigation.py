"""Nav2 action adapter for Bindu's existing task port.

Velocity producers must implement NavigationVelocity at command generation.
Unmodified Nav2 cmd_vel is deliberately not relabeled with the active goal.
"""
import json
import time
import uuid
from pathlib import Path
from builtin_interfaces.msg import Time


def seconds(value):
    return value.sec + value.nanosec / 1e9


def stamp(value):
    ns = int(value*1e9)
    return Time(sec=ns//1000000000, nanosec=ns%1000000000)


from bindu_navigation.navigation import NavigationConfig, NavigationGate, planar_pose


class Nav2Navigation:
    def bind(self, port):
        from ament_index_python.packages import get_package_share_directory
        from rclpy.action import ActionClient
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from nav2_msgs.action import NavigateToPose
        from bindu_interfaces.msg import NavigationVelocity, NavigationPose
        port.declare_parameter('navigation_config', str(Path(get_package_share_directory('bindu_runtime'))/'config/navigation_sim.json'))
        port.declare_parameter('navigation_backend', 'navigation/backend')
        self.config = NavigationConfig.load(port.get_parameter('navigation_config').value)
        backend = port.get_parameter('navigation_backend').value.rstrip('/')
        self.port, self.gate, self.pose = port, None, None
        self.client = ActionClient(port, NavigateToPose, backend+'/navigate_to_pose', callback_group=port.group)
        self.velocity_sub = port.create_subscription(NavigationVelocity, backend+'/velocity', self.on_velocity, QoSProfile(depth=8, reliability=ReliabilityPolicy.BEST_EFFORT), callback_group=port.group)
        self.pose_sub = port.create_subscription(NavigationPose, backend+'/pose', self.on_pose, QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT), callback_group=port.group)

    def on_velocity(self, msg):
        gate = self.gate
        if gate is None:
            return
        c = msg.command
        v = c.twist
        if msg.goal_id == gate.goal_id and any(x != 0. for x in (v.linear.y, v.linear.z, v.angular.x, v.angular.y)):
            gate.fault = 'NAV_UNSUPPORTED_VELOCITY_AXIS'
            gate.close()
            return
        gate.accept_velocity(msg.goal_id, msg.source_id, msg.sequence, seconds(c.header.stamp),
                             c.header.frame_id, (v.linear.x, v.angular.z), self.port.now())

    def on_pose(self, msg):
        self.pose = msg

    def update_pose(self, gate):
        if self.pose is None:
            return
        p = self.pose.pose.pose
        pose = planar_pose((p.position.x, p.position.y, p.position.z),
                           (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w))
        gate.accept_pose(self.pose.map_id, self.pose.pose.header.frame_id,
                         seconds(self.pose.pose.header.stamp), pose, self.port.now())

    async def navigate(self, port, context, site):
        from action_msgs.msg import GoalStatus
        from nav2_msgs.action import NavigateToPose
        from unique_identifier_msgs.msg import UUID
        from bindu_interfaces.msg import MotionCommand
        from bindu_interfaces.srv import SubmitMotion
        import math
        if site not in self.config.sites:
            raise RuntimeError('NAV_UNKNOWN_SITE')
        if not self.client.server_is_ready():
            raise RuntimeError('NAV_BACKEND_UNAVAILABLE')
        target = self.config.sites[site]
        goal_id = uuid.uuid4().hex
        gate = NavigationGate(self.config, goal_id, port.now())
        self.update_pose(gate)
        gate.check(port.now(), require_velocity=False)
        port.check(context)
        self.gate = gate
        handle = send = result = None
        started = time.monotonic()
        instance = port.state.instance_id
        revision = None
        last_sequence = -1
        last_command = None
        succeeded = False
        try:
            request = NavigateToPose.Goal()
            request.pose.header.frame_id = self.config.frame
            request.pose.header.stamp = stamp(port.now())
            request.pose.pose.position.x, request.pose.pose.position.y = target.x, target.y
            request.pose.pose.orientation.z = math.sin(target.yaw/2)
            request.pose.pose.orientation.w = math.cos(target.yaw/2)
            send = self.client.send_goal_async(request, goal_uuid=UUID(uuid=list(bytes.fromhex(goal_id))))
            handle = await port.wait(send, 2., context)
            if not handle.accepted:
                raise RuntimeError('NAV_GOAL_REJECTED')
            result = handle.get_result_async()
            port.event('NAV_STARTED', site, context.task_id, goal_id)
            while True:
                port.check(context)
                if time.monotonic()-started >= self.config.timeout:
                    raise RuntimeError('NAV_TIMEOUT')
                if port.state.instance_id != instance:
                    raise RuntimeError('NAV_EXECUTION_RESTARTED')
                if last_command and port.state.command_id == last_command and port.state.state in ('FAILED', 'CANCELED'):
                    raise RuntimeError('NAV_EXECUTION_FAILED:'+port.state.code)
                if port.state.stop_failures:
                    raise RuntimeError('NAV_EXECUTION_STOP_FAILED')
                self.update_pose(gate)
                gate.check(port.now(), require_velocity=False)
                if result.done():
                    response = result.result()
                    if response.status != GoalStatus.STATUS_SUCCEEDED or response.result.error_code:
                        raise RuntimeError('NAV_ACTION_FAILED:'+str(response.result.error_code))
                    if not gate.at_site(target):
                        raise RuntimeError('NAV_GOAL_POSE_MISMATCH')
                    succeeded = True
                    break
                if gate.velocity is None:
                    if time.monotonic()-started >= 2.:
                        raise RuntimeError('NAV_INPUT_TIMEOUT')
                else:
                    gate.check(port.now())
                    if gate.sequence != last_sequence:
                        last_sequence = gate.sequence
                        source_stamp, (linear, angular) = gate.velocity
                        command = MotionCommand(schema_version=1, command_id=uuid.uuid4().hex,
                            lease_id=context.lease_id, epoch=context.epoch, profile_hash=port.profile.digest,
                            task_id=context.task_id, observation_id=goal_id, mode='base_velocity',
                            resource_group='base', stamp=stamp(source_stamp),
                            valid_for=self.config.input_timeout+.1, duration=self.config.input_timeout,
                            expected_revision=port.state.revision if revision is None else revision)
                        command.velocity.linear.x, command.velocity.angular.z = linear, angular
                        reply = await port.wait(port.submit_client.call_async(SubmitMotion.Request(command=command)), .5, context)
                        if not reply.accepted:
                            raise RuntimeError('NAV_EXECUTION_REJECTED:'+reply.code)
                        revision = reply.revision
                        last_command = command.command_id
                await port.pause()
        except Exception as exc:
            port.event('NAV_ERROR', json.dumps({'code': str(exc), 'now': port.now(),
                'pose_stamp': gate.pose[0] if gate.pose else None,
                'input_stamp': gate.velocity[0] if gate.velocity else None,
                'sequence': gate.sequence}), context.task_id, goal_id)
            raise
        finally:
            # Fence submissions before any asynchronous cancellation or stop.
            gate.close()
            self.gate = None
            cancel = None
            uncertain = False
            if handle is not None and handle.accepted and (result is None or not result.done()):
                cancel = handle.cancel_goal_async()
            elif send is not None and handle is None:
                # Goal response can arrive after our timeout/cancellation.
                def cancel_late(future):
                    try:
                        late = future.result()
                        if late.accepted:
                            late.cancel_goal_async()
                    except Exception as exc:
                        port.get_logger().error('NAV_LATE_CANCEL_FAILED: '+str(exc))
                send.add_done_callback(cancel_late)
                uncertain = True
            stopped = await self.stop(port, context, gate)
            if cancel is not None:
                try:
                    await port.wait(cancel, .5)
                    if result is not None:
                        await port.wait(result, .5)
                except RuntimeError:
                    uncertain = True
            if not stopped:
                raise RuntimeError('NAV_STOP_UNCONFIRMED')
            if uncertain:
                raise RuntimeError('NAV_CANCEL_UNCONFIRMED')
        if succeeded:
            # Recheck localization after the stopping dwell, not just at result time.
            check = NavigationGate(self.config, goal_id, port.now())
            self.update_pose(check)
            check.check(port.now(), require_velocity=False)
            if not check.at_site(target):
                raise RuntimeError('NAV_GOAL_POSE_MISMATCH')
            port.event('NAV_SUCCEEDED', site, context.task_id, goal_id)

    async def stop(self, port, context, gate):
        from bindu_interfaces.srv import ControlExecution
        requested_at = port.now()
        instance = port.state.instance_id
        try:
            reply = await port.wait(port.control_client.call_async(ControlExecution.Request(
                operation='stop', lease_id=context.lease_id, epoch=context.epoch)), .5)
            if not reply.ok:
                return False
            end = time.monotonic()+self.config.stop_timeout
            while time.monotonic() < end:
                s = port.state
                if s and s.instance_id == instance and not s.stop_failures and s.state != 'STOPPING':
                    if gate.stopped(seconds(s.feedback_stamp), (s.base_velocity.linear.x, s.base_velocity.angular.z),
                                    port.now(), requested_at):
                        return True
                await port.pause()
        except RuntimeError:
            return False
        return False
