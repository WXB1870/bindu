"""Task/skill orchestration talks only to typed capability and execution ports."""
import threading
import time
import uuid
from dataclasses import dataclass
from rclpy.task import Future
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from bindu_interfaces.action import FetchDrink
from bindu_interfaces.msg import ExecutionState, RecorderHealth, MotionCommand
from bindu_interfaces.srv import Lease, SubmitMotion, ControlExecution, LocateObject
from bindu_tasks.scene import Scene
from bindu_contracts.contracts import Observation
from bindu_runtime.plugins import load_provider
from bindu_tasks.fetch_drink import fetch_drink
from .common import RuntimeNode, seconds, stamp, spin


class Canceled(Exception):
    pass


@dataclass
class Attempt:
    task_id: str
    goal: object
    lease_id: str = ''
    epoch: int = 0
    observation_id: str = ''
    renewal: object = None
    fault: str = ''


class TaskNode(RuntimeNode):
    def __init__(self):
        super().__init__('task')
        self.declare_parameter('planner_provider', 'bindu_planning.simulated:PlannerStrategy')
        self.declare_parameter('chunk_provider', 'bindu_vla.simulated:ChunkStrategy')
        self.declare_parameter('navigation_provider', 'bindu_navigation.simulated:SimNavigation')
        self.strategies = {'planner':load_provider(self.get_parameter('planner_provider').value),
                           'chunk':load_provider(self.get_parameter('chunk_provider').value)}
        self.navigation = load_provider(self.get_parameter('navigation_provider').value)
        self.scene = Scene()
        self.state = self.health = None
        self.current = None
        self.busy = threading.Lock()
        self.group = ReentrantCallbackGroup()
        self.lease_client = self.create_client(Lease, 'execution/lease', callback_group=self.group)
        self.submit_client = self.create_client(SubmitMotion, 'execution/submit', callback_group=self.group)
        self.control_client = self.create_client(ControlExecution, 'execution/control', callback_group=self.group)
        self.locate_client = self.create_client(LocateObject, 'perception/locate', callback_group=self.group)
        self.create_subscription(ExecutionState, 'execution/state', self.on_state, 1, callback_group=self.group)
        self.create_subscription(RecorderHealth, 'recorder/health', self.on_health, 5, callback_group=self.group)
        self.server = ActionServer(self, FetchDrink, 'tasks/fetch_drink', execute_callback=self.execute,
                                   goal_callback=self.goal, cancel_callback=lambda _:CancelResponse.ACCEPT,
                                   callback_group=self.group)
        self.create_timer(.3, self.renew, callback_group=self.group)

    def on_state(self, state):
        self.state = state

    def on_health(self, health):
        if health.run_id == self.run_id:
            self.health = health

    def goal(self, request):
        ready = (self.state and self.state.profile_hash == self.profile.digest and self.health and
                 self.health.ready and 0 <= self.now()-seconds(self.health.stamp) < .75 and
                 all(c.service_is_ready() for c in (self.lease_client,self.submit_client,self.control_client,self.locate_client)))
        supported = (request.strategy in self.strategies and request.object_id == 'drink' and request.task_id and
                     {'arm','hand'} <= set(self.profile.groups) and
                     {'joint_position','base_velocity','simulated_grasp'} <= set(self.profile.capabilities))
        if not ready or not supported or self.scene.held_object or not self.busy.acquire(blocking=False):
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def renew(self):
        ctx = self.current
        if not ctx or not ctx.lease_id:
            return
        if ctx.renewal:
            if not ctx.renewal.done():
                return
            if ctx.renewal.exception() or not ctx.renewal.result().ok:
                ctx.fault = 'LEASE_RENEW_FAILED'
                return
        ctx.renewal = self.lease_client.call_async(Lease.Request(operation='renew', lease_id=ctx.lease_id, epoch=ctx.epoch))

    def check(self, ctx):
        if ctx.goal.is_cancel_requested:
            raise Canceled()
        if ctx.fault:
            raise RuntimeError(ctx.fault)
        if not self.health or not self.health.ready or not 0 <= self.now()-seconds(self.health.stamp) <= .75:
            raise RuntimeError('RECORDER_UNAVAILABLE')
        if not self.state or not 0 <= self.now()-seconds(self.state.feedback_stamp) <= .3:
            raise RuntimeError('FEEDBACK_STALE')

    async def pause(self):
        # Yield the ROS executor; do not block a worker while awaiting a service.
        future = Future(executor=self.executor)
        timer = self.create_timer(.01, lambda: future.set_result(None) if not future.done() else None)
        try:
            await future
        finally:
            self.destroy_timer(timer)

    async def wait(self, future, timeout, ctx=None):
        end = time.monotonic() + timeout
        while not future.done():
            if ctx:
                self.check(ctx)
            if time.monotonic() >= end:
                raise RuntimeError('CAPABILITY_TIMEOUT')
            await self.pause()
        if ctx:
            self.check(ctx)
        return future.result()

    def stage(self, ctx, name):
        self.check(ctx)
        ctx.goal.publish_feedback(FetchDrink.Feedback(stage=name))
        self.event('STAGE', name, ctx.task_id, observation_id=ctx.observation_id)

    async def move(self, ctx, command):
        self.check(ctx)
        command.schema_version = 1
        command.command_id = uuid.uuid4().hex
        command.task_id, command.observation_id = ctx.task_id, ctx.observation_id
        command.profile_hash = self.profile.digest
        command.lease_id, command.epoch = ctx.lease_id, ctx.epoch
        command.stamp, command.valid_for = stamp(self.now()), 3.
        response = await self.wait(self.submit_client.call_async(SubmitMotion.Request(command=command)), 1., ctx)
        if not response.accepted:
            raise RuntimeError(response.code)
        end = time.monotonic() + 4
        while time.monotonic() < end:
            self.check(ctx)
            state = self.state
            if state and state.command_id == command.command_id:
                if state.state == 'SUCCEEDED':
                    return
                if state.state in ('FAILED','CANCELED','SUPERSEDED'):
                    raise RuntimeError(state.code)
            await self.pause()
        raise RuntimeError('EXECUTION_TIMEOUT')

    async def navigate(self, ctx, site):
        await self.navigation.navigate(self, ctx, site)

    async def base_segment(self, ctx, velocity, duration):
        command = MotionCommand(mode='base_velocity', resource_group='base', duration=duration)
        command.velocity.linear.x = velocity
        await self.move(ctx, command)

    async def joints(self, ctx, strategy, group, target):
        positions = dict(zip(self.state.joints.name, self.state.joints.position))
        plan = await strategy.plan(self.profile, group, tuple(positions[j] for j in self.profile.groups[group]), target)
        command = MotionCommand(mode='finite_trajectory', resource_group=plan.group, joint_names=list(plan.names))
        for i, (t, q) in enumerate(zip(plan.offsets, plan.points)):
            ns = round(t*1e9)
            command.points.append(JointTrajectoryPoint(positions=list(q),
                velocities=list(plan.velocities[i]) if plan.velocities else [],
                accelerations=list(plan.accelerations[i]) if plan.accelerations else [], time_from_start=Duration(sec=ns//1000000000,nanosec=ns%1000000000)))
        await self.move(ctx, command)

    async def stop(self, ctx):
        if not ctx.lease_id:
            return True
        try:
            # Revoke first: a late submission under this lease can no longer start motion.
            requested_at = self.now()
            response = await self.wait(self.lease_client.call_async(Lease.Request(operation='release',lease_id=ctx.lease_id,epoch=ctx.epoch)),1.)
            ctx.lease_id = ''
            end = time.monotonic()+self.profile.stop_timeout+.5
            while time.monotonic() < end:
                s = self.state
                if s and (s.code == 'STOP_FEEDBACK_TIMEOUT' or 'EMERGENCY_STOP' in s.code):
                    return False
                if (response.ok and s and s.state != 'STOPPING' and not s.reference.name and not s.stop_failures and seconds(s.feedback_stamp) >= requested_at and 0 <= self.now()-seconds(s.feedback_stamp) < .2 and
                    abs(s.base_velocity.linear.x)<1e-6 and abs(s.base_velocity.angular.z)<1e-6 and
                    all(abs(v)<.01 for v in s.joints.velocity)):
                    return True
                await self.pause()
        except RuntimeError:
            pass
        return False

    async def locate(self, ctx, object_id):
        reply = None
        for attempt in range(2):
            self.check(ctx)
            request = LocateObject.Request(schema_version=1, request_id=uuid.uuid4().hex,
                                          task_id=ctx.task_id,object_id=object_id)
            try:
                reply = await self.wait(self.locate_client.call_async(request), .7, ctx)
                if reply.request_id != request.request_id or not reply.found:
                    raise RuntimeError('INVALID_PERCEPTION_RESULT')
                p = reply.pose.pose.position
                self.scene.accept(Observation(reply.observation_id,reply.object_id,seconds(reply.pose.header.stamp),
                                              reply.pose.header.frame_id,(p.x,p.y,p.z),reply.simulated),self.now())
                break
            except (RuntimeError,ValueError):
                if attempt:
                    raise
                self.event('RETRY','LOCATE',ctx.task_id)
        ctx.observation_id = reply.observation_id

    def confirm_grasp(self, object_id, target):
        positions = dict(zip(self.state.joints.name, self.state.joints.position))
        self.scene.confirm_grasp(object_id, positions, self.profile.groups['hand'], target)

    async def execute(self, goal):
        ctx = Attempt(goal.request.task_id, goal)
        self.current = ctx
        result = FetchDrink.Result(simulated=True)
        try:
            lease = await self.wait(self.lease_client.call_async(Lease.Request(operation='acquire', owner=ctx.task_id,
                              resources=['arm','hand','base'])), 1., ctx)
            if not lease.ok:
                raise RuntimeError(lease.code)
            ctx.lease_id, ctx.epoch = lease.lease_id, lease.epoch
            if lease.profile_hash != self.profile.digest:
                raise RuntimeError('PROFILE_MISMATCH')
            strategy = self.strategies[goal.request.strategy]
            await fetch_drink(self, ctx, goal.request.object_id, strategy)
            if not await self.stop(ctx):
                raise RuntimeError('STOP_UNCONFIRMED')
            result.success, result.code, result.held_object = True, 'SIMULATED_TASK_COMPLETE', self.scene.held_object
            goal.succeed()
        except Canceled:
            result.code = 'CANCELED' if await self.stop(ctx) else 'STOP_UNCONFIRMED'
            if result.code == 'CANCELED':
                goal.canceled()
            else:
                goal.abort()
        except (RuntimeError,ValueError,KeyError) as exc:
            result.code = str(exc)
            if not await self.stop(ctx):
                result.code += ':STOP_UNCONFIRMED'
            goal.abort()
        finally:
            self.event('TASK_SUCCEEDED' if result.success else 'TASK_ENDED', result.code,ctx.task_id,observation_id=ctx.observation_id)
            self.current = None
            self.busy.release()
        return result


def main():
    spin(TaskNode, threaded=False)
