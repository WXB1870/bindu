"""VR -> bounded IK worker -> leased joint stream. Simulation devices only."""
import json
import hashlib
import math
import time
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.task import Future
from std_srvs.srv import Trigger
from bindu_contracts.teleoperation import VRFrame
from bindu_teleoperation.config import load_config
from bindu_teleoperation.session import TeleopSession
from bindu_kinematics.worker import KinematicsWorker
from bindu_interfaces.action import TeleopSession as TeleopAction
from bindu_interfaces.msg import VRInput, ExecutionState, RecorderHealth, MotionCommand
from bindu_interfaces.srv import Lease, SubmitMotion, ControlExecution
from .common import RuntimeNode, seconds, stamp, spin


class TeleopNode(RuntimeNode):
    def __init__(self):
        super().__init__('teleop')
        self.declare_parameter('teleop_config', str(Path(get_package_share_directory('bindu_runtime'))/'config/teleop_v34.json'))
        self.declare_parameter('model_root', str(Path(get_package_share_directory('bindu_kinematics'))/'models'))
        self.cfg = load_config(self.get_parameter('teleop_config').value, self.profile,
                               self.get_parameter('model_root').value)
        self.worker = KinematicsWorker(self.cfg['kinematics'])
        self.worker_error = ''
        self.solution = self.frame = self.state = self.health = None
        self.busy = False
        group = ReentrantCallbackGroup()
        self.lease_client = self.create_client(Lease, 'execution/lease', callback_group=group)
        self.submit_client = self.create_client(SubmitMotion, 'execution/submit', callback_group=group)
        self.control_client = self.create_client(ControlExecution, 'execution/control', callback_group=group)
        self.create_subscription(VRInput, 'teleop/vr/input', self.on_frame, 1)
        self.create_subscription(ExecutionState, 'execution/state', lambda m: setattr(self, 'state', m), 1)
        self.create_subscription(RecorderHealth, 'recorder/health', self.on_health, 5)
        self.create_timer(.005, self.poll_worker)
        self.create_service(Trigger, 'teleop/ready', self.ready)
        self.server = ActionServer(self, TeleopAction, 'teleop/session', execute_callback=self.execute,
                                  goal_callback=self.goal, cancel_callback=lambda _: CancelResponse.ACCEPT,
                                  callback_group=group)

    def on_health(self, msg):
        if msg.run_id == self.run_id:
            self.health = msg

    def on_frame(self, msg):
        self.frame = VRFrame(msg.source_id, msg.seq, seconds(msg.stamp), msg.side, tuple(msg.pose),
                             msg.grip, msg.trigger, msg.init, msg.stop, msg.valid, msg.clutch_seq)

    def poll_worker(self):
        if self.worker_error:
            return
        try:
            result = self.worker.poll()
            if result is not None:
                self.solution = result
        except RuntimeError as exc:
            self.worker_error = str(exc)
            self.event('TELEOP_IK_FAILED', self.worker_error)

    def check(self):
        now = self.now()
        if self.worker_error:
            raise RuntimeError(self.worker_error)
        if not self.worker.ready:
            raise RuntimeError('IK_NOT_READY')
        if (self.state is None or self.state.profile_hash != self.profile.digest or
                not self.state.simulated or not 0 <= now-seconds(self.state.feedback_stamp) <= self.cfg['feedback_max_age']):
            raise RuntimeError('TELEOP_FEEDBACK_STALE')
        if self.health is None or not self.health.ready or not 0 <= now-seconds(self.health.stamp) < .75:
            raise RuntimeError('RECORDER_UNAVAILABLE')
        if self.frame is None or not 0 <= now-self.frame.stamp <= self.cfg['input_max_age']:
            raise RuntimeError('VR_INPUT_TIMEOUT')

    def readiness_error(self):
        try:
            self.check()
            probe = TeleopSession(self.cfg, self.profile.groups[self.cfg['resource_group']])
            probe.ingest(self.frame, self.now())
            if self.busy:
                return 'TELEOP_BUSY'
            if not all(c.service_is_ready() for c in (self.lease_client, self.submit_client, self.control_client)):
                return 'EXECUTION_NOT_READY'
        except (RuntimeError, ValueError) as exc:
            return str(exc)
        return ''

    def ready(self, request, reply):
        reason = self.readiness_error()
        reply.success, reply.message = not bool(reason), reason or 'TELEOP_READY'
        return reply

    def goal(self, request):
        reason = self.readiness_error()
        if (not request.task_id or request.resource_group != self.cfg['resource_group'] or
                not math.isfinite(request.duration) or not 0 < request.duration <= 3600):
            reason = 'TELEOP_INVALID_GOAL'
        if reason:
            self.event('TELEOP_GOAL_REJECTED', reason, request.task_id)
            return GoalResponse.REJECT
        self.busy = True
        return GoalResponse.ACCEPT

    async def pause(self):
        future = Future(executor=self.executor)
        timer = self.create_timer(.005, lambda: future.set_result(None) if not future.done() else None)
        try:
            await future
        finally:
            self.destroy_timer(timer)

    async def wait(self, future, timeout=.7):
        end = time.monotonic()+timeout
        while not future.done():
            if time.monotonic() >= end:
                raise RuntimeError('TELEOP_SERVICE_TIMEOUT')
            await self.pause()
        return future.result()

    async def stop(self, lease, release=False):
        requested = self.now()
        if release:
            future = self.lease_client.call_async(Lease.Request(operation='release', lease_id=lease.lease_id, epoch=lease.epoch))
        else:
            future = self.control_client.call_async(ControlExecution.Request(operation='hold', lease_id=lease.lease_id, epoch=lease.epoch))
        try:
            reply = await self.wait(future)
            if not reply.ok:
                return False
            end = time.monotonic()+self.profile.stop_timeout+.5
            while time.monotonic() < end:
                s = self.state
                if s and (s.code == 'STOP_FEEDBACK_TIMEOUT' or 'EMERGENCY_STOP' in s.code):
                    return False
                if (s and not s.stop_failures and seconds(s.feedback_stamp) >= requested and
                        0 <= self.now()-seconds(s.feedback_stamp) < self.cfg['feedback_max_age'] and
                        not s.reference.name and all(abs(v) < .01 for v in s.joints.velocity)):
                    return True
                await self.pause()
        except RuntimeError:
            pass
        return False

    async def execute(self, goal):
        session = TeleopSession(self.cfg, self.profile.groups[self.cfg['resource_group']])
        result = TeleopAction.Result(simulated=True)
        lease = pending = renewal = None
        accepted = 0
        canceled = False
        stopped = True
        self.solution = None
        try:
            lease = await self.wait(self.lease_client.call_async(Lease.Request(operation='acquire',
                owner=session.identity, resources=[goal.request.resource_group])))
            if not lease.ok or lease.profile_hash != self.profile.digest:
                raise RuntimeError(lease.code if not lease.ok else 'PROFILE_MISMATCH')
            started = time.monotonic()
            next_renewal = started
            pending_at = renewal_at = 0.
            last_mode = 'idle'
            self.event('TELEOP_STARTED', json.dumps({'session_id':session.identity,
                'input_source':self.frame.source_id, 'config':self.cfg,
                'model_sha256':hashlib.sha256(Path(self.cfg['kinematics']['urdf']).read_bytes()).hexdigest(),
                'collision_checked':False, 'input_timestamp':'receiver_event'}), goal.request.task_id)
            while True:
                now = time.monotonic()
                if goal.is_cancel_requested:
                    canceled = True
                    break
                if now-started >= goal.request.duration:
                    break
                self.check()
                session.ingest(self.frame, self.now())
                session.check(self.now())
                if pending is not None:
                    if pending.done():
                        reply = pending.result()
                        if not reply.accepted:
                            raise RuntimeError(reply.code)
                        accepted += 1
                        pending = None
                    elif now-pending_at > .7:
                        raise RuntimeError('TELEOP_SUBMIT_TIMEOUT')
                if renewal is not None:
                    if renewal.done():
                        if not renewal.result().ok:
                            raise RuntimeError('LEASE_RENEW_FAILED')
                        renewal = None
                    elif now-renewal_at > .7:
                        raise RuntimeError('LEASE_RENEW_TIMEOUT')
                if now >= next_renewal and renewal is None:
                    renewal = self.lease_client.call_async(Lease.Request(operation='renew', lease_id=lease.lease_id, epoch=lease.epoch))
                    renewal_at, next_renewal = now, now+.3
                if session.mode != last_mode:
                    if session.mode == 'idle':
                        # Await earlier submission before hold; never publish a late
                        # accepted follow target after reporting clutch release.
                        if pending is not None:
                            reply = await self.wait(pending)
                            if not reply.accepted:
                                raise RuntimeError(reply.code)
                            accepted += 1
                            pending = None
                        if not await self.stop(lease):
                            raise RuntimeError('TELEOP_HOLD_UNCONFIRMED')
                    last_mode = session.mode
                    self.event('TELEOP_MODE', session.mode, goal.request.task_id)
                    if session.mode == 'idle':
                        # Physical braking may outlast the input deadline.
                        # Re-ingest the receiver's latest frame next iteration;
                        # do not age-check the pre-braking session snapshot.
                        await self.pause()
                        continue
                s = self.state
                if (session.mode == 'follow' and s.command_id.startswith(session.identity+'_') and
                        s.state == 'FAILED'):
                    raise RuntimeError(s.code)
                if self.solution is not None and pending is None:
                    solution, self.solution = self.solution, None
                    req = solution.request
                    if req.context:
                        measured = dict(zip(s.joints.name, s.joints.position))
                        names = sorted(self.cfg['kinematics']['locked_joints'])
                        if any(not math.isfinite(measured[n]) or abs(measured[n]-v)>.01
                               for n,v in zip(names,req.context)):
                            raise ValueError('TELEOP_CONTEXT_CHANGED')
                    self.event('TELEOP_IK_RESULT', json.dumps({'code':solution.code,
                        'generation':req.generation, 'elapsed':solution.elapsed,
                        'target':req.target, 'input_stamp':req.stamp, 'position_error':solution.position_error,
                        'rotation_error':solution.rotation_error}), goal.request.task_id,
                        req.request_id, req.request_id)
                    if session.accept_result(solution, self.now()):
                        command = MotionCommand(schema_version=1, command_id=req.request_id,
                            task_id=goal.request.task_id, observation_id=req.request_id,
                            profile_hash=self.profile.digest, lease_id=lease.lease_id, epoch=lease.epoch,
                            mode='joint_target', resource_group=goal.request.resource_group,
                            stamp=stamp(req.stamp), valid_for=req.expires-req.stamp,
                            joint_names=list(req.names), positions=list(solution.positions))
                        pending = self.submit_client.call_async(SubmitMotion.Request(command=command))
                        pending_at = now
                if not self.worker.busy and self.solution is None and pending is None:
                    request = session.request(dict(zip(s.joints.name, s.joints.position)), seconds(s.feedback_stamp), self.now())
                    if request is not None:
                        self.worker.submit(request)
                goal.publish_feedback(TeleopAction.Feedback(session_id=session.identity, mode=session.mode,
                                                             accepted_commands=accepted))
                await self.pause()
            result.code = 'CANCELED' if canceled else 'TELEOP_SESSION_COMPLETE'
        except (RuntimeError, ValueError, KeyError) as exc:
            result.code = str(exc)
        finally:
            if lease is not None and lease.ok:
                stopped = await self.stop(lease, release=True)
            if pending is not None:
                try:
                    if (await self.wait(pending)).accepted:
                        accepted += 1
                except RuntimeError:
                    self.event('TELEOP_SUBMIT_UNCONFIRMED', session.identity, goal.request.task_id)
            if not stopped:
                result.code += ':STOP_UNCONFIRMED'
            result.accepted_commands = accepted
            result.success = result.code == 'TELEOP_SESSION_COMPLETE'
            self.event('TELEOP_ENDED', json.dumps({'session_id':session.identity, 'code':result.code,
                       'accepted':accepted}), goal.request.task_id)
            self.busy = False
        if result.success:
            goal.succeed()
        elif canceled and stopped:
            goal.canceled()
        else:
            goal.abort()
        return result

    def destroy_node(self):
        self.worker.close()
        return super().destroy_node()


def main():
    spin(TeleopNode, threaded=False)
