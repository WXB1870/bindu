"""Pi ROS 2 adapter: legacy ZMQ stream -> leased, expiring joint targets.

Networking lives in this process, never in the periodic execution process.
Only the existing simulated execution backend is enabled by RuntimeNode.
"""
import json
import math
from pathlib import Path
import time
import uuid
import numpy as np
import zmq
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.task import Future
from bindu_interfaces.action import PiSession
from bindu_interfaces.msg import ExecutionState, RecorderHealth, PiObservation, MotionCommand
from bindu_interfaces.srv import Lease, SubmitMotion
from bindu_vla.pi.adapter import load_config, observation_payload, CommandAdapter, IMAGE_KEYS
from bindu_vla.pi.transport import PiTransport
from .common import RuntimeNode, seconds, stamp, spin


class PiNode(RuntimeNode):
    def __init__(self):
        super().__init__('pi_client')
        self.declare_parameter('pi_config', str(Path(get_package_share_directory('bindu_runtime'))/'config/pi_loopback.json'))
        self.cfg = load_config(self.get_parameter('pi_config').value, self.profile)
        self.state = self.health = self.observation = None
        self.busy = False
        self.transport = None
        self.group = ReentrantCallbackGroup()
        self.lease_client = self.create_client(Lease, 'execution/lease', callback_group=self.group)
        self.submit_client = self.create_client(SubmitMotion, 'execution/submit', callback_group=self.group)
        self.create_subscription(ExecutionState, 'execution/state', self.on_state, 1)
        self.create_subscription(RecorderHealth, 'recorder/health', self.on_health, 5)
        self.create_subscription(PiObservation, 'vla/pi/observation', self.on_observation, 1)
        self.server = ActionServer(self, PiSession, 'vla/pi/session', execute_callback=self.execute,
                                   goal_callback=self.goal, cancel_callback=lambda _: CancelResponse.ACCEPT,
                                   callback_group=self.group)

    def on_state(self, message):
        self.state = message

    def on_health(self, message):
        if message.run_id == self.run_id:
            self.health = message

    def on_observation(self, message):
        self.observation = message

    def snapshot(self, session_id, prompt):
        obs = self.observation
        if obs is None or not obs.simulated:
            raise ValueError('PI_OBSERVATION_NOT_READY')
        timestamps = [seconds(obs.joints.header.stamp)] + [seconds(i.header.stamp) for i in obs.images]
        if (not all(0 <= self.now()-t <= self.cfg['observation_max_age'] for t in timestamps) or
                max(timestamps)-min(timestamps) > self.cfg['max_sync_skew']):
            raise ValueError('PI_OBSERVATION_STALE_OR_UNSYNCED')
        if (len(obs.joints.name) != len(obs.joints.position) or len(set(obs.joints.name)) != len(obs.joints.name) or
                len(obs.images) != 3 or len(obs.image_names) != 3 or set(obs.image_names) != set(IMAGE_KEYS)):
            raise ValueError('PI_OBSERVATION_LAYOUT')
        images = {}
        for name, image in zip(obs.image_names, obs.images):
            if (image.encoding != 'rgb8' or image.height != 224 or image.width != 224 or
                    image.step != 224*3 or len(image.data) != 224*224*3):
                raise ValueError('PI_IMAGE_LAYOUT')
            images[name] = np.frombuffer(bytes(image.data), dtype=np.uint8).reshape(224, 224, 3)
        payload = observation_payload(self.cfg, dict(zip(obs.joints.name, obs.joints.position)), images,
                                      obs.observation_id, session_id, prompt)
        return payload, min(timestamps)

    def check(self):
        if not self.health or not self.health.ready or not 0 <= self.now()-seconds(self.health.stamp) < .75:
            raise RuntimeError('RECORDER_UNAVAILABLE')
        if (not self.state or self.state.profile_hash != self.profile.digest or
                not 0 <= self.now()-seconds(self.state.feedback_stamp) < .3):
            raise RuntimeError('FEEDBACK_STALE')

    def goal(self, request):
        try:
            self.check()
            self.snapshot('readiness', request.prompt)
        except (RuntimeError, ValueError):
            return GoalResponse.REJECT
        if (self.busy or not request.task_id or request.resource_group not in self.profile.groups or
                not math.isfinite(request.duration) or not 0 < request.duration <= 60 or
                not self.lease_client.service_is_ready() or not self.submit_client.service_is_ready()):
            return GoalResponse.REJECT
        self.busy = True
        return GoalResponse.ACCEPT

    async def pause(self):
        future = Future(executor=self.executor)
        timer = self.create_timer(.01, lambda: future.set_result(None) if not future.done() else None)
        try:
            await future
        finally:
            self.destroy_timer(timer)

    async def wait(self, future, timeout=1.):
        end = time.monotonic()+timeout
        while not future.done():
            if time.monotonic() >= end:
                raise RuntimeError('PI_SERVICE_TIMEOUT')
            await self.pause()
        return future.result()

    async def release(self, lease):
        if not lease or not lease.ok:
            return True
        try:
            requested = self.now()
            reply = await self.wait(self.lease_client.call_async(Lease.Request(
                operation='release', lease_id=lease.lease_id, epoch=lease.epoch)))
            if not reply.ok:
                return False
            end = time.monotonic()+.5
            while time.monotonic() < end:
                state = self.state
                if (state and not state.stop_failures and seconds(state.feedback_stamp) >= requested and
                        0 <= self.now()-seconds(state.feedback_stamp) < .2 and
                        abs(state.base_velocity.linear.x) < 1e-6 and abs(state.base_velocity.angular.z) < 1e-6 and
                        all(abs(v) < .01 for v in state.joints.velocity)):
                    return True
                await self.pause()
        except RuntimeError:
            pass
        return False

    async def execute(self, goal):
        session_id = uuid.uuid4().hex
        task_id = goal.request.task_id
        lease = pending = renewal = None
        accepted = dropped = 0
        result = PiSession.Result(simulated=True)
        canceled = False
        try:
            lease = await self.wait(self.lease_client.call_async(Lease.Request(
                operation='acquire', owner=session_id, resources=[goal.request.resource_group])))
            if not lease.ok or lease.profile_hash != self.profile.digest:
                raise RuntimeError(lease.code if not lease.ok else 'PROFILE_MISMATCH')
            self.transport = PiTransport(self.cfg)
            adapter = CommandAdapter(self.cfg, self.profile, goal.request.resource_group, session_id, time.time())
            started = last_command = next_renewal = time.monotonic()
            pending_since = renewal_since = 0.
            self.event('PI_SESSION_STARTED', json.dumps({'session_id':session_id, 'mode':self.cfg['mode'],
                       'correlation':'required' if self.cfg['require_correlation'] else 'legacy_optional'}), task_id)
            while True:
                now = time.monotonic()
                if goal.is_cancel_requested:
                    canceled = True
                    break
                self.check()
                if (self.state.task_id == task_id and self.state.command_id.startswith(session_id) and
                        self.state.state in ('FAILED', 'CANCELED')):
                    raise RuntimeError(self.state.code)
                if pending is not None:
                    if pending.done():
                        reply = pending.result()
                        if not reply.accepted:
                            raise RuntimeError(reply.code)
                        accepted += 1
                        pending = None
                    elif now-pending_since > .7:
                        raise RuntimeError('PI_SUBMIT_TIMEOUT')
                if renewal is not None:
                    if renewal.done():
                        if not renewal.result().ok:
                            raise RuntimeError('LEASE_RENEW_FAILED')
                        renewal = None
                    elif now-renewal_since > .7:
                        raise RuntimeError('LEASE_RENEW_TIMEOUT')
                if now >= next_renewal and renewal is None:
                    renewal = self.lease_client.call_async(Lease.Request(operation='renew', lease_id=lease.lease_id, epoch=lease.epoch))
                    renewal_since, next_renewal = now, now+.3
                if now-last_command > self.cfg['command_timeout']:
                    raise RuntimeError('PI_COMMAND_TIMEOUT')
                if now-started >= goal.request.duration:
                    if not accepted:
                        raise RuntimeError('PI_NO_ACCEPTED_COMMAND')
                    break
                payload, source_stamp = self.snapshot(session_id, goal.request.prompt)
                obs_id = payload['bindu']['observation_id']
                adapter.observe(obs_id, source_stamp)
                messages, errors = self.transport.exchange(payload)
                latest = None
                for message in messages:
                    try:
                        converted = adapter.convert(message, time.time(), self.now())
                        if latest is not None:
                            dropped += 1
                        latest = (message, converted)
                    except ValueError as exc:
                        errors.append(str(exc))
                for error in errors:
                    dropped += 1
                    self.event('PI_DROPPED', error, task_id)
                if latest is not None:
                    if pending is not None:
                        dropped += 1
                    else:
                        message, (names, values, source_time, correlated_obs) = latest
                        command = MotionCommand(schema_version=1, command_id=session_id+'_'+str(message.seq),
                            task_id=task_id, observation_id=correlated_obs, profile_hash=self.profile.digest,
                            lease_id=lease.lease_id, epoch=lease.epoch, mode='joint_target',
                            resource_group=goal.request.resource_group, stamp=stamp(source_time),
                            valid_for=self.cfg['command_max_age'], joint_names=list(names), positions=list(values))
                        pending = self.submit_client.call_async(SubmitMotion.Request(command=command))
                        pending_since = last_command = now
                        self.event('PI_COMMAND_RECEIVED', json.dumps({'seq':message.seq,
                            'source_wall':message.timestamp_send, 'age_ms':(self.now()-source_time)*1000,
                            'latest_observation_id':obs_id, 'correlated':bool(correlated_obs)}),
                            task_id, command.command_id, correlated_obs)
                goal.publish_feedback(PiSession.Feedback(session_id=session_id, accepted_commands=accepted,
                                                         dropped_commands=dropped))
                await self.pause()
            result.code = 'CANCELED' if canceled else 'PI_SESSION_COMPLETE'
        except (RuntimeError, ValueError, zmq.ZMQError) as exc:
            result.code = str(exc)
        finally:
            # Close input before revoking. Late submitted commands carry the old lease.
            if self.transport is not None:
                self.transport.close()
                self.transport = None
            stopped = await self.release(lease)
            # A submit may have been accepted just before cancellation. Revoke
            # first, then account for its bounded acknowledgement without retry.
            if pending is not None:
                try:
                    if (await self.wait(pending, .7)).accepted:
                        accepted += 1
                except RuntimeError:
                    self.event('PI_SUBMIT_UNCONFIRMED', session_id, task_id)
            if not stopped:
                result.code += ':STOP_UNCONFIRMED'
            result.accepted_commands = accepted
            result.success = result.code == 'PI_SESSION_COMPLETE'
            self.event('PI_SESSION_ENDED', json.dumps({'session_id':session_id, 'code':result.code,
                       'accepted':accepted, 'dropped':dropped}), task_id)
            self.busy = False
        if result.success:
            goal.succeed()
        elif canceled and stopped:
            goal.canceled()
        else:
            goal.abort()
        return result


def main():
    spin(PiNode, threaded=False)
