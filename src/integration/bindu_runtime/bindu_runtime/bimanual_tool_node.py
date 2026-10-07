"""Simulation-only dual-hand rigid-tool teleop through the shared position executor."""
import json
import math
import time
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.task import Future
from std_srvs.srv import Trigger
from bindu_contracts.teleoperation import VRFrame
from bindu_teleoperation.bimanual_session import BimanualSession
from bindu_kinematics.bimanual_worker import BimanualWorker
from bindu_interfaces.action import TeleopSession as TeleopAction
from bindu_interfaces.msg import VRInput, ExecutionState, RecorderHealth, MotionCommand, TeleopModeState
from bindu_interfaces.srv import Lease, SubmitMotion, ControlExecution
from .common import MODE_STATE_MAX_AGE, RuntimeNode, seconds, stamp, spin


class BimanualToolNode(RuntimeNode):
    def __init__(self):
        super().__init__('teleop_tool')
        share=Path(get_package_share_directory('bindu_runtime'))
        self.declare_parameter('tool_config',str(share/'config/bimanual_g1_sim.json'))
        self.declare_parameter('model_root',str(Path(get_package_share_directory('bindu_description'))/'urdf'))
        self.cfg=json.loads(Path(self.get_parameter('tool_config').value).read_text())
        if not self.cfg.get('simulation_only') or self.cfg.get('collision_scope') != 'empty_scene_only':
            raise ValueError('BIMANUAL_REQUIRES_EMPTY_SCENE_SIMULATION')
        urdf=Path(self.cfg['urdf'])
        self.cfg['urdf']=str(urdf if urdf.is_absolute() else
                             Path(self.get_parameter('model_root').value)/urdf)
        self.groups=tuple(self.cfg['resource_groups'])
        if (len(self.groups) not in (2,3) or len(set(self.groups))!=len(self.groups) or
                any(g not in self.profile.groups for g in self.groups)):
            raise ValueError('BIMANUAL_RESOURCE_LAYOUT')
        self.names=tuple(n for g in self.groups for n in self.profile.groups[g])
        self.worker=BimanualWorker(self.cfg,self.profile)
        self.worker_error=''
        self.worker_result=None
        self.preflight_token=0
        self.preflight_status=''
        self.preflight_at=0.
        self.preflight_error=''
        self.frames={}
        self.state=self.health=self.mode_state=None
        self.busy=False
        group=ReentrantCallbackGroup()
        self.lease_client=self.create_client(Lease,'execution/lease',callback_group=group)
        self.submit_client=self.create_client(SubmitMotion,'execution/submit',callback_group=group)
        self.control_client=self.create_client(ControlExecution,'execution/control',callback_group=group)
        self.create_subscription(VRInput,'teleop/vr/input',self.on_frame,5)
        self.create_subscription(ExecutionState,'execution/state',lambda m:setattr(self,'state',m),1)
        self.create_subscription(RecorderHealth,'recorder/health',self.on_health,5)
        self.create_subscription(TeleopModeState,'teleop/mode_state',
            lambda m:setattr(self,'mode_state',m),QoSProfile(depth=1,
                reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(.005,self.poll_worker)
        self.create_service(Trigger,'teleop/tool_ready',self.ready)
        self.server=ActionServer(self,TeleopAction,'teleop/tool_session',
            execute_callback=self.execute,goal_callback=self.goal,
            cancel_callback=lambda _:CancelResponse.ACCEPT,callback_group=group)

    def on_health(self,msg):
        if msg.run_id == self.run_id: self.health=msg

    def on_frame(self,msg):
        if msg.side in ('left','right'):
            self.frames[msg.side]=VRFrame(msg.source_id,msg.seq,seconds(msg.stamp),msg.side,
                tuple(msg.pose),msg.grip,msg.trigger,msg.init,msg.stop,msg.valid,msg.clutch_seq)

    def poll_worker(self):
        if self.worker_error: return
        try:
            item=self.worker.poll()
            if item is not None:
                kind,token,value=item
                if token==self.preflight_token and token<0:
                    self.preflight_status='ready' if kind=='result' else 'failed'
                    self.preflight_error='' if kind=='result' else value
                else:
                    self.worker_result=item
        except RuntimeError as exc:
            self.worker_error=str(exc)
            self.event('BIMANUAL_WORKER_FAILED',self.worker_error)

    def ready(self,request,reply):
        reason=self.worker_error or ('' if self.worker.ready else 'BIMANUAL_WORKER_NOT_READY')
        s=self.state
        if not reason and (self.busy or s is None or s.profile_hash!=self.profile.digest or
                not s.simulated or not 0<=self.now()-seconds(s.feedback_stamp)<=self.cfg['feedback_max_age']):
            reason='BIMANUAL_PREFLIGHT_FEEDBACK'
        if not reason:
            if self.preflight_status=='ready' and self.now()-self.preflight_at<=self.cfg['feedback_max_age']:
                pass
            elif self.preflight_status=='failed' and self.now()-self.preflight_at<=self.cfg['feedback_max_age']:
                reason=self.preflight_error
            elif self.preflight_status=='pending':
                reason='BIMANUAL_PREFLIGHT_PENDING'
            else:
                measured=dict(zip(s.joints.name,s.joints.position))
                self.preflight_token-=1
                if self.worker.submit('capture',self.preflight_token,measured):
                    self.preflight_status='pending'
                    self.preflight_at=self.now()
                reason='BIMANUAL_PREFLIGHT_PENDING'
        reply.success,reply.message=not bool(reason),reason or 'BIMANUAL_READY'
        return reply

    def check(self):
        now=self.now()
        m=self.mode_state
        if (m is None or m.profile_hash != self.profile.digest or
                m.phase != 'READY' or m.mode != 'bimanual_tool' or
                not 0 <= now-seconds(m.stamp) < MODE_STATE_MAX_AGE):
            raise RuntimeError('BIMANUAL_MODE_NOT_SELECTED')
        if self.worker_error or not self.worker.ready:
            raise RuntimeError(self.worker_error or 'BIMANUAL_WORKER_NOT_READY')
        s=self.state
        if (s is None or s.profile_hash != self.profile.digest or not s.simulated or
                not 0 <= now-seconds(s.feedback_stamp) <= self.cfg['feedback_max_age']):
            raise RuntimeError('BIMANUAL_FEEDBACK_STALE')
        if self.health is None or not self.health.ready or not 0 <= now-seconds(self.health.stamp)<.75:
            raise RuntimeError('RECORDER_UNAVAILABLE')
        if (any(side not in self.frames or self.frames[side].stamp <= seconds(m.changed_at) or
                not 0 <= now-self.frames[side].stamp <= self.cfg['input_max_age']
                for side in ('left','right'))):
            raise RuntimeError('VR_PAIR_TIMEOUT')

    def goal(self,request):
        try: self.check()
        except RuntimeError as exc:
            self.event('BIMANUAL_GOAL_REJECTED',str(exc),request.task_id)
            return GoalResponse.REJECT
        if (self.busy or not request.task_id or request.resource_group != 'bimanual_tool' or
                not math.isfinite(request.duration) or not 0 < request.duration <= 3600 or
                not all(c.service_is_ready() for c in
                        (self.lease_client,self.submit_client,self.control_client))):
            return GoalResponse.REJECT
        self.busy=True
        return GoalResponse.ACCEPT

    async def pause(self):
        future=Future(executor=self.executor)
        timer=self.create_timer(.005,lambda:future.set_result(None) if not future.done() else None)
        try: await future
        finally: self.destroy_timer(timer)

    async def wait(self,future,timeout=.7):
        deadline=time.monotonic()+timeout
        while not future.done():
            if time.monotonic() >= deadline: raise RuntimeError('BIMANUAL_SERVICE_TIMEOUT')
            await self.pause()
        return future.result()

    async def stop(self,lease,release=False):
        requested=self.now()
        if release:
            future=self.lease_client.call_async(Lease.Request(operation='release',
                lease_id=lease.lease_id,epoch=lease.epoch))
        else:
            future=self.control_client.call_async(ControlExecution.Request(operation='hold',
                lease_id=lease.lease_id,epoch=lease.epoch))
        try:
            reply=await self.wait(future)
            if not reply.ok: return False
            deadline=time.monotonic()+self.profile.stop_timeout+.5
            while time.monotonic()<deadline:
                s=self.state
                if s and ('EMERGENCY_STOP' in s.code or s.code == 'STOP_FEEDBACK_TIMEOUT'):
                    return False
                velocities=dict(zip(s.joints.name,s.joints.velocity)) if s else {}
                if (s and not s.stop_failures and seconds(s.feedback_stamp)>=requested and
                        0 <= self.now()-seconds(s.feedback_stamp)<self.cfg['feedback_max_age'] and
                        not s.reference.name and
                        all(n in velocities and math.isfinite(velocities[n]) and
                            abs(velocities[n])<.01 for n in self.names) and
                        (not release or not s.lease_active)):
                    return True
                await self.pause()
        except RuntimeError:
            pass
        s=self.state
        self.event('BIMANUAL_STOP_UNCONFIRMED',json.dumps({
            'state':s.state if s else '', 'code':s.code if s else '',
            'lease_active':s.lease_active if s else None,
            'reference_count':len(s.reference.name) if s else None,
            'max_controlled_velocity':max((abs(v) for n,v in zip(s.joints.name,s.joints.velocity)
                if n in self.names),default=None) if s else None}))
        return False

    async def execute(self,goal):
        mode=self.mode_state
        session=BimanualSession(self.cfg,mode.generation,seconds(mode.changed_at))
        result=TeleopAction.Result(simulated=True)
        lease=pending=renewal=None
        accepted=0
        stopped=True
        canceled=False
        worker_token=0
        awaiting=''
        work_seq=-1
        work_stamp=0.
        last_mode='idle'
        self.worker_result=None
        self.preflight_status=''
        try:
            lease=await self.wait(self.lease_client.call_async(Lease.Request(operation='acquire',
                owner=session.identity,resources=list(self.groups))))
            if not lease.ok or lease.profile_hash != self.profile.digest:
                raise RuntimeError(lease.code if not lease.ok else 'PROFILE_MISMATCH')
            # Service round-trips can span a DDS scheduling gap. Wait for
            # fresh mode, input, feedback and recorder samples before moving.
            startup_deadline=time.monotonic()+.7
            while True:
                if self.mode_state and self.mode_state.generation!=session.generation:
                    raise RuntimeError('BIMANUAL_MODE_GENERATION_CHANGED')
                try: self.check();break
                except RuntimeError:
                    if time.monotonic()>=startup_deadline: raise
                    await self.pause()
            self.event('BIMANUAL_STARTED',json.dumps({'session_id':session.identity,
                'mode_generation':session.generation,'resource_groups':self.groups,
                'collision_scope':self.cfg['collision_scope'],'tool_frame':self.cfg['left_frame'],
                'tool_transform':self.cfg['left_to_tool']}),goal.request.task_id)
            started=time.monotonic()
            next_renewal=started
            pending_at=renewal_at=0.
            while True:
                now=time.monotonic()
                if goal.is_cancel_requested: canceled=True;break
                if now-started >= goal.request.duration: break
                self.check()
                if self.mode_state.generation != session.generation:
                    raise RuntimeError('BIMANUAL_MODE_GENERATION_CHANGED')
                left,right=self.frames['left'],self.frames['right']
                if left.source_id != right.source_id or left.seq != right.seq:
                    # ROS delivers the two sides as separate messages. Wait for
                    # their common receiver event instead of faulting on the
                    # brief interval between callbacks.
                    await self.pause()
                    continue
                session.ingest(left,right,self.now())
                if pending is not None:
                    if pending.done():
                        reply=pending.result()
                        if not reply.accepted: raise RuntimeError(reply.code)
                        accepted+=1
                        pending=None
                    elif now-pending_at>.7: raise RuntimeError('BIMANUAL_SUBMIT_TIMEOUT')
                if renewal is not None:
                    if renewal.done():
                        if not renewal.result().ok: raise RuntimeError('LEASE_RENEW_FAILED')
                        renewal=None
                    elif now-renewal_at>.7: raise RuntimeError('LEASE_RENEW_TIMEOUT')
                if now>=next_renewal and renewal is None:
                    renewal=self.lease_client.call_async(Lease.Request(operation='renew',
                        lease_id=lease.lease_id,epoch=lease.epoch))
                    renewal_at,next_renewal=now,now+.3
                if session.mode != last_mode:
                    if session.mode == 'idle':
                        if pending is not None:
                            reply=await self.wait(pending)
                            if not reply.accepted: raise RuntimeError(reply.code)
                            accepted+=1
                            pending=None
                        if not await self.stop(lease):
                            raise RuntimeError('BIMANUAL_HOLD_UNCONFIRMED')
                        self.worker_result=None
                        awaiting=''
                    last_mode=session.mode
                    self.event('BIMANUAL_CLUTCH',session.mode,goal.request.task_id)
                s=self.state
                if (s.command_id.startswith(session.identity+'_') and s.state=='FAILED'):
                    raise RuntimeError(s.code)
                if self.worker_result is not None:
                    kind,token,value=self.worker_result
                    self.worker_result=None
                    if token==worker_token and session.mode=='follow' and awaiting:
                        if kind=='error': raise RuntimeError(value)
                        if awaiting=='capture':
                            session.capture(value)
                            self.event('BIMANUAL_CAPTURED',json.dumps({'tool_pose':value.tolist(),
                                'input_stamp':session.stamp}),goal.request.task_id)
                        else:
                            if (session.seq == work_seq and
                                    work_stamp >= self.now()-self.cfg['command_max_age'] and pending is None):
                                positions,metrics=value
                                command=MotionCommand(schema_version=2,
                                    command_id=session.identity+'_'+str(token),
                                    task_id=goal.request.task_id,observation_id=str(token),
                                    profile_hash=self.profile.digest,lease_id=lease.lease_id,epoch=lease.epoch,
                                    mode='joint_target',resource_group='',resource_groups=list(self.groups),
                                    stamp=stamp(work_stamp),valid_for=self.cfg['command_max_age'],
                                    joint_names=list(self.names),positions=list(positions))
                                pending=self.submit_client.call_async(SubmitMotion.Request(command=command))
                                pending_at=now
                                self.event('BIMANUAL_QP_RESULT',json.dumps({'solve_us':metrics[0],
                                    'relative_error':metrics[1],'recovery_scale':metrics[2],
                                    'input_stamp':work_stamp}),goal.request.task_id,command.command_id)
                        awaiting=''
                if (session.mode=='follow' and not self.worker.busy and not awaiting and
                        pending is None and (session.seq != work_seq or session.anchor_tool is None)):
                    measured=dict(zip(s.joints.name,s.joints.position))
                    worker_token+=1
                    operation='capture' if session.anchor_tool is None else 'solve'
                    if self.worker.submit(operation,worker_token,measured,session.target):
                        awaiting=operation
                        work_seq,work_stamp=session.seq,session.stamp
                goal.publish_feedback(TeleopAction.Feedback(session_id=session.identity,
                    mode=session.mode,accepted_commands=accepted))
                await self.pause()
            result.code='CANCELED' if canceled else 'TELEOP_SESSION_COMPLETE'
        except (RuntimeError,ValueError,KeyError) as exc:
            result.code=str(exc)
            if result.code=='BIMANUAL_MODE_NOT_SELECTED':
                mode=self.mode_state
                self.event('BIMANUAL_MODE_DIAGNOSTIC',json.dumps({
                    'mode':mode.mode if mode else '', 'phase':mode.phase if mode else '',
                    'generation':mode.generation if mode else None,
                    'age':self.now()-seconds(mode.stamp) if mode else None}),
                    goal.request.task_id)
        finally:
            if lease is not None and lease.ok:
                stopped=await self.stop(lease,release=True)
            if pending is not None:
                try:
                    if (await self.wait(pending)).accepted: accepted+=1
                except RuntimeError:
                    self.event('BIMANUAL_SUBMIT_UNCONFIRMED',session.identity,goal.request.task_id)
            if not stopped: result.code+=':STOP_UNCONFIRMED'
            result.accepted_commands=accepted
            result.success=result.code=='TELEOP_SESSION_COMPLETE'
            self.event('BIMANUAL_ENDED',json.dumps({'session_id':session.identity,
                'code':result.code,'accepted':accepted}),goal.request.task_id)
            self.busy=False
        if result.success: goal.succeed()
        elif canceled and stopped: goal.canceled()
        else: goal.abort()
        return result

    def destroy_node(self):
        self.worker.close()
        return super().destroy_node()


def main():
    spin(BimanualToolNode)
