"""PICO simulation coordinator. Hardware is deliberately not a simulated fallback.

Use the existing mode/tool Actions and shared execution lease. All motion and
actuator handoffs wait for fresh stop feedback; stale completions cannot rearm.
"""
import json
import math
import time
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionClient
from rclpy.task import Future
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger, SetBool
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from bindu_contracts.teleoperation import VRFrame
from bindu_teleoperation.pico import PicoControl, PicoFeedback, PICO_MAPPING
from bindu_kinematics.bimanual_worker import BimanualWorker
from bindu_interfaces.msg import VRInput, ExecutionState, MotionCommand, TeleopModeState, RecorderHealth
from bindu_execution.interpolation import Curve, State
from bindu_interfaces.srv import Lease, SubmitMotion, ControlExecution, PicoContext
from bindu_interfaces.action import SelectTeleopMode, TeleopSession
from .common import RuntimeNode, seconds, stamp, spin, joints_at_rest


class Superseded(RuntimeError):
    pass


class PicoNode(RuntimeNode):
    def __init__(self):
        super().__init__('pico_control')
        share=Path(get_package_share_directory('bindu_runtime'))
        self.declare_parameter('pico_config',str(share/'config/pico_snowbot_sim.json'),descriptor=ParameterDescriptor(read_only=True))
        self.declare_parameter('tool_config',str(share/'config/bimanual_snowbot_shovel.json'),descriptor=ParameterDescriptor(read_only=True))
        self.declare_parameter('model_root',str(Path(get_package_share_directory('bindu_description'))/'urdf'),descriptor=ParameterDescriptor(read_only=True))
        self.declare_parameter('device_backend','kinematic',descriptor=ParameterDescriptor(read_only=True))
        self.cfg=json.loads(Path(self.get_parameter('pico_config').value).read_text())
        if self.cfg.get('simulation_only') is not True or self.cfg.get('mapping')!=PICO_MAPPING:
            raise ValueError('PICO_REQUIRES_EXPLICIT_SIMULATION_CONFIG')
        self.backend=self.get_parameter('device_backend').value
        if self.backend!='kinematic':
            raise ValueError('PICO_ACTUATOR_BACKEND_UNAVAILABLE:'+self.backend)
        self.tool_cfg=json.loads(Path(self.get_parameter('tool_config').value).read_text())
        if not self.tool_cfg.get('right_axial_grasp') or self.tool_cfg.get('collision_scope')!='configured_tool_geometry' or not self.tool_cfg.get('demo',{}).get('initial_positions_rad'):
            raise ValueError('PICO_REQUIRES_AXIAL_SHOVEL_CONFIG')
        self.tool_cfg['urdf']=str(Path(self.get_parameter('model_root').value)/self.tool_cfg['urdf'])
        self.groups=tuple(self.tool_cfg['resource_groups'])+tuple(self.cfg['grippers'][s]['group'] for s in ('left','right'))
        if len(set(self.groups))!=len(self.groups) or any(g not in self.profile.groups for g in self.groups):
            raise ValueError('PICO_RESOURCE_LAYOUT')
        self.names=tuple(n for g in self.groups for n in self.profile.groups[g])
        for side in ('left','right'):
            item=self.cfg['grippers'][side]
            if tuple(self.profile.groups[item['group']])!=(item['joint'],):raise ValueError('PICO_GRIPPER_LAYOUT')
            targets=[item[k] for k in ('open','closed','virtual')]
            if (any(type(q) not in (float,int) or not math.isfinite(q) or
                    not self.profile.limits[item['joint']][0]<=q<=self.profile.limits[item['joint']][1] for q in targets)
                    or len(set(targets))!=3):raise ValueError('PICO_GRIPPER_TARGETS')
        if self.cfg.get('initial_load_state','unknown')!='unknown':
            raise ValueError('PICO_STARTUP_LOAD_MUST_BE_UNKNOWN')
        self.control=PicoControl(initial_load='unknown',
                                 max_age=float(self.tool_cfg['input_max_age']))
        validation_cfg=dict(self.tool_cfg,worker_timeout=30.)
        self.validator=BimanualWorker(validation_cfg,self.profile,preload_path_checks=True)
        self.validation_result=None
        self.frames={};self.state=self.mode=None;self.handoff_complete=False
        self.tool_goal=None;self.tool_result=None;self.tool_phase='inactive'
        self.lease=None;self.pending_intent=None;self.running=None
        self.waiters=set();self.stop_deadline=None;self.release_until=0.;self.recording=None;self.record_future=None
        self.last_status=0.;self.last_phase=None
        self.leases=self.create_client(Lease,'execution/lease')
        self.submit=self.create_client(SubmitMotion,'execution/submit')
        self.stop_client=self.create_client(ControlExecution,'execution/control')
        self.ready_client=self.create_client(Trigger,'teleop/tool_ready')
        self.record_client=self.create_client(SetBool,'recorder/capture')
        self.modes=ActionClient(self,SelectTeleopMode,'teleop/select_mode')
        self.tools=ActionClient(self,TeleopSession,'teleop/tool_session')
        self.projected=self.create_publisher(VRInput,'teleop/vr/input',10)
        self.display=self.create_publisher(String,'teleop/pico/state',10)
        self.create_subscription(VRInput,'teleop/vr/raw',self.on_frame,10)
        self.create_subscription(ExecutionState,'execution/state',self.on_state,10)
        self.create_subscription(RecorderHealth,'recorder/health',self.on_recording,5)
        self.create_subscription(TeleopModeState,'teleop/mode_state',self.on_mode,
            QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_service(PicoContext,'teleop/pico/context',self.on_context)
        self.create_timer(.005,self.tick)

    def on_state(self,msg):
        self.state=msg if self.matches_mode(msg) and msg.profile_hash==self.profile.digest else None

    def on_recording(self,msg):
        if self.matches_mode(msg) and msg.run_id==self.run_id and 0<=self.now()-seconds(msg.stamp)<.75:
            self.recording=msg.capture_enabled

    def on_mode(self,msg):
        self.mode=msg if self.matches_mode(msg) and msg.profile_hash==self.profile.digest else None

    def reading(self):
        s=self.state
        if (s is None or s.device_backend!=self.backend or not s.simulated or
                not 0<=self.now()-seconds(s.feedback_stamp)<=self.tool_cfg['feedback_max_age'] or
                len(s.joints.name)!=len(s.joints.position) or len(set(s.joints.name))!=len(s.joints.name) or
                not all(math.isfinite(q) for q in s.joints.position) or s.stop_failures):
            raise RuntimeError('PICO_FEEDBACK_UNAVAILABLE')
        measured=dict(zip(s.joints.name,s.joints.position))
        if not set(self.profile.limits)<=set(measured):raise RuntimeError('PICO_FEEDBACK_LAYOUT')
        return s,measured

    def feedback(self):
        try:
            s,q=self.reading()
            modes=[]
            for side in ('left','right'):
                cfg=self.cfg['grippers'][side];value=q[cfg['joint']]
                modes.append(next((m for m in ('closed','open','virtual') if abs(value-cfg[m])<.003),'unknown'))
            stopped=((self.handoff_complete or not s.lease_active) and s.state not in ('RUNNING','STOPPING') and not s.reference.name and
                     joints_at_rest(s.joints,self.names,.005))
            return PicoFeedback(seconds(s.feedback_stamp),stopped,tuple(modes))
        except RuntimeError:
            return PicoFeedback(0.,False,healthy=False)

    def on_context(self,req,reply):
        self.control.update_feedback(self.feedback(),self.now())
        try:
            expected=(self.control.source+':'+self.instance_id) if self.control.source else None
            if req.source_id!=expected:raise ValueError('PICO_COORDINATOR_SOURCE_CHANGED')
            self.control.context(self.control.source,req.expected_generation,req.load_state,req.external_support,self.now())
            reply.success,reply.code=True,'OPERATOR_DECLARATION_ACCEPTED_NOT_SENSOR_PROOF'
            self.event('PICO_CONTEXT',json.dumps(self.control.snapshot()))
        except ValueError as exc:reply.success,reply.code=False,str(exc)
        reply.generation=self.control.generation
        return reply

    def on_frame(self,msg):
        if not self.matches_mode(msg) or msg.side not in ('left','right'):return
        self.frames[msg.side]=msg
        if set(self.frames)!= {'left','right'}:return
        a,b=self.frames['left'],self.frames['right']
        if a.source_id!=b.source_id or a.seq!=b.seq:return
        pair=tuple(VRFrame(m.source_id,m.seq,seconds(m.stamp),m.side,tuple(m.pose),m.grip,m.trigger,
                          m.init,m.stop,m.valid,m.clutch_seq,m.mapping,m.buttons,tuple(int(v) for v in m.button_edges),m.virtual_grip)
                   for m in (a,b))
        self.control.update_feedback(self.feedback(),self.now())
        previous=(self.control.source,self.control.seq)
        previous_reason=self.control.reason
        self.control.ingest(*pair,self.now())
        if self.control.reason=='PICO_PAIR_INVALID' and previous_reason!=self.control.reason:
            self.event('PICO_PAIR_DIAGNOSTIC',json.dumps(dict(now=self.now(),frames=[
                dict(stamp=f.stamp,seq=f.seq,side=f.side,source=f.source_id,mapping=f.mapping,
                     valid=f.valid,init=f.init,grip=f.grip,trigger=f.trigger,buttons=f.buttons,
                     edges=f.button_edges,clutch_seq=f.clutch_seq) for f in pair])))
        if self.control.source!=a.source_id or self.control.seq!=a.seq or previous==(a.source_id,a.seq):return
        allowed=self.control.phase in ('engaging','following') and self.now()>=self.release_until
        for m in (a,b):
            self.projected.publish(VRInput(stamp=m.stamp,source_id=m.source_id+':'+self.instance_id,runtime_mode=m.runtime_mode,
                seq=m.seq,side=m.side,pose=m.pose,grip=m.grip if allowed else 0.,trigger=m.trigger,
                init=False,stop=self.control.phase=='locked' or not self.handoff_complete,valid=m.valid,clutch_seq=m.clutch_seq,
                mapping=PICO_MAPPING,buttons=m.buttons,button_edges=m.button_edges,
                virtual_grip=self.control.virtual))

    async def pause(self):
        future=Future(executor=self.executor);self.waiters.add(future)
        try:await future
        finally:self.waiters.discard(future)

    def current(self,intent):
        if not self.control.pending or self.control.pending.token!=intent.token:
            raise Superseded('PICO_OPERATION_SUPERSEDED')
        self.control.tick(self.now())
        if self.control.pending.token!=intent.token:raise Superseded('PICO_OPERATION_SUPERSEDED')

    async def wait(self,future,timeout=2.):
        end=time.monotonic()+timeout
        while not future.done():
            if time.monotonic()>end:raise RuntimeError('PICO_RPC_TIMEOUT')
            await self.pause()
        return future.result()

    async def stopped(self,requested,timeout=6.,release=False):
        if self.stop_deadline is None:self.stop_deadline=time.monotonic()+timeout
        while time.monotonic()<self.stop_deadline:
            f=self.feedback();s=self.state
            if s and s.stop_failures:raise RuntimeError('PICO_STOP_UNCONFIRMED')
            if f.healthy and f.stopped and f.stamp>=requested and (not release or not s.lease_active):
                self.stop_deadline=None
                return
            await self.pause()
        raise RuntimeError('PICO_STOP_UNCONFIRMED')

    async def cancel_tool(self):
        if self.tool_goal is not None:
            await self.wait(self.tool_goal.cancel_goal_async())
            if self.tool_result is not None:await self.wait(self.tool_result,7.)
            self.tool_goal=self.tool_result=None;self.tool_phase='inactive'
        requested=self.now()
        await self.stopped(requested,release=True)

    async def release_lease(self):
        if self.lease is None:return
        lease=self.lease;self.lease=None
        reply=await self.wait(self.stop_client.call_async(ControlExecution.Request(operation='hold',
            lease_id=lease.lease_id,epoch=lease.epoch)))
        if not reply.ok:raise RuntimeError(reply.code)
        await self.stopped(self.now())
        reply=await self.wait(self.leases.call_async(Lease.Request(operation='release',lease_id=lease.lease_id,epoch=lease.epoch)))
        if not reply.ok:raise RuntimeError(reply.code)
        await self.stopped(self.now(),release=True)

    async def move(self,intent,targets,duration,*,fit_duration=False):
        self.current(intent)
        if not self.feedback().stopped:raise RuntimeError('PICO_MOTION_REQUIRES_STOP')
        lease=await self.wait(self.leases.call_async(Lease.Request(operation='acquire',
            owner=self.instance_id,resources=list(self.groups))))
        if not lease.ok:raise RuntimeError(lease.code)
        self.lease=lease
        try:
            self.current(intent)
            deadline=time.monotonic()+1.
            while not (self.state and self.state.lease_active and self.state.lease_epoch==lease.epoch):
                self.current(intent)
                if time.monotonic()>deadline:raise RuntimeError('PICO_LEASE_FEEDBACK_TIMEOUT')
                await self.pause()
            s,q=self.reading();end=[targets.get(n,q[n]) for n in self.names]
            revision=s.revision;segment=0
            before=State.rest(tuple(q[n] for n in self.names));after=State.rest(tuple(end))
            curve=Curve.between(0.,duration,before,after)
            if not curve.valid(self.profile,self.names):raise RuntimeError('PICO_MOTION_LIMITS')
            if fit_duration:
                # Virtual/closed differs by only 0.02 rad in this simulated
                # gripper. Fit the small move to the same v/a/jerk limits;
                # loading/unloading keeps its configured full-open duration.
                # Keep transport reserve beyond the 120 ms reference horizon;
                # fitting the mathematical curve to its exact jerk boundary
                # leaves no budget for timestamp/service scheduling jitter.
                while duration*.75>=.3:
                    candidate=Curve.between(0.,duration*.75,before,after)
                    if not candidate.valid(self.profile,self.names):break
                    duration*=.75;curve=candidate
            self.event('PICO_ACTUATOR_STARTED',json.dumps(dict(operation=intent.operation,
                duration_s=duration,fit_duration=fit_duration,simulated_actuator=True)))
            started=self.now();command_id=self.instance_id+'_'+str(intent.token)
            deadline=time.monotonic()+duration+6.;renew_at=0.;next_submit=0.
            while time.monotonic()<deadline:
                self.current(intent)
                state,measured=self.reading();now=self.now()
                if state.command_id.startswith(command_id+'_') and state.state=='FAILED':raise RuntimeError(state.code)
                if (now>=started+duration and seconds(state.feedback_stamp)>=started+duration and
                        all(abs(measured[n]-v)<.002 for n,v in zip(self.names,end)) and
                        joints_at_rest(state.joints,self.names,.005)):return
                if time.monotonic()>renew_at:
                    reply=await self.wait(self.leases.call_async(Lease.Request(operation='renew',
                        lease_id=lease.lease_id,epoch=lease.epoch)))
                    if not reply.ok:raise RuntimeError(reply.code)
                    renew_at=time.monotonic()+.2
                if now>=next_submit:
                    sample=curve.sample(min(duration,max(0.,now-started)+.12))
                    point=JointTrajectoryPoint(positions=list(sample.q),velocities=list(sample.v),
                        accelerations=list(sample.a),time_from_start=Duration(nanosec=120000000))
                    msg=MotionCommand(schema_version=2,runtime_mode=self.runtime_mode,command_id=command_id+'_'+str(segment),
                        task_id='pico_'+intent.operation,expected_revision=revision,profile_hash=self.profile.digest,lease_id=lease.lease_id,
                        epoch=lease.epoch,mode='joint_reference_segment',resource_groups=list(self.groups),
                        stamp=stamp(now),valid_for=.25,joint_names=list(self.names),points=[point])
                    self.current(intent)
                    response=await self.wait(self.submit.call_async(SubmitMotion.Request(command=msg)))
                    if not response.accepted:raise RuntimeError(response.code)
                    revision=response.revision;segment+=1
                    next_submit=now+.04
                await self.pause()
            raise RuntimeError('PICO_ACTUATOR_FEEDBACK_TIMEOUT')
        finally:
            await self.release_lease()

    async def validate_prepare(self,intent,targets=None):
        self.current(intent);s,q=self.reading()
        if not joints_at_rest(s.joints,self.names,1e-6):raise RuntimeError('PREPARE_REQUIRES_STATIONARY_START')
        loaded=targets is not None
        target=targets if loaded else {n:self.tool_cfg['demo']['initial_positions_rad'][n]
                for g in self.tool_cfg['resource_groups'] for n in self.profile.groups[g]}
        end=time.monotonic()+self.validator.startup_timeout
        while not self.validator.ready or self.validator.busy:
            self.current(intent)
            if time.monotonic()>end:raise RuntimeError('PICO_PREPARE_VALIDATOR_TIMEOUT')
            await self.pause()
        names=[n for g in self.tool_cfg['resource_groups'] for n in self.profile.groups[g]]
        self.validation_result=None
        if not self.validator.submit('gripper_path' if loaded else 'prepare_path',intent.token,q,
                                     target if loaded else [target[n] for n in names]):
            raise RuntimeError('PICO_PREPARE_VALIDATOR_BUSY')
        end=time.monotonic()+30.
        while self.validation_result is None:
            self.current(intent)
            if time.monotonic()>end:raise RuntimeError('PICO_PREPARE_VALIDATION_TIMEOUT')
            await self.pause()
        kind,token,value=self.validation_result
        if token!=intent.token or kind!='result':raise RuntimeError(str(value))
        _,current=self.reading()
        if any(abs(current[n]-q[n])>.0001 for n in (self.names if loaded else names)):raise RuntimeError('PREPARE_STATE_CHANGED')
        return target

    async def confirm_tool(self,intent):
        f=self.feedback()
        if not f.stopped or f.modes[0]!='closed' or f.modes[1] not in ('closed','virtual'):
            raise RuntimeError('PICO_GRIPPER_CONFIRMATION_REQUIRED')
        end=time.monotonic()+self.tool_cfg.get('worker_startup_timeout',30.)
        while time.monotonic()<end:
            self.current(intent)
            if self.ready_client.service_is_ready():
                reply=await self.wait(self.ready_client.call_async(Trigger.Request()))
                if reply.success:return
                if reply.message not in ('BIMANUAL_WORKER_NOT_READY','BIMANUAL_PREFLIGHT_PENDING'):
                    raise RuntimeError(reply.message)
            await self.pause()
        raise RuntimeError('PICO_TOOL_CONFIRM_TIMEOUT')

    async def follow(self,intent):
        self.current(intent)
        if self.mode is None:raise RuntimeError('PICO_MODE_AUTHORITY_UNAVAILABLE')
        self.release_until=float('inf')  # Mode selection requires released projected hands.
        goal=SelectTeleopMode.Goal(mode='bimanual_tool',side='',expected_generation=self.mode.generation,timeout=5.)
        handle=await self.wait(self.modes.send_goal_async(goal))
        if not handle.accepted:raise RuntimeError('PICO_MODE_REJECTED')
        self.current(intent)
        selected=(await self.wait(handle.get_result_async(),6.)).result
        if not selected.success:raise RuntimeError(selected.code)
        self.current(intent)
        # Deliver real fresh input events with a released projected clutch before
        # the new session observes held hands. Never fabricate/refresh an input stamp.
        end=time.monotonic()+1.
        while not (self.mode and self.mode.phase=='READY' and self.mode.mode=='bimanual_tool' and
                   self.mode.generation==selected.generation and self.now()-seconds(self.mode.changed_at)>.06):
            self.current(intent)
            if time.monotonic()>end:raise RuntimeError('PICO_MODE_FEEDBACK_TIMEOUT')
            await self.pause()
        self.release_until=self.now()+.1
        future=self.tools.send_goal_async(TeleopSession.Goal(task_id='pico_'+self.instance_id,
            resource_group='bimanual_tool',duration=3600.),feedback_callback=self.tool_feedback)
        def fence(f):
            handle=f.result()
            if handle.accepted and (not self.control.pending or self.control.pending.token!=intent.token):
                handle.cancel_goal_async()
        future.add_done_callback(fence)
        self.tool_goal=await self.wait(future)
        if not self.tool_goal.accepted:self.tool_goal=None;raise RuntimeError('PICO_TOOL_SESSION_REJECTED')
        self.tool_result=self.tool_goal.get_result_async()
        self.current(intent)
        end=time.monotonic()+5.
        while time.monotonic()<end:
            self.current(intent)
            if self.tool_result.done():raise RuntimeError(self.tool_result.result().result.code)
            if self.tool_phase.startswith('follow:'):return
            await self.pause()
        raise RuntimeError('PICO_FOLLOW_CAPTURE_TIMEOUT')

    def tool_feedback(self,msg):
        self.tool_phase=msg.feedback.mode

    async def perform(self,intent):
        ok=False;reason=''
        try:
            await self.cancel_tool()
            self.current(intent)
            if intent.operation=='prepare':
                if self.control.load!='empty':raise RuntimeError('PICO_PREPARE_REQUIRES_EMPTY')
                target=await self.validate_prepare(intent)
                await self.move(intent,target,self.cfg['prepare_duration'])
            elif intent.operation=='grippers':
                if not self.control.supported or self.control.phase!='loading':raise RuntimeError('PICO_LOADING_REQUIRED')
                target={self.cfg['grippers'][side]['joint']:self.cfg['grippers'][side][mode]
                        for side,mode in zip(('left','right'),intent.value)}
                await self.move(intent,target,self.cfg['gripper_duration'])
            elif intent.operation=='confirm_tool':await self.confirm_tool(intent)
            elif intent.operation=='switch_grasp':
                if self.control.load!='held' or self.feedback().modes[0]!='closed':raise RuntimeError('PICO_LEFT_SUPPORT_REQUIRED')
                cfg=self.cfg['grippers']['right']
                target={cfg['joint']:cfg['virtual' if intent.value else 'closed']}
                await self.validate_prepare(intent,target)
                await self.move(intent,target,self.cfg['gripper_duration'],fit_duration=True)
            elif intent.operation=='follow':await self.follow(intent)
            elif intent.operation!='stop':raise RuntimeError('PICO_OPERATION_UNSUPPORTED')
            ok=True
        except Exception as exc:
            reason=type(exc).__name__+':'+str(exc)
            try:await self.cancel_tool()
            except Exception as stop_error:reason+=':'+type(stop_error).__name__+':'+str(stop_error)
        self.control.complete(intent.token,ok,reason,self.now())
        self.event('PICO_OPERATION_RESULT',json.dumps(dict(token=intent.token,operation=intent.operation,
            success=ok,reason=reason,simulated_actuator=True)))

    def tick(self):
        for f in tuple(self.waiters):
            if not f.done():f.set_result(None)
        try:
            item=self.validator.poll()
            if item is not None:self.validation_result=item
        except RuntimeError as exc:
            if self.control.pending and self.control.pending.operation=='prepare':self.control.lock(str(exc),self.now())
        if self.state and not self.state.lease_active and self.feedback().healthy:
            self.handoff_complete=True
        self.control.update_feedback(self.feedback(),self.now());self.control.tick(self.now())
        if (self.tool_result and self.tool_result.done() and self.control.phase in ('engaging','following')):
            self.control.lock('PICO_TOOL_SESSION_ENDED:'+self.tool_result.result().result.code,self.now())
        for intent in self.control.drain():
            if intent.operation=='record':
                if self.record_future is None and self.record_client.service_is_ready():
                    self.record_future=self.record_client.call_async(SetBool.Request(data=not self.recording))
                continue
            self.pending_intent=intent
        if self.record_future and self.record_future.done():
            reply=self.record_future.result()
            if reply.success:self.recording=reply.message=='RECORDING_ENABLED'
            self.event('PICO_RECORDING',reply.message);self.record_future=None
        if self.running is not None and self.running.done():self.running=None
        if self.running is None and self.pending_intent is not None:
            intent,self.pending_intent=self.pending_intent,None
            self.running=self.executor.create_task(self.perform(intent))
        if self.now()-self.last_status>=.1:
            self.last_status=self.now()
            view=dict(self.control.snapshot(),run_id=self.run_id,profile_hash=self.profile.digest,vr_source_id=self.control.source,
                source_id=(self.control.source+':'+self.instance_id) if self.control.source else None,
                coordinator_id=self.instance_id,stamp=self.now(),simulated=True,runtime_mode=self.runtime_mode,
                handoff_complete=self.handoff_complete,gripper_modes=list(self.feedback().modes),tool_phase=self.tool_phase,recording=self.recording,
                physical_gripper_confirmed=None,actuator_feedback='simulated_joint_position')
            self.display.publish(String(data=json.dumps(view)))
            if self.last_phase!=(self.control.phase,self.control.reason):
                self.last_phase=(self.control.phase,self.control.reason)
                self.event('PICO_STATE',json.dumps(view))

    def destroy_node(self):
        self.validator.close()
        return super().destroy_node()


def main():
    spin(PicoNode,threaded=False)
