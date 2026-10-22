"""PICO coordinator over shared position operations or legacy demo actuators.

Use the existing mode/tool Actions and shared execution lease. All motion and
actuator handoffs wait for fresh stop feedback; stale completions cannot rearm.
"""
import json
import math
import time
import uuid
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
from bindu_interfaces.msg import VRInput, ExecutionState, MotionCommand, TeleopModeState, RecorderHealth, RuntimeEvent
from bindu_execution.interpolation import Curve, State
from bindu_interfaces.srv import Lease, SubmitMotion, ControlExecution, PicoContext
from bindu_interfaces.action import SelectTeleopMode, TeleopSession
from .common import RuntimeNode, seconds, stamp, spin, joints_at_rest


class Superseded(RuntimeError):
    pass


def prepare_reference(profile,names,measured):
    """Keep explicit A references legal without rewriting measured feedback."""
    result=dict(measured)
    tolerance=profile.feedback_tolerances['joint_position']
    for n in names:
        q=measured[n];lo,hi=profile.limits[n]
        if not math.isfinite(q) or not lo-tolerance<=q<=hi+tolerance:
            raise RuntimeError('PREPARE_START_JOINT_LIMIT:'+n)
        result[n]=min(hi,max(lo,q))
    return result


class PicoNode(RuntimeNode):
    def __init__(self):
        super().__init__('pico_control', allow_hardware=True, default_mode='simulation')
        self.declare_parameter('recording_enabled',True,descriptor=ParameterDescriptor(read_only=True))
        self.recording_enabled=self.get_parameter('recording_enabled').value
        share=Path(get_package_share_directory('bindu_runtime'))
        self.declare_parameter('pico_config',str(share/'config/robots/snowbot/pico_snowbot_sim.json'),descriptor=ParameterDescriptor(read_only=True))
        self.declare_parameter('tool_config',str(share/'config/robots/snowbot/bimanual_snowbot_shovel.json'),descriptor=ParameterDescriptor(read_only=True))
        self.declare_parameter('model_root',str(Path(get_package_share_directory('bindu_description'))/'urdf'),descriptor=ParameterDescriptor(read_only=True))
        self.declare_parameter('device_backend','kinematic',descriptor=ParameterDescriptor(read_only=True))
        self.declare_parameter('gripper_config','',descriptor=ParameterDescriptor(read_only=True))
        self.cfg=json.loads(Path(self.get_parameter('pico_config').value).read_text())
        self.position_contract = self.cfg.get('schema_version') == 2
        if self.cfg.get('schema_version') not in (1,2):
            raise ValueError('PICO_CONFIG_SCHEMA')
        if self.cfg.get('simulation_only') is not self.simulation or self.cfg.get('mapping')!=PICO_MAPPING:
            raise ValueError('PICO_CONFIG_RUNTIME_MODE_MISMATCH')
        self.backend=self.get_parameter('device_backend').value
        if (not self.position_contract and not self.simulation or
                self.backend not in (('kinematic','external_simulation') if self.simulation and self.position_contract else
                                     ('kinematic',) if self.simulation else ('snowbot_vendor',))):
            raise ValueError('PICO_ACTUATOR_BACKEND_UNAVAILABLE:'+self.backend)
        self.tool_cfg=json.loads(Path(self.get_parameter('tool_config').value).read_text())
        confirmation=self.tool_cfg.get('tool_grasp_confirmation','calibrated')
        if confirmation not in ('calibrated','operator'):
            raise ValueError('PICO_TOOL_GRASP_CONFIRMATION_CONFIG')
        self.operator_grasp=confirmation=='operator'
        self.operator_grasp_positions={}
        if self.operator_grasp and (not self.position_contract or
                self.tool_cfg.get('grasp_reference_source')!='measured_rigid'):
            raise ValueError('PICO_OPERATOR_GRASP_REQUIRES_MEASURED_RIGID')
        if (not self.tool_cfg.get('right_axial_grasp') or self.tool_cfg.get('collision_scope')!='configured_tool_geometry' or
                not self.position_contract and not self.tool_cfg.get('demo',{}).get('initial_positions_rad')):
            raise ValueError('PICO_REQUIRES_AXIAL_SHOVEL_CONFIG')
        self.tool_cfg['urdf']=str(Path(self.get_parameter('model_root').value)/self.tool_cfg['urdf'])
        self.groups=tuple(self.tool_cfg['resource_groups'])+(() if self.position_contract else
            tuple(self.cfg['grippers'][s]['group'] for s in ('left','right')))
        if len(set(self.groups))!=len(self.groups) or any(g not in self.profile.groups for g in self.groups):
            raise ValueError('PICO_RESOURCE_LAYOUT')
        self.names=tuple(n for g in self.groups for n in self.profile.groups[g])
        for side in (() if self.position_contract else ('left','right')):
            item=self.cfg['grippers'][side]
            if tuple(self.profile.groups[item['group']])!=(item['joint'],):raise ValueError('PICO_GRIPPER_LAYOUT')
            targets=[item[k] for k in ('open','closed','virtual')]
            if (any(type(q) not in (float,int) or not math.isfinite(q) or
                    not self.profile.limits[item['joint']][0]<=q<=self.profile.limits[item['joint']][1] for q in targets)
                    or len(set(targets))!=3):raise ValueError('PICO_GRIPPER_TARGETS')
        initial_load=self.cfg.get('initial_load_state','unknown')
        if initial_load not in ('unknown','empty') or (initial_load!='unknown' and not (self.simulation and self.position_contract)):
            raise ValueError('PICO_STARTUP_LOAD_MUST_BE_UNKNOWN')
        if not self.simulation and self.cfg.get('device_initial_positions_rad'):
            raise ValueError('REAL_REQUIRES_MEASURED_INITIAL_STATE')
        self.declare_parameter('simulation_demo',False)
        self.simulation_demo=self.get_parameter('simulation_demo').value
        if type(self.simulation_demo) is not bool or (self.simulation_demo and
                (not self.simulation or self.backend!='kinematic' or self.position_contract)):
            raise ValueError('PICO_DEMO_REQUIRES_KINEMATIC_SIMULATION')
        if self.simulation_demo:
            from bindu_teleoperation.simulation_fixture import short_rod_config
            self.tool_cfg=short_rod_config(self.tool_cfg)
        self.prepare_positions=(self.cfg.get('prepare_positions_rad') if self.position_contract else
            (self.cfg.get('simulation_demo_prepare_positions_rad',
            self.tool_cfg['demo']['initial_positions_rad']) if self.simulation_demo else
            self.tool_cfg['demo']['initial_positions_rad']))
        prepare_names=[n for g in self.tool_cfg['resource_groups'] for n in self.profile.groups[g]]
        if self.prepare_positions is not None and (not set(prepare_names)<=set(self.prepare_positions) or any(
                type(self.prepare_positions[n]) not in (float,int) or not math.isfinite(self.prepare_positions[n]) or
                not self.profile.limits[n][0]<=self.prepare_positions[n]<=self.profile.limits[n][1]
                for n in prepare_names)):
            raise ValueError('PICO_PREPARE_PRESET_INVALID')
        if not 0 < float(self.cfg['prepare_duration']) <= 60:
            raise ValueError('PICO_PREPARE_DURATION_INVALID')
        self.control=PicoControl(initial_load=initial_load,simulation_demo=self.simulation_demo,
                                 max_age=float(self.tool_cfg['input_max_age']),guided_loading=self.position_contract,
                                 dual_modes=self.position_contract,direct_grippers=self.position_contract,
                                 initial_mode=self.cfg.get('initial_teleop_mode','independent_arms'),
                                 pose_jump_translation_m=self.cfg.get('pose_jump_translation_m',.35),
                                 pose_jump_rotation_rad=self.cfg.get('pose_jump_rotation_rad',1.2))
        validation_cfg=dict(self.tool_cfg,worker_timeout=30.)
        recovery=self.cfg.get('prepare_collision_recovery')
        if recovery is not None:
            if not self.simulation or not self.position_contract:
                raise ValueError('PICO_PREPARE_RECOVERY_REQUIRES_SIMULATION_POSITION')
            from bindu_kinematics.tool_geometry import InitialSelfCollisionRecovery
            validation_cfg['prepare_collision_recovery']=InitialSelfCollisionRecovery(recovery).config
        self.validator=BimanualWorker(validation_cfg,self.profile,preload_path_checks=True)
        self.validation_result=None
        self.prepare_recovery=None
        self.frames={};self.state=self.mode=None;self.handoff_complete=False
        self.tool_goal=None;self.tool_result=None;self.tool_phase='inactive'
        self.lease=None;self.pending_intent=None;self.running=None
        self.waiters=set();self.stop_deadline=None;self.release_until=0.;self.recording=None;self.record_future=None
        self.last_status=0.;self.last_phase=None
        self.last_prepare_check=None
        self.last_context_at=self.control.context_at
        self.last_pair_at=None
        self.last_tick_at=None
        self.motion_limit=None;self.motion_limit_stamp=0.
        self.position_ops = None
        if self.position_contract:
            from .pico_position import PositionOperations
            self.position_ops = PositionOperations(self, self.get_parameter('gripper_config').value)
        self.leases=self.create_client(Lease,'execution/lease')
        self.submit=self.create_client(SubmitMotion,'execution/submit')
        self.stop_client=self.create_client(ControlExecution,'execution/control')
        self.ready_client=self.create_client(Trigger,'teleop/tool_ready')
        self.arms_ready_client=self.create_client(Trigger,'teleop/arms_ready') if self.position_contract else None
        self.record_client=self.create_client(SetBool,'recorder/capture') if self.recording_enabled else None
        self.modes=ActionClient(self,SelectTeleopMode,'teleop/select_mode')
        self.tools=ActionClient(self,TeleopSession,'teleop/tool_session')
        self.projected=self.create_publisher(VRInput,'teleop/vr/input',10)
        self.display=self.create_publisher(String,'teleop/pico/state',10)
        self.operator_state=self.create_publisher(String,'teleop/state',10)
        self.create_subscription(VRInput,'teleop/vr/raw',self.on_frame,10)
        self.create_subscription(ExecutionState,'execution/state',self.on_state,10)
        self.create_subscription(RuntimeEvent,'events',self.on_motion_event,20)
        if self.recording_enabled:
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
        if s is not None and s.stop_failures:
            raise RuntimeError('PICO_STOP_UNCONFIRMED:'+s.code)
        if (s is None or s.device_backend!=self.backend or s.simulated != self.simulation or
                not 0<=self.now()-seconds(s.feedback_stamp)<=self.tool_cfg['feedback_max_age'] or
                len(s.joints.name)!=len(s.joints.position) or len(set(s.joints.name))!=len(s.joints.name) or
                not all(math.isfinite(q) for q in s.joints.position)):
            raise RuntimeError('PICO_FEEDBACK_UNAVAILABLE')
        measured=dict(zip(s.joints.name,s.joints.position))
        if not set(self.profile.limits)<=set(measured):raise RuntimeError('PICO_FEEDBACK_LAYOUT')
        return s,measured

    def feedback(self):
        try:
            s,q=self.reading()
            modes=list(self.position_ops.modes(require_stationary=self.control.phase!='following')) if self.position_ops else []
            for side in (() if self.position_ops else ('left','right')):
                cfg=self.cfg['grippers'][side];value=q[cfg['joint']]
                modes.append(next((m for m in ('closed','open','virtual') if abs(value-cfg[m])<.003),'unknown'))
            stopped=((self.handoff_complete or not s.lease_active) and s.state not in ('RUNNING','STOPPING') and not s.reference.name and
                     joints_at_rest(s.joints,self.names,.005) and (not self.position_ops or self.position_ops.stopped()))
            return PicoFeedback(seconds(s.feedback_stamp),stopped,tuple(modes))
        except RuntimeError as exc:
            return PicoFeedback(0.,False,healthy=False,fault=str(exc))

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

    def on_motion_event(self,msg):
        if (msg.run_id!=self.run_id or msg.runtime_mode!=self.runtime_mode or
                msg.task_id!='pico_'+self.instance_id):return
        if msg.state in ('BIMANUAL_CAPTURED','BIMANUAL_ENDED'):
            self.motion_limit=None
        elif msg.state=='BIMANUAL_QP_RESULT':
            try:
                value=json.loads(msg.code)
                self.motion_limit=value.get('geometry_step')
                self.motion_limit_stamp=seconds(msg.stamp)
            except (ValueError,TypeError):pass

    def on_frame(self,msg):
        if not self.matches_mode(msg) or msg.side not in ('left','right'):return
        self.frames[msg.side]=msg
        if set(self.frames)!= {'left','right'}:return
        a,b=self.frames['left'],self.frames['right']
        if a.source_id!=b.source_id or a.seq!=b.seq:return
        now=self.now()
        if self.last_pair_at is not None and now-self.last_pair_at>.1:
            self.event('PICO_INPUT_DELIVERY_GAP',json.dumps(dict(
                interval_s=now-self.last_pair_at,input_age_s=now-seconds(a.stamp),seq=a.seq)))
        self.last_pair_at=now
        pair=tuple(VRFrame(m.source_id,m.seq,seconds(m.stamp),m.side,tuple(m.pose),m.grip,m.trigger,
                          m.init,m.stop,m.valid,m.clutch_seq,m.mapping,m.buttons,tuple(int(v) for v in m.button_edges),m.virtual_grip)
                   for m in (a,b))
        self.control.update_feedback(self.feedback(),self.now())
        previous=(self.control.source,self.control.seq)
        previous_frames=self.control.frames
        previous_reason=self.control.reason
        previous_jump=self.control.last_pose_jump
        self.control.ingest(*pair,self.now())
        if self.control.last_pose_jump is not previous_jump:
            self.event('PICO_POSE_JUMP',json.dumps(self.control.last_pose_jump))
        if self.control.last_prepare_check is not self.last_prepare_check:
            self.last_prepare_check=self.control.last_prepare_check
            self.event('PICO_PREPARE_CHECK',json.dumps(self.last_prepare_check))
        if self.control.reason=='PICO_PAIR_INVALID' and previous_reason!=self.control.reason:
            self.event('PICO_PAIR_DIAGNOSTIC',json.dumps(dict(now=self.now(),frames=[
                dict(stamp=f.stamp,seq=f.seq,side=f.side,source=f.source_id,mapping=f.mapping,
                     valid=f.valid,init=f.init,grip=f.grip,trigger=f.trigger,buttons=f.buttons,
                     edges=f.button_edges,clutch_seq=f.clutch_seq) for f in pair])))
        if self.control.reason=='PICO_TIME_REVERSED' and previous_reason!=self.control.reason:
            self.event('PICO_TIME_DIAGNOSTIC',json.dumps(dict(
                now=self.now(),source=a.source_id,previous_seq=previous[1],input_seq=a.seq,
                frames=[dict(side=new.side,previous_stamp=old.stamp,input_stamp=new.stamp,
                             interval_s=new.stamp-old.stamp)
                        for old,new in zip(previous_frames,pair)])))
        if self.control.source!=a.source_id or self.control.seq!=a.seq or previous==(a.source_id,a.seq):return
        allowed=self.control.phase in ('engaging','following') and self.now()>=self.release_until
        for i,m in enumerate((a,b)):
            hand_allowed=allowed and (self.control.teleop_mode!='independent_arms' or self.control.squeeze_released[i])
            self.projected.publish(VRInput(stamp=m.stamp,source_id=m.source_id+':'+self.instance_id,runtime_mode=m.runtime_mode,
                seq=m.seq,side=m.side,pose=m.pose,grip=m.grip if hand_allowed else 0.,trigger=0. if self.position_contract else m.trigger,
                init=False,stop=self.control.phase=='locked' or not self.handoff_complete,valid=m.valid,clutch_seq=m.clutch_seq,
                mapping=PICO_MAPPING,buttons=0 if self.position_contract else m.buttons,
                button_edges=[0]*8 if self.position_contract else m.button_edges,
                virtual_grip=self.control.virtual))

    async def pause(self):
        future=Future(executor=self.executor);self.waiters.add(future)
        try:await future
        finally:self.waiters.discard(future)

    def current(self,intent):
        if not self.control.pending or self.control.pending.token!=intent.token:
            raise Superseded('PICO_OPERATION_SUPERSEDED')
        self.control.tick(self.now())
        if not self.control.pending or self.control.pending.token!=intent.token:raise Superseded('PICO_OPERATION_SUPERSEDED')

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
            start=prepare_reference(self.profile,self.names,q) if self.position_contract and intent.operation=='prepare' else q
            before=State.rest(tuple(start[n] for n in self.names));after=State.rest(tuple(end))
            curve=Curve.between(0.,duration,before,after)
            if not curve.valid(self.profile,self.names):raise RuntimeError('PICO_MOTION_LIMITS')
            if fit_duration:
                # Fit simulation-demo preparation and small grasp changes to
                # the same v/a/jerk limits. Loading/unloading and the strict
                # preparation flow retain their configured duration.
                # Keep transport reserve beyond the 120 ms reference horizon;
                # fitting the mathematical curve to its exact jerk boundary
                # leaves no budget for timestamp/service scheduling jitter.
                while duration*.75>=.3:
                    candidate=Curve.between(0.,duration*.75,before,after)
                    if not candidate.valid(self.profile,self.names):break
                    duration*=.75;curve=candidate
            self.event('PICO_ACTUATOR_STARTED',json.dumps(dict(operation=intent.operation,
                duration_s=duration,fit_duration=fit_duration,simulated_actuator=self.simulation)))
            started=self.now();command_id=self.instance_id+'_'+str(intent.token)+'_'+uuid.uuid4().hex[:8]
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
                    sample=curve.sample(min(duration,max(0.,now-started)+(0. if self.position_contract else .12)))
                    point=JointTrajectoryPoint(positions=list(sample.q),velocities=list(sample.v),
                        accelerations=list(sample.a),time_from_start=Duration(nanosec=120000000))
                    msg=MotionCommand(schema_version=2,runtime_mode=self.runtime_mode,command_id=command_id+'_'+str(segment),
                        task_id='pico_'+intent.operation,expected_revision=revision,profile_hash=self.profile.digest,lease_id=lease.lease_id,
                        epoch=lease.epoch,mode='joint_target' if self.position_contract else 'joint_reference_segment',resource_groups=list(self.groups),
                        stamp=stamp(now),valid_for=.25,joint_names=list(self.names),
                        positions=list(sample.q) if self.position_contract else [],points=[] if self.position_contract else [point])
                    self.current(intent)
                    response=await self.wait(self.submit.call_async(SubmitMotion.Request(command=msg)))
                    if not response.accepted:raise RuntimeError(response.code)
                    revision=response.revision;segment+=1
                    next_submit=now+.04
                await self.pause()
            state,measured=self.reading()
            errors={n:float(v-measured[n]) for n,v in zip(self.names,end)}
            worst=max(errors,key=lambda n:abs(errors[n]))
            self.event('PICO_ACTUATOR_FEEDBACK_TIMEOUT',json.dumps(dict(
                operation=intent.operation,elapsed_s=self.now()-started,
                feedback_stamp=seconds(state.feedback_stamp),
                target=dict(zip(self.names,end)),measured={n:measured[n] for n in self.names},
                reference=dict(zip(state.reference.name,state.reference.position)),
                error_rad=errors,worst_joint=worst,position_tolerance_rad=.002,
                measured_at_rest=joints_at_rest(state.joints,self.names,.005),
                velocity_tolerance_rad_s=.005,arrival_confirmed=False)))
            raise RuntimeError(f'PICO_ACTUATOR_FEEDBACK_TIMEOUT:{worst}:error_rad={errors[worst]:.6f}')
        finally:
            await self.release_lease()

    async def validate_prepare(self,intent,targets=None,*,allow_recovery=True):
        self.current(intent);s,q=self.reading()
        self.prepare_recovery=None
        if targets is None and self.prepare_positions is None:
            raise RuntimeError('PICO_PREPARE_PRESET_REQUIRED')
        if not joints_at_rest(s.joints,self.names,.005 if self.position_contract else 1e-6):raise RuntimeError('PREPARE_REQUIRES_STATIONARY_START')
        gripper_move=targets is not None
        target=targets if gripper_move else {n:self.prepare_positions[n]
                for g in self.tool_cfg['resource_groups'] for n in self.profile.groups[g]}
        end=time.monotonic()+self.validator.startup_timeout
        while not self.validator.ready or self.validator.busy:
            self.current(intent)
            if time.monotonic()>end:raise RuntimeError('PICO_PREPARE_VALIDATOR_TIMEOUT')
            await self.pause()
        names=[n for g in self.tool_cfg['resource_groups'] for n in self.profile.groups[g]]
        reference=prepare_reference(self.profile,names,q) if self.position_contract and not gripper_move else q
        extra={}
        if reference!=q:
            extra['reference']=reference
            self.event('PICO_PREPARE_BOUNDARY_REFERENCE',json.dumps(dict(
                measured={n:q[n] for n in names if q[n]!=reference[n]},
                reference={n:reference[n] for n in names if q[n]!=reference[n]},
                tolerance_rad=self.profile.feedback_tolerances['joint_position'])))
        self.validation_result=None
        # Explicit A return is independent of load metadata and the captured
        # two-hand grasp. The operator observes the tool; robot CAD, joint
        # limits, fresh stopped feedback and cancellation remain mandatory.
        operation='gripper_path' if gripper_move else 'prepare_path'
        if allow_recovery and operation=='prepare_path' and self.cfg.get('prepare_collision_recovery'):
            operation='prepare_recovery_path'
        self.event('PICO_PREPARE_VALIDATION',json.dumps(dict(operation=operation,load_state=self.control.load,
            load_state_checked=False,tool_present=gripper_move,coupling_checked=False)))
        if not self.validator.submit(operation,intent.token,q,
                                     target if gripper_move else [target[n] for n in names],**extra):
            raise RuntimeError('PICO_PREPARE_VALIDATOR_BUSY')
        end=time.monotonic()+30.
        while self.validation_result is None:
            self.current(intent)
            if time.monotonic()>end:raise RuntimeError('PICO_PREPARE_VALIDATION_TIMEOUT')
            await self.pause()
        kind,token,value=self.validation_result
        if token!=intent.token or kind!='result':raise RuntimeError(str(value))
        _,current=self.reading()
        if any(abs(current[n]-q[n])>.0001 for n in (self.names if gripper_move else names)):raise RuntimeError('PREPARE_STATE_CHANGED')
        self.prepare_recovery=value.get('self_collision_recovery') if isinstance(value,dict) else None
        if self.prepare_recovery:
            self.event('PICO_PREPARE_RECOVERY',json.dumps(self.prepare_recovery))
        return target

    async def confirm_tool(self,intent,mode='bimanual_tool'):
        f=self.feedback()
        if not f.healthy:raise RuntimeError('PICO_FEEDBACK_UNAVAILABLE')
        if not f.stopped:raise RuntimeError('PICO_MODE_REQUIRES_STOPPED')
        if mode=='bimanual_tool' and self.operator_grasp:
            # Explicit tool-mode selection / two-hand engagement confirms the
            # physical grip. Keep raw modes unknown; never invent calibration.
            states={side:self.position_ops.reading(side) for side in ('left','right')}
            self.operator_grasp_positions={side:s.measured_position for side,s in states.items()}
            self.control.virtual=False
            self.event('PICO_TOOL_GRASP_OPERATOR',json.dumps(dict(operation=intent.operation,
                gripper_positions=self.operator_grasp_positions,grasp_sensor_confirmed=False,
                reference_source='measured_rigid')))
        elif mode=='bimanual_tool' and self.position_ops and not self.position_ops.config['calibrated']:
            raise RuntimeError('PICO_TOOL_GRASP_UNCONFIRMED_UNCALIBRATED')
        if mode=='bimanual_tool' and not self.operator_grasp and (f.modes[0]!='closed' or f.modes[1] not in ('closed','virtual')):
            raise RuntimeError('PICO_GRIPPER_CONFIRMATION_REQUIRED')
        client=self.arms_ready_client if mode=='independent_arms' else self.ready_client
        end=time.monotonic()+self.tool_cfg.get('worker_startup_timeout',30.)
        while time.monotonic()<end:
            self.current(intent)
            if client.service_is_ready():
                reply=await self.wait(client.call_async(Trigger.Request()))
                if reply.success:return
                if reply.message not in ('BIMANUAL_WORKER_NOT_READY','BIMANUAL_PREFLIGHT_PENDING'):
                    raise RuntimeError(reply.message)
            await self.pause()
        raise RuntimeError('PICO_TOOL_CONFIRM_TIMEOUT')

    async def select_tool_mode(self,intent,mode=None):
        self.current(intent)
        mode=mode or self.control.teleop_mode
        if self.mode is None:raise RuntimeError('PICO_MODE_AUTHORITY_UNAVAILABLE')
        self.release_until=float('inf')  # Mode selection requires released projected hands.
        goal=SelectTeleopMode.Goal(mode=mode,side='',expected_generation=self.mode.generation,timeout=5.)
        handle=await self.wait(self.modes.send_goal_async(goal))
        if not handle.accepted:raise RuntimeError('PICO_MODE_REJECTED')
        self.current(intent)
        selected=(await self.wait(handle.get_result_async(),6.)).result
        if not selected.success:raise RuntimeError(selected.code)
        self.current(intent)
        # Deliver real fresh input events with a released projected clutch before
        # the new session observes held hands. Never fabricate/refresh an input stamp.
        end=time.monotonic()+1.
        while not (self.mode and self.mode.phase=='READY' and self.mode.mode==mode and
                   self.mode.generation==selected.generation and self.now()-seconds(self.mode.changed_at)>.06):
            self.current(intent)
            if time.monotonic()>end:raise RuntimeError('PICO_MODE_FEEDBACK_TIMEOUT')
            await self.pause()

    async def follow(self,intent):
        self.current(intent)
        self.release_until=self.now()+.1
        future=self.tools.send_goal_async(TeleopSession.Goal(task_id='pico_'+self.instance_id,
            resource_group=self.control.teleop_mode,duration=3600.),feedback_callback=self.tool_feedback)
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
            if self.tool_phase=='follow' or self.tool_phase.startswith('follow:'):return
            await self.pause()
        raise RuntimeError('PICO_FOLLOW_CAPTURE_TIMEOUT')

    def tool_feedback(self,msg):
        self.tool_phase=msg.feedback.mode

    async def perform(self,intent):
        ok=False;reason='';read_only=False;rejected=False;canceled=False;stopped_failure=False
        try:
            await self.cancel_tool()
            self.current(intent)
            read_only=intent.operation in ('prepare','confirm_tool','follow','select_mode')
            if intent.operation=='prepare':
                # A return validates the measured robot path without making a
                # load declaration or preserving the previous coupled grasp.
                if not self.position_contract and self.control.load!='empty':raise RuntimeError('PICO_PREPARE_REQUIRES_EMPTY')
                target=await self.validate_prepare(intent)
                _,measured=self.reading()
                error=max(abs(measured[n]-q) for n,q in target.items())
                if self.position_contract and error<.002 and not self.prepare_recovery:
                    self.event('PICO_PREPARE_ALREADY_REACHED',json.dumps(dict(max_joint_error_rad=error)))
                else:
                    read_only=False
                    recovery=self.prepare_recovery
                    if recovery:
                        escape=recovery['exit_waypoint']
                        distance=max(abs(measured[n]-v) for n,v in escape.items())
                        # Rest-to-rest quintic peak speed is 1.875 * travel / time.
                        duration=max(1.,2.5*distance/recovery['max_joint_speed_rad_s'])
                        await self.move(intent,escape,duration)
                        self.current(intent)
                        target=await self.validate_prepare(intent,allow_recovery=False)
                    await self.move(intent,target,self.cfg['prepare_duration'],fit_duration=self.simulation_demo)
            elif intent.operation=='manual_grippers':
                if (not self.position_ops or self.control.phase!='gripper_control' or
                        not self.control.frames or any(f.grip>=.3 for f in self.control.frames)):
                    raise RuntimeError('GRIPPER_REQUIRES_STOP_AND_RELEASED_SQUEEZE')
                await self.position_ops.move_grippers(intent,direct=True)
            elif intent.operation=='grippers':
                if not self.control.supported or self.control.phase!='loading':raise RuntimeError('PICO_LOADING_REQUIRED')
                if self.position_ops:
                    await self.position_ops.move_grippers(intent)
                else:
                    target={self.cfg['grippers'][side]['joint']:self.cfg['grippers'][side][mode]
                            for side,mode in zip(('left','right'),intent.value)}
                    await self.move(intent,target,self.cfg['gripper_duration'])
            elif intent.operation=='confirm_tool':await self.confirm_tool(intent)
            elif intent.operation=='select_mode':
                self.event('PICO_MODE_CHECK',json.dumps(dict(token=intent.token,
                    current_mode=self.control.teleop_mode,target_mode=intent.value,
                    source_id=self.control.source,seq=self.control.seq)))
                await self.confirm_tool(intent,intent.value)
                await self.select_tool_mode(intent,intent.value)
            elif intent.operation=='switch_grasp':
                if self.position_contract:
                    if self.operator_grasp or not self.position_ops.axial_available:
                        raise RuntimeError('PICO_AXIAL_GRASP_UNAVAILABLE')
                    target=self.position_ops.grasp_target(intent.value)
                    await self.validate_prepare(intent,target)
                    await self.position_ops.switch_grasp(intent)
                    self.current(intent)
                    await self.confirm_tool(intent)
                else:
                    if self.control.load!='held' or self.feedback().modes[0]!='closed':raise RuntimeError('PICO_LEFT_SUPPORT_REQUIRED')
                    cfg=self.cfg['grippers']['right']
                    target={cfg['joint']:cfg['virtual' if intent.value else 'closed']}
                    await self.validate_prepare(intent,target)
                    await self.move(intent,target,self.cfg['gripper_duration'],fit_duration=True)
            elif intent.operation=='follow':
                if self.position_contract:
                    self.event('PICO_ENGAGE_CHECK',json.dumps(dict(token=intent.token,
                        source_id=self.control.source,seq=self.control.seq,load_state=self.control.load,
                        feedback_stamp=self.feedback().stamp,requested_mode=self.control.teleop_mode,
                        mode=self.mode.mode if self.mode else None)))
                    await self.confirm_tool(intent,self.control.teleop_mode)
                    self.current(intent)
                await self.select_tool_mode(intent)
                read_only=False
                await self.follow(intent)
            elif intent.operation!='stop':raise RuntimeError('PICO_OPERATION_UNSUPPORTED')
            ok=True
        except Superseded:
            # A newer fenced intent owns the workflow. Keep the same measured
            # stop barrier; a cleanup failure remains a real operation error.
            reason='PICO_OPERATION_SUPERSEDED'
            try:
                await self.cancel_tool()
                canceled=True
            except Exception as stop_error:
                reason+=':'+type(stop_error).__name__+':'+str(stop_error)
        except Exception as exc:
            reason=type(exc).__name__+':'+str(exc)
            try:
                await self.cancel_tool()
                rejected=read_only
                stopped_failure=(intent.operation=='prepare' and
                    reason.startswith('RuntimeError:PICO_ACTUATOR_FEEDBACK_TIMEOUT:'))
            except Exception as stop_error:reason+=':'+type(stop_error).__name__+':'+str(stop_error)
        if ok and intent.operation=='manual_grippers':
            # The release barrier has just observed fresh device hold. Read it
            # now rather than depending on the coordinator timer's previous
            # feedback sample when handing a held A request back to standby.
            self.control.update_feedback(self.feedback(),self.now())
        if stopped_failure:self.control.update_feedback(self.feedback(),self.now())
        self.control.complete(intent.token,ok,reason,self.now(),rejected=rejected,stopped_failure=stopped_failure)
        if canceled:
            self.event('PICO_OPERATION_CANCELED',json.dumps(dict(token=intent.token,operation=intent.operation,
                outcome='superseded',reason=reason,simulated_actuator=self.simulation)))
        else:
            self.event('PICO_OPERATION_RESULT',json.dumps(dict(token=intent.token,operation=intent.operation,
                success=ok,reason=reason,read_only_rejection=rejected,simulated_actuator=self.simulation)))

    def tick(self):
        tick_at=time.monotonic()
        if self.last_tick_at is not None and tick_at-self.last_tick_at>.1:
            self.event('PICO_COORDINATOR_TICK_GAP',json.dumps(dict(
                interval_s=tick_at-self.last_tick_at,
                input_age_s=self.now()-self.control.stamp if math.isfinite(self.control.stamp) else None,
                phase=self.control.phase,seq=self.control.seq)))
        self.last_tick_at=tick_at
        for f in tuple(self.waiters):
            if not f.done():f.set_result(None)
        try:
            item=self.validator.poll()
            if item is not None:self.validation_result=item
        except RuntimeError as exc:
            if self.control.pending and self.control.pending.operation=='prepare':self.control.lock(str(exc),self.now())
        if self.state and not self.state.lease_active and self.feedback().healthy:
            self.handoff_complete=True
        feedback=self.feedback()
        if self.position_ops and self.control.teleop_mode=='bimanual_tool' and feedback.healthy:
            if (not self.operator_grasp and not self.control.pending and feedback.stopped and
                    self.control.phase in ('checking','standby','tool_standby') and
                    feedback.modes[1] in ('closed','virtual')):
                self.control.virtual=feedback.modes[1]=='virtual'
            if self.operator_grasp and self.control.phase in ('engaging','following') and self.operator_grasp_positions:
                try:
                    for side,position in self.operator_grasp_positions.items():
                        s=self.position_ops.reading(side)
                        if (s.state in ('MOVING','STOPPING') or
                                abs(s.measured_position-position)>self.position_ops.config['position_tolerance']):
                            raise RuntimeError('PICO_GRASP_POSITION_CHANGED')
                except RuntimeError as exc:self.control.lock(str(exc),self.now())
            if (not self.operator_grasp and self.control.phase in ('engaging','following') and
                    feedback.modes!=('closed','virtual' if self.control.virtual else 'closed')):
                self.control.lock('PICO_GRASP_POSITION_CHANGED',self.now())
        self.control.update_feedback(feedback,self.now());self.control.tick(self.now())
        if self.control.context_at!=self.last_context_at:
            self.last_context_at=self.control.context_at
            self.event('PICO_CONTEXT',json.dumps(self.control.snapshot(self.now())))
        if (self.tool_result and self.tool_result.done() and self.control.phase in ('engaging','following')):
            self.control.lock('PICO_TOOL_SESSION_ENDED:'+self.tool_result.result().result.code,self.now())
        for intent in self.control.drain():
            if intent.operation=='record':
                if not self.recording_enabled:
                    self.event('PICO_RECORDING','RECORDING_DISABLED')
                elif self.record_future is None and self.record_client.service_is_ready():
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
            if self.simulation_demo:
                try:
                    _,q=self.reading()
                    ready=self.prepare_positions
                    prepared=all(abs(q[n]-ready[n])<.003 for g in self.tool_cfg['resource_groups'] for n in self.profile.groups[g])
                    self.control.simulation_fixture(prepared,self.now())
                except RuntimeError:
                    pass
            view=dict(self.control.snapshot(self.now()),run_id=self.run_id,profile_hash=self.profile.digest,vr_source_id=self.control.source,
                source_id=(self.control.source+':'+self.instance_id) if self.control.source else None,
                schema=1,device_backend=self.backend,coordinator_id=self.instance_id,stamp=self.now(),simulated=self.simulation,runtime_mode=self.runtime_mode,
                handoff_complete=self.handoff_complete,gripper_modes=list(self.feedback().modes),tool_phase=self.tool_phase,recording=self.recording,recording_enabled=self.recording_enabled,
                motion_limit=self.motion_limit if self.control.phase=='following' and 0<=self.now()-self.motion_limit_stamp<=.25 else None,
                physical_gripper_confirmed=None,actuator_feedback='joint_position',
                capabilities=dict(prepare=self.prepare_positions is not None,
                    independent_arms=self.position_contract,mode_switch=self.position_contract,
                    grippers=bool(self.position_ops.config['calibrated'] or self.position_ops.config.get('allow_uncalibrated_direct', False)) if self.position_ops else True,
                    axial_grasp=not self.operator_grasp and self.position_ops.axial_available if self.position_ops else True),
                tool_grasp_confirmation='operator' if self.operator_grasp else 'calibrated',
                execution_state=self.state.state if self.state is not None else 'unavailable',
                feedback_age_s=self.now()-seconds(self.state.feedback_stamp) if self.state is not None else None)
            raw=tuple(self.frames.values())
            feedback=self.feedback()
            view.update(motion_stopped=feedback.healthy and feedback.stopped,
                tool_ground_policy=self.tool_cfg['demo']['acceptance'].get('ground_policy','required'),
                tool_variant='short_rod' if self.simulation_demo else 'shovel',
                position_scale=self.tool_cfg['position_scale'],rotation_scale=1.,
                mapping_frame='robot_base',control_point='individual_flange' if self.control.teleop_mode=='independent_arms' else
                    'grip_midpoint' if self.simulation_demo or self.tool_cfg.get('vr_tool_mapping')=='hand_line_midpoint' else 'tool_origin',
                orientation_source='individual_wrist' if self.control.teleop_mode=='independent_arms' else
                    'hand_line' if self.simulation_demo or self.tool_cfg.get('vr_tool_mapping')=='hand_line_midpoint' else 'left_wrist',
                model_joints=dict(zip(self.state.joints.name,self.state.joints.position)) if feedback.healthy else None,
                input_max_age_s=self.control.max_age,
                stop_confirmed=(self.control.rest(self.now()) and self.control.pending is None and
                                self.running is None and self.handoff_complete),
                grasp_applied=None if self.control.teleop_mode=='independent_arms' else
                    'right_axial' if self.control.virtual else 'rigid',
                tracking_valid={f.side:bool(f.valid and 0<=self.now()-seconds(f.stamp)<=self.control.max_age) for f in raw})
            view['raw_input_age_s']=max((self.now()-seconds(f.stamp) for f in raw),default=None)
            view['raw_input_valid']=(len(raw)==2 and all(f.valid for f in raw) and
                len({(f.source_id,f.seq) for f in raw})==1)
            self.display.publish(String(data=json.dumps(view)))
            self.operator_state.publish(String(data=json.dumps(view)))
            if self.last_phase!=(self.control.phase,self.control.reason):
                self.last_phase=(self.control.phase,self.control.reason)
                self.event('PICO_STATE',json.dumps(view))

    def destroy_node(self):
        self.validator.close()
        return super().destroy_node()


def main():
    spin(PicoNode,threaded=False)
