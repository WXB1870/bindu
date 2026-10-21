"""Dual-hand tool teleop through an explicitly selected position executor."""
import json
import math
import time
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.task import Future
from std_srvs.srv import Trigger, SetBool
from rcl_interfaces.msg import ParameterDescriptor
from bindu_contracts.teleoperation import VRFrame, ToolPoseFrame
from bindu_teleoperation.bimanual_session import BimanualSession, ToolPoseSession, AcceptedReference, ToolPause
from bindu_teleoperation.independent_session import IndependentArmsSession
from bindu_kinematics.bimanual_worker import BimanualWorker
from bindu_hardware.snowbot_gripper import validate_config, JOINTS
from bindu_interfaces.action import TeleopSession as TeleopAction
from bindu_interfaces.msg import VRInput, ExecutionState, RecorderHealth, MotionCommand, TeleopModeState, ToolPoseInput
from bindu_interfaces.srv import Lease, SubmitMotion, ControlExecution
from .common import MODE_STATE_MAX_AGE, RuntimeNode, seconds, stamp, spin, joints_at_rest


class BimanualToolNode(RuntimeNode):
    def __init__(self):
        super().__init__('teleop_tool',allow_hardware=True,default_mode='simulation')
        self.declare_parameter('require_recording',True,descriptor=ParameterDescriptor(read_only=True))
        self.require_recording=self.get_parameter('require_recording').value
        share=Path(get_package_share_directory('bindu_runtime'))
        self.declare_parameter('tool_config',str(share/'config/robots/g1/bimanual_g1_sim.json'))
        self.declare_parameter('model_root',str(Path(get_package_share_directory('bindu_description'))/'urdf'))
        self.cfg=json.loads(Path(self.get_parameter('tool_config').value).read_text())
        self.declare_parameter('position_contract',False)
        position_contract=self.get_parameter('position_contract').value
        self.declare_parameter('execution_mode','position',descriptor=ParameterDescriptor(read_only=True))
        from .teleop_deployment import apply_execution_tracking
        self.cfg=apply_execution_tracking(self.cfg,self.get_parameter('execution_mode').value,
                                          real=not self.simulation,position_contract=position_contract)
        self.declare_parameter('gripper_config','',descriptor=ParameterDescriptor(read_only=True))
        gripper_path=self.get_parameter('gripper_config').value
        self.gripper_cfg=validate_config(json.loads(Path(gripper_path).read_text()),self.profile) if position_contract and gripper_path else None
        self.independent_enabled=position_contract
        self.cfg['independent_arms_enabled']=position_contract
        if (self.cfg.get('simulation_only') is not (self.simulation and not position_contract) or
                self.cfg.get('collision_scope') not in ('empty_scene_only','configured_tool_geometry')):
            raise ValueError('BIMANUAL_CONFIG_RUNTIME_MODE_MISMATCH')
        if not self.simulation and self.cfg.get('collision_scope')!='configured_tool_geometry':
            raise ValueError('BIMANUAL_REAL_REQUIRES_CONFIGURED_GEOMETRY')
        urdf=Path(self.cfg['urdf'])
        self.cfg['urdf']=str(urdf if urdf.is_absolute() else
                             Path(self.get_parameter('model_root').value)/urdf)
        self.declare_parameter('input_source','vr')
        self.input_source=self.get_parameter('input_source').value
        if self.input_source not in ('vr','tool_pose'):
            raise ValueError('BIMANUAL_INPUT_SOURCE')
        if self.input_source == 'tool_pose' and not self.cfg.get('input_frame'):
            raise ValueError('BIMANUAL_INPUT_FRAME')
        self.declare_parameter('tool_ground_policy','configured')
        ground_policy=self.get_parameter('tool_ground_policy').value
        if ground_policy not in ('configured','report_only') or (
                ground_policy=='report_only' and (not self.simulation or self.input_source!='vr')):
            raise ValueError('BIMANUAL_GROUND_POLICY_OVERRIDE')
        if ground_policy=='report_only':
            self.cfg['demo']['acceptance']['ground_policy']='report_only'
        self.declare_parameter('simulation_demo',False)
        demo=self.get_parameter('simulation_demo').value
        if type(demo) is not bool or (demo and (not self.simulation or self.input_source!='vr')):
            raise ValueError('PICO_DEMO_REQUIRES_KINEMATIC_SIMULATION')
        if demo:
            from bindu_teleoperation.simulation_fixture import short_rod_config
            self.cfg=short_rod_config(self.cfg)
        if self.cfg.get('integration_max_sample_gap',self.cfg.get('max_integration_interval',.1)) > self.cfg['input_max_age']:
            raise ValueError('BIMANUAL_SAMPLE_GAP_EXCEEDS_INPUT_TIMEOUT')
        self.tool_frame=None
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
        self.preflight_mode=''
        self.preflight_grasp=''
        self.frames={}
        self.state=self.health=self.mode_state=None
        self.busy=False
        self.grasp_continuation=None
        self.pause_control=ToolPause()
        group=ReentrantCallbackGroup()
        self.lease_client=self.create_client(Lease,'execution/lease',callback_group=group)
        self.submit_client=self.create_client(SubmitMotion,'execution/submit',callback_group=group)
        self.control_client=self.create_client(ControlExecution,'execution/control',callback_group=group)
        # Keep paired input ordered without sharing the worker/state timer lock.
        # Otherwise one hand can starve while the other keeps arriving.
        self.create_subscription(VRInput,'teleop/vr/input',self.on_frame,5,
                                 callback_group=MutuallyExclusiveCallbackGroup())
        self.create_subscription(ToolPoseInput,'teleop/tool/input',self.on_tool_frame,5,
                                 callback_group=MutuallyExclusiveCallbackGroup())
        self.create_subscription(ExecutionState,'execution/state',lambda m:self.store_mode_message('state',m),1)
        if self.require_recording:
            self.create_subscription(RecorderHealth,'recorder/health',self.on_health,5)
        self.create_subscription(TeleopModeState,'teleop/mode_state',
            lambda m:self.store_mode_message('mode_state',m),QoSProfile(depth=1,
                reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.waiters=set()
        self.create_timer(.005,self.poll_worker)
        self.create_service(Trigger,'teleop/tool_ready',self.ready)
        if self.independent_enabled:self.create_service(Trigger,'teleop/arms_ready',self.arms_ready)
        self.create_service(SetBool,'teleop/tool_pause',self.request_pause)
        self.server=ActionServer(self,TeleopAction,'teleop/tool_session',
            execute_callback=self.execute,goal_callback=self.goal,
            cancel_callback=lambda _:CancelResponse.ACCEPT,callback_group=group)

    def request_pause(self,request,reply):
        if not self.busy:
            reply.success,reply.message=False,'TOOL_SESSION_INACTIVE'
            return reply
        try:
            self.pause_control.request(request.data,self.now())
        except ValueError as exc:
            reply.success,reply.message=False,str(exc)
            return reply
        reply.success=True
        reply.message='PAUSE_REQUESTED' if request.data else 'RESUME_REQUESTED'
        event = ('BIMANUAL_PAUSED' if self.pause_control.phase == 'paused' else
                 'BIMANUAL_PAUSE_REQUESTED' if request.data else 'BIMANUAL_RESUME_REQUESTED')
        self.event(event, 'STOP_CONFIRMED' if event == 'BIMANUAL_PAUSED' else self.pause_control.phase)
        return reply

    def on_health(self,msg):
        if msg.run_id == self.run_id: self.store_mode_message('health',msg)

    def axial_position_ready(self):
        cfg=self.gripper_cfg
        s=self.state
        if (not cfg or not cfg['calibrated'] or 'virtual' not in cfg['grippers']['right'] or
                s is None or s.profile_hash!=self.profile.digest or s.simulated!=self.simulation or
                not 0<=self.now()-seconds(s.feedback_stamp)<=self.cfg['feedback_max_age']):
            return False
        q=dict(zip(s.joints.name,s.joints.position));v=dict(zip(s.joints.name,s.joints.velocity))
        return all(n in q and n in v and math.isfinite(q[n]) and math.isfinite(v[n]) and
            abs(q[n]-cfg['grippers'][side]['virtual' if side=='right' else 'closed'])<=cfg['position_tolerance'] and
            abs(v[n])<=cfg['velocity_tolerance'] for side,n in JOINTS.items())

    def on_frame(self,msg):
        if self.matches_mode(msg) and msg.side in ('left','right'):
            # Gripper motion is coordinated separately. Accept the projected
            # axial bit only after the configured opening has measured feedback.
            previous=self.frames.get(msg.side)
            edges=tuple(int(v) for v in msg.button_edges)
            edge_changed=bool(previous and previous.source_id==msg.source_id and
                len(edges)==len(previous.button_edges)==8 and edges!=previous.button_edges)
            blocked=not self.simulation and (msg.mapping!='pico_snowbot_v1' or msg.init or
                msg.trigger>=.3 or msg.virtual_grip and not self.axial_position_ready() or bool(msg.buttons) or len(edges) not in (0,8) or
                edge_changed or bool(self.busy and previous and previous.stop))
            self.frames[msg.side]=VRFrame(msg.source_id,msg.seq,seconds(msg.stamp),msg.side,
                tuple(msg.pose),msg.grip,msg.trigger,msg.init,msg.stop or blocked,msg.valid,msg.clutch_seq,
                msg.mapping or "v34",msg.buttons,tuple(int(v) for v in msg.button_edges),msg.virtual_grip)

    def on_tool_frame(self,msg):
        if self.input_source == 'tool_pose' and self.matches_mode(msg):
            self.tool_frame=ToolPoseFrame(msg.source_id,msg.seq,seconds(msg.stamp),
                msg.frame_id,tuple(msg.pose),msg.tight,msg.valid,msg.stop or (not self.simulation and
                    (not msg.tight or bool(self.busy and self.tool_frame and self.tool_frame.stop))))

    def poll_worker(self):
        # Reuse one timer: creating/destroying a ROS timer at every await
        # stalls callback scheduling during sustained input streams.
        for future in tuple(self.waiters):
            if not future.done(): future.set_result(None)
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
        return self.preflight(reply,'bimanual_tool')

    def arms_ready(self,request,reply):
        return self.preflight(reply,'independent_arms')

    def preflight(self,reply,mode):
        reason=self.worker_error or ('' if self.worker.ready else 'BIMANUAL_WORKER_NOT_READY')
        s=self.state
        if not reason and (self.busy or s is None or s.profile_hash!=self.profile.digest or
                s.simulated != self.simulation or not 0<=self.now()-seconds(s.feedback_stamp)<=self.cfg['feedback_max_age']):
            reason='BIMANUAL_PREFLIGHT_FEEDBACK'
        if not reason:
            grasp='right_axial' if mode=='bimanual_tool' and self.axial_position_ready() else 'rigid'
            if self.preflight_mode!=mode or self.preflight_grasp!=grasp:
                if self.worker.busy:
                    reply.success,reply.message=False,'BIMANUAL_PREFLIGHT_PENDING';return reply
                self.preflight_status='';self.preflight_mode=mode;self.preflight_grasp=grasp
            if self.preflight_status=='ready' and self.now()-self.preflight_at<=self.cfg['feedback_max_age']:
                pass
            elif self.preflight_status=='failed' and self.now()-self.preflight_at<=self.cfg['feedback_max_age']:
                reason=self.preflight_error
            elif self.preflight_status=='pending':
                reason='BIMANUAL_PREFLIGHT_PENDING'
            else:
                measured=dict(zip(s.joints.name,s.joints.position))
                self.preflight_token-=1
                operation='independent_capture' if mode=='independent_arms' else 'capture'
                if self.worker.submit(operation,self.preflight_token,measured,grasp_mode=grasp):
                    self.preflight_status='pending'
                    self.preflight_at=self.now()
                reason='BIMANUAL_PREFLIGHT_PENDING'
        reply.success,reply.message=not bool(reason),reason or 'BIMANUAL_READY'
        return reply

    def check(self, require_input=True):
        now=self.now()
        m=self.mode_state
        if (m is None or m.profile_hash != self.profile.digest or
                m.phase != 'READY' or m.mode not in (('bimanual_tool','independent_arms') if self.independent_enabled else ('bimanual_tool',)) or
                not 0 <= now-seconds(m.stamp) < MODE_STATE_MAX_AGE):
            raise RuntimeError('BIMANUAL_MODE_NOT_SELECTED')
        if self.worker_error or not self.worker.ready:
            raise RuntimeError(self.worker_error or 'BIMANUAL_WORKER_NOT_READY')
        s=self.state
        if (s is None or s.profile_hash != self.profile.digest or s.simulated != self.simulation or
                not 0 <= now-seconds(s.feedback_stamp) <= self.cfg['feedback_max_age']):
            raise RuntimeError('BIMANUAL_FEEDBACK_STALE')
        if getattr(self,'require_recording',True) and (self.health is None or not self.health.ready or not 0 <= now-seconds(self.health.stamp)<.75):
            raise RuntimeError('RECORDER_UNAVAILABLE')
        if not require_input:
            if self.tool_frame is not None and self.tool_frame.stop and self.input_source == 'tool_pose':
                raise RuntimeError('TOOL_INPUT_STOP')
            if self.input_source == 'vr' and any(f.stop for f in self.frames.values()):
                raise RuntimeError('VR_STOP')
            return
        if self.input_source == 'tool_pose':
            f=self.tool_frame
            if (f is None or not f.valid or f.stop or f.stamp<=seconds(m.changed_at) or
                    not 0<=now-f.stamp<=self.cfg['input_max_age']):
                raise RuntimeError('TOOL_INPUT_TIMEOUT')
        elif (any(side not in self.frames or self.frames[side].stamp <= seconds(m.changed_at) or
                not 0 <= now-self.frames[side].stamp <= self.cfg['input_max_age']
                for side in ('left','right'))):
            raise RuntimeError('VR_PAIR_TIMEOUT')

    def goal(self,request):
        try: self.check()
        except RuntimeError as exc:
            self.event('BIMANUAL_GOAL_REJECTED',str(exc),request.task_id)
            return GoalResponse.REJECT
        if (self.busy or not request.task_id or request.resource_group != self.mode_state.mode or
                not math.isfinite(request.duration) or not 0 < request.duration <= 3600 or
                not all(c.service_is_ready() for c in
                        (self.lease_client,self.submit_client,self.control_client))):
            return GoalResponse.REJECT
        self.pause_control=ToolPause()
        self.busy=True
        return GoalResponse.ACCEPT

    async def pause(self):
        future=Future(executor=self.executor)
        self.waiters.add(future)
        try: await future
        finally: self.waiters.discard(future)

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
                if (s and s.state not in ('RUNNING','STOPPING') and not s.stop_failures and seconds(s.feedback_stamp)>=requested and
                        0 <= self.now()-seconds(s.feedback_stamp)<self.cfg['feedback_max_age'] and
                        not s.reference.name and
                        joints_at_rest(s.joints, self.names, .005) and
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
        session_type=(IndependentArmsSession if mode.mode=='independent_arms' else
                      ToolPoseSession if self.input_source=='tool_pose' else BimanualSession)
        continuation=self.grasp_continuation
        self.grasp_continuation=None
        session=session_type(self.cfg,mode.generation,seconds(mode.changed_at),
            **({'continuation':continuation} if self.input_source=='vr' else {}))
        reference=AcceptedReference(self.cfg.get('max_integration_interval',.1),
            max_sample_gap=self.cfg.get('integration_max_sample_gap'))
        result=TeleopAction.Result(simulated=self.simulation)
        lease=pending=renewal=None
        accepted=0
        stopped=True
        canceled=False
        worker_token=0
        awaiting=''
        work_seq=-1
        work_stamp=0.
        work_sampled_at=0.
        work_positions=None
        work_target=None
        work_dt=None
        work_gap=None
        work_feedback=work_reference=None
        pending_reference=None
        last_mode='idle'
        last_grasp_generation=0
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
                'teleop_mode':mode.mode,
                'mode_generation':session.generation,'resource_groups':self.groups,
                'collision_scope':'robot_body_only' if isinstance(session,IndependentArmsSession) else self.cfg['collision_scope'],
                'tool_frame':self.cfg['left_frame'],
                'tool_transform':None if isinstance(session,IndependentArmsSession) else self.cfg['left_to_tool'],
                'position_scale':self.cfg['position_scale'],'rotation_scale':1.,
                'vr_tool_mapping':self.cfg.get('vr_tool_mapping','legacy_wrist'),
                'pose_filter':self.cfg.get('pose_filter') if self.input_source=='vr' else None,
                'pose_filter_space':('per_hand_robot_pose' if isinstance(session,IndependentArmsSession)
                    else 'paired_controller_pose') if self.input_source=='vr' else None,
                'mapping_frame':'robot_base','geometry_kind':self.cfg.get('demo',{}).get('geometry_kind','shovel'),
                'max_integration_interval':reference.max_interval,'max_sample_gap':reference.max_sample_gap,
                'right_axial_grasp':bool(session.axial),
                'grip_actuation':'kinematic_constraint_only'}),goal.request.task_id)
            started=time.monotonic()
            next_renewal=started
            pending_at=renewal_at=0.
            while True:
                now=time.monotonic()
                if goal.is_cancel_requested: canceled=True;break
                if now-started >= goal.request.duration: break
                self.check(require_input=self.pause_control.phase=='follow')
                if self.mode_state.generation != session.generation:
                    raise RuntimeError('BIMANUAL_MODE_GENERATION_CHANGED')
                if pending is not None:
                    if pending.done():
                        reply=pending.result()
                        if not reply.accepted: raise RuntimeError(reply.code)
                        reference.accept(*pending_reference)
                        self.event('BIMANUAL_SUBMIT_TIMING',json.dumps({
                            'input_seq':work_seq,'reference_sampled_at':reference.sampled_at,
                            'acknowledged_at':self.now(),'submit_round_trip_s':now-pending_at,
                            'server_monotonic_s':now}),goal.request.task_id)
                        pending_reference=None
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
                if self.pause_control.phase == 'pausing':
                    if pending is not None:
                        reply=await self.wait(pending)
                        if not reply.accepted: raise RuntimeError(reply.code)
                        accepted+=1
                        pending=None
                    if not await self.stop(lease):
                        raise RuntimeError('BIMANUAL_PAUSE_STOP_UNCONFIRMED')
                    worker_token+=1
                    self.worker_result=None
                    awaiting=''
                    reference.reset()
                    pending_reference=None
                    session.reanchor()
                    self.pause_control.confirm_stopped()
                    self.event('BIMANUAL_PAUSED','STOP_CONFIRMED',goal.request.task_id)
                if self.pause_control.phase == 'paused':
                    goal.publish_feedback(TeleopAction.Feedback(session_id=session.identity,
                        mode='paused:'+session.grasp_mode,accepted_commands=accepted))
                    await self.pause()
                    continue
                if self.pause_control.phase == 'waiting_input':
                    frame_stamp=(self.tool_frame.stamp if self.tool_frame else float('-inf')) if self.input_source=='tool_pose' else min(
                        (f.stamp for f in self.frames.values()),default=float('-inf'))
                    frame_seq=(self.tool_frame.seq if self.tool_frame else -1) if self.input_source=='tool_pose' else min(
                        (f.seq for f in self.frames.values()),default=-1)
                    if frame_seq <= session.seq or not self.pause_control.accepts(frame_stamp):
                        if self.now()-self.pause_control.resume_after > self.cfg['input_max_age']:
                            raise RuntimeError('TOOL_RESUME_INPUT_TIMEOUT')
                        await self.pause()
                        continue
                    self.check()
                    self.event('BIMANUAL_RESUMED','FRESH_INPUT',goal.request.task_id)
                if self.input_source == 'tool_pose':
                    session.ingest(self.tool_frame,self.now())
                else:
                    left,right=self.frames['left'],self.frames['right']
                    if left.source_id != right.source_id or left.seq != right.seq:
                        await self.pause()
                        continue
                    session.ingest(left,right,self.now())
                if session.grasp_generation != last_grasp_generation:
                    # Finish accepted submissions, stop physically, then fence
                    # every old worker result and rebuild both VR/QP anchors.
                    if pending is not None:
                        reply=await self.wait(pending)
                        if not reply.accepted: raise RuntimeError(reply.code)
                        accepted+=1
                        pending=None
                    if not await self.stop(lease):
                        raise RuntimeError('BIMANUAL_GRASP_STOP_UNCONFIRMED')
                    worker_token+=1
                    self.worker_result=None
                    awaiting=''
                    reference.reset()
                    pending_reference=None
                    if isinstance(session,IndependentArmsSession):session.reanchor(preserve_active=True)
                    else:session.reanchor()
                    last_grasp_generation=session.grasp_generation
                    self.event('BIMANUAL_GRASP_REQUESTED',json.dumps({
                        'mode':session.grasp_mode,'generation':last_grasp_generation,
                        'simulated':self.simulation,'physical_gripper_command':False}),goal.request.task_id)
                    continue
                if session.mode != last_mode:
                    if session.mode == 'idle':
                        if pending is not None:
                            reply=await self.wait(pending)
                            if not reply.accepted: raise RuntimeError(reply.code)
                            accepted+=1
                            pending=None
                        if not await self.stop(lease):
                            raise RuntimeError('BIMANUAL_HOLD_UNCONFIRMED')
                        worker_token+=1
                        self.worker_result=None
                        awaiting=''
                        reference.reset()
                        pending_reference=None
                    last_mode=session.mode
                    self.event('BIMANUAL_CLUTCH',session.mode,goal.request.task_id)
                s=self.state
                if (s.command_id.startswith(session.identity+'_') and s.state=='FAILED'):
                    raise RuntimeError(s.code)
                if self.worker_result is not None:
                    kind,token,value=self.worker_result
                    self.worker_result=None
                    if token==worker_token and session.mode=='follow' and awaiting:
                        geometry=(self.worker.diagnostic[1] if self.worker.diagnostic and
                                  self.worker.diagnostic[0]==token else None)
                        if geometry is not None and geometry['scale']<1.:
                            self.event('BIMANUAL_GEOMETRY_STEP',json.dumps(dict(geometry,
                                input_seq=work_seq,control_sampled_at=work_sampled_at)),goal.request.task_id)
                        if kind=='error':
                            self.event('BIMANUAL_WORK_REJECTED',json.dumps({
                                'code':value,'operation':awaiting,'input_seq':work_seq,'input_stamp':work_stamp,
                                'control_sampled_at':work_sampled_at,'integration_dt':work_dt,'sample_gap_s':work_gap,
                                'grasp_mode':session.grasp_mode,'feedback':work_feedback,
                                'joint_names':self.names,'reference':work_reference,'target':work_target}),goal.request.task_id)
                            raise RuntimeError(value)
                        if awaiting=='capture':
                            if not 0 <= self.now()-work_sampled_at <= reference.max_sample_gap:
                                raise RuntimeError('BIMANUAL_CAPTURE_STALE')
                            session.capture(value)
                            # No motion was submitted while capturing. Start the
                            # integration clock here; capture latency is not dt.
                            reference.capture(work_positions,self.now())
                            self.event('BIMANUAL_CAPTURED',json.dumps({'tool_pose':value.tolist(),
                                'feedback':work_feedback,
                                'input_stamp':session.stamp,'input_seq':session.seq,'grasp_mode':session.grasp_mode,
                                'controller_pose':session.anchor_input.tolist(),
                                'raw_controller_pose':session.raw_input.tolist() if self.input_source=='vr' else None,
                                'pose_filter':self.cfg.get('pose_filter') if self.input_source=='vr' else None,
                                'pose_filter_state':([hand.pose_filter.snapshot() if hand.pose_filter else None
                                    for hand in session.hands] if isinstance(session,IndependentArmsSession) else
                                    session.pose_filter.snapshot() if isinstance(session,BimanualSession) and session.pose_filter else None),
                                'controller_anchors':([value.tolist() if value is not None else None
                                    for value in session.anchors] if isinstance(session,IndependentArmsSession) else None),
                                'robot_anchors':([value.tolist() if value is not None else None
                                    for value in session.robot_anchors] if isinstance(session,IndependentArmsSession) else None),
                                'control_point_in_tool_m':session.control_point_in_tool.tolist() if isinstance(session,BimanualSession) else None,
                                'feedback_sampled_at':work_sampled_at,'capture_completed_at':reference.sampled_at,
                                'input_after_capture':reference.input_after_capture,
                                'grasp_generation':session.grasp_generation,
                                'grasp_continuation':getattr(session,'resumed_grasp',False)}),goal.request.task_id)
                        else:
                            # A newer VR frame does not invalidate a bounded,
                            # same-grasp solve. Clutch/mode changes clear awaiting.
                            # Discarded results never advance the accepted reference.
                            if (work_stamp >= self.now()-self.cfg['command_max_age'] and pending is None):
                                if not 0 <= self.now()-work_sampled_at <= reference.max_interval:
                                    raise RuntimeError('BIMANUAL_RESULT_STALE')
                                positions,metrics=value
                                command=MotionCommand(schema_version=2, runtime_mode=self.runtime_mode,
                                    command_id=session.identity+'_'+str(token),
                                    task_id=goal.request.task_id,observation_id=str(token),
                                    profile_hash=self.profile.digest,lease_id=lease.lease_id,epoch=lease.epoch,
                                    mode='joint_target',resource_group='',resource_groups=list(self.groups),
                                    stamp=stamp(work_stamp),valid_for=self.cfg['command_max_age'],
                                    joint_names=list(self.names),positions=list(positions))
                                pending=self.submit_client.call_async(SubmitMotion.Request(command=command))
                                pending_reference=(positions,work_sampled_at)
                                pending_at=now
                                self.event('BIMANUAL_QP_RESULT',json.dumps({'solve_us':metrics[0],
                                    'timing':geometry.get('timing') if geometry else None,
                                    'geometry_step':geometry if geometry and geometry['scale']<1. else None,
                                    'relative_error':None if isinstance(session,IndependentArmsSession) else metrics[1],
                                    'relative_error_source':('accepted_reference' if self.cfg.get('reference_kinematics') else 'measured'),
                                    'geometry_step_scale':geometry['scale'] if geometry else 1.,
                                    'teleop_mode':mode.mode,'recovery_scale':metrics[2],
                                    'input_stamp':work_stamp,'input_seq':work_seq,'integration_dt':work_dt,
                                    'sample_gap_s':work_gap,
                                    'control_sampled_at':work_sampled_at,'result_received_at':self.now(),
                                    'result_age_s':self.now()-work_sampled_at,
                                    'target':work_target}),
                                    goal.request.task_id,command.command_id)
                        awaiting=''
                if (session.mode=='follow' and session.anchor_input is not None and
                        not self.worker.busy and not awaiting and
                        pending is None and (session.seq != work_seq or session.anchor_tool is None)):
                    measured=dict(zip(s.joints.name,s.joints.position))
                    worker_token+=1
                    operation='capture' if session.anchor_tool is None else 'solve'
                    sampled_at=self.now()
                    # A capture and this loop can share a clock tick. The next
                    # fresh controller frame will provide a positive interval.
                    if operation=='solve' and sampled_at<=reference.sampled_at:
                        await self.pause()
                        continue
                    if operation=='solve' and not reference.permits_input(session.stamp):
                        # Stop raises the executor's timestamp fence. A frame
                        # already queued during FK capture must not cross it.
                        # Wait for real input; never relabel its timestamp.
                        await self.pause()
                        continue
                    gap=sampled_at-reference.sampled_at if operation=='solve' else None
                    if gap is not None and gap>.08:
                        self.event('BIMANUAL_SAMPLE_GAP',json.dumps({
                            'sample_gap_s':gap,'max_sample_gap_s':reference.max_sample_gap,
                            'max_integration_interval_s':reference.max_interval,
                            'reference_sampled_at':reference.sampled_at,'control_sampled_at':sampled_at,
                            'server_monotonic_s':time.monotonic(),'input_seq':session.seq,
                            'input_stamp':session.stamp,'input_age_s':sampled_at-session.stamp,
                            'feedback_stamp':seconds(s.feedback_stamp),'feedback_age_s':sampled_at-seconds(s.feedback_stamp),
                            'will_reject':gap>reference.max_sample_gap}),goal.request.task_id)
                    baseline,dt=reference.step(sampled_at) if operation=='solve' else (None,None)
                    if self.worker.submit(operation,worker_token,measured,session.target,
                                          reference=baseline,dt=dt,grasp_mode=session.grasp_mode,
                                          target_stamp=session.stamp):
                        awaiting=operation
                        work_seq,work_stamp=session.seq,session.stamp
                        work_sampled_at,work_dt=sampled_at,dt
                        work_gap=gap
                        work_positions=tuple(measured[n] for n in self.names)
                        work_feedback,work_reference=measured,baseline
                        work_target=None if session.target is None else session.target.ravel().tolist()
                goal.publish_feedback(TeleopAction.Feedback(session_id=session.identity,
                    mode=(('capturing' if session.mode=='follow' and session.anchor_tool is None else session.mode)
                          +(':'+session.grasp_mode if session.axial or isinstance(session,IndependentArmsSession) else '')),
                    accepted_commands=accepted))
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
            if canceled and stopped and result.code=='CANCELED' and isinstance(session,BimanualSession):
                self.grasp_continuation=session.stopped_continuation(self.now())
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
    # All service waits yield and numerical work lives in a separate process.
    # Serialize callbacks like ExecutionNode: multiple executor threads can
    # starve fresh state/input callbacks behind the high-frequency polling timer.
    spin(BimanualToolNode, threaded=False)
