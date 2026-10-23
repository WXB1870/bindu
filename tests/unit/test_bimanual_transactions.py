"""Run the production execute coroutine with actual sessions/reference/executor.

Only ROS messages/services, callback scheduling and validated solver candidates
are replaced by in-memory transport. This does not validate QP math, DDS or a
physical device. No ROS module is imported or publisher constructed.
"""
import ast
import asyncio
from dataclasses import replace
import json
from pathlib import Path
from types import SimpleNamespace as S
import unittest
from unittest.mock import Mock

import numpy as np
from bindu_contracts.contracts import Motion
from bindu_contracts.profile import Profile
from bindu_contracts.teleoperation import VRFrame
from bindu_execution.executor import Executor, Rejected
from bindu_contracts.runtime_mode import command_mode
from bindu_hardware.drivers.simulated_joints import SimJointDriver
from bindu_hardware.drivers.simulated_base import SimBaseDriver
from bindu_hardware.robot_io.system import RobotIO
from bindu_teleoperation.bimanual_session import AcceptedReference, BimanualSession, ToolPoseSession, ToolPause
from bindu_teleoperation.independent_session import IndependentArmsSession
from bindu_runtime.stop_evidence import stop_feedback_matches


SOURCE = Path(__file__).resolve().parents[2]/'src/integration/bindu_runtime/bindu_runtime/bimanual_tool_node.py'


class Rig:
    def __init__(self, *, ack_delay=.02, reject=False, drop=False, input_gap=False,
                 transition=None, delivery_gap=None, send_failure=False, state_delay=0.,
                 duplicate_window=0., solve_errors=(),control_step=.02,steady_step=.02,
                 after_error=None,candidate_step=.001):
        self.clock = 10.
        self.steady=10.
        self.control_step,self.steady_step=control_step,steady_step
        self.after_error,self.error_injected=after_error,False
        self.candidate_step=candidate_step
        self.step = 0
        self.ack_delay, self.reject, self.drop = ack_delay, reject, drop
        self.input_gap, self.transition = input_gap, transition
        self.delivery_gap=delivery_gap
        self.send_failure=send_failure
        self.state_delay,self.state_history=state_delay,[]
        self.duplicate_window,self.frozen_frames=duplicate_window,None
        self.solve_errors=iter(solve_errors)
        self.solve_attempts=[]
        self.clutch=0
        self.futures=[]
        self.transitioned = False
        self.receipts, self.commands, self.references, self.events = [], [], [], []
        self.releases = 0
        self.profile = Profile('memory', {'waist':['w'],'left_arm':['l'],'right_arm':['r']},
            {n:(-1.,1.) for n in ('w','l','r')}, .5, ('joint_position',), 'memory',stop_timeout=.3)
        drivers = {g:SimJointDriver(names,.5) for g,names in self.profile.groups.items()}
        self.io = RobotIO(self.profile,drivers,SimBaseDriver())
        self.engine = Executor(self.profile,self.io,steady=lambda:self.steady)
        # Use the actual service handlers as well as the actual executor.
        source=Path(__file__).resolve().parents[2]/'src/integration/bindu_runtime/bindu_runtime/execution_node.py'
        definition=next(n for n in ast.parse(source.read_text()).body
                        if isinstance(n,ast.ClassDef) and n.name=='ExecutionNode')
        handlers=[n for n in definition.body if getattr(n,'name','') in ('lease','control','accept','submit')]
        service_namespace=dict(Motion=Motion,Rejected=Rejected,command_mode=command_mode,
                               seconds=lambda x:x,stamp=lambda x:x)
        module=ast.Module(body=[ast.ClassDef(name='Services',bases=[],keywords=[],
                         body=handlers,decorator_list=[])],type_ignores=[])
        exec(compile(ast.fix_missing_locations(module),str(source),'exec'),service_namespace)
        self.services=services=service_namespace['Services']()
        services.engine,services.robot_io,services.profile=self.engine,self.io,self.profile
        services.now=lambda:self.clock
        services.runtime_mode,services.closing='simulation',False
        services.accepted_pub=S(publish=Mock())
        services.event=lambda *args:None
        rig = self
        class Reference(AcceptedReference):
            def __init__(self,*args,**kwargs):
                super().__init__(*args,**kwargs)
                self.commits = []
                self.captures = []
                rig.references.append(self)
            def capture(self,*args):
                super().capture(*args)
                self.captures.append((self.positions,self.sampled_at))
            def accept(self,*args):
                super().accept(*args)
                self.commits.append((rig.clock,self.positions,self.sampled_at))
        tree = ast.parse(SOURCE.read_text())
        node = next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='BimanualToolNode')
        names = {'execute','goal','check','startup_input_ready','wait','begin_stop','stop','stop_pending','execution_revision','cleanup_event'}
        methods = [n for n in node.body if getattr(n,'name','') in names]
        namespace = dict(time=S(monotonic=lambda:self.steady),json=json,math=__import__('math'),
            stop_feedback_matches=stop_feedback_matches,
            WORKER_POLL_PERIOD=next(ast.literal_eval(n.value) for n in tree.body
                if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and
                    t.id=='WORKER_POLL_PERIOD' for t in n.targets)),
            seconds=lambda x:x,stamp=lambda x:x,MODE_STATE_MAX_AGE=.5,
            AcceptedReference=Reference,BimanualSession=BimanualSession,
            ToolPoseSession=ToolPoseSession,IndependentArmsSession=IndependentArmsSession,ToolPause=ToolPause,
            TeleopAction=S(Result=lambda **kw:S(**kw),Feedback=lambda **kw:S(**kw)),
            GoalResponse=S(ACCEPT='ACCEPT',REJECT='REJECT'),
            MotionCommand=self.motion_command,SubmitMotion=S(Request=lambda **kw:S(**kw)),
            Lease=S(Request=lambda **kw:S(**kw)),ControlExecution=S(Request=lambda **kw:S(**kw)),
            joints_at_rest=lambda joints,names,tol:all(abs(v)<=tol for v in joints.velocity))
        module = ast.Module(body=[ast.ClassDef(name='Production',bases=[],keywords=[],
            body=methods,decorator_list=[])],type_ignores=[])
        exec(compile(ast.fix_missing_locations(module),str(SOURCE),'exec'),namespace)
        self.node = node = namespace['Production']()
        node.profile,node.simulation,node.runtime_mode=self.profile,True,'simulation'
        node.groups,node.names=tuple(self.profile.groups),('w','l','r')
        node.cfg=dict(input_max_age=.25,feedback_max_age=.2,command_max_age=.2,
            engage_seconds=.02,position_scale=1.,operator_yaw_rad=0.,max_integration_interval=.1,
            integration_max_sample_gap=.25,collision_scope='configured_tool_geometry',
            left_frame='left',left_to_tool=np.eye(4).ravel().tolist())
        node.input_source,node.require_recording='vr',False
        node.independent_enabled=True
        node.trajectory_reference=False
        node.worker_error=''
        node.worker=S(ready=True,busy=False,diagnostic=None,submit=self.solve)
        node.frames={}
        node.mode_state=S(mode='bimanual_tool',phase='READY',generation=1,
            stamp=self.clock,changed_at=9.,profile_hash=self.profile.digest)
        node.grasp_continuation=None
        node.pause_control=ToolPause()
        node.busy=True
        node.stop_unconfirmed=False
        node.execution_instance_id=''
        node.event=lambda *args:self.events.append(args)
        node.now=lambda:self.clock
        node.pause=self.pause
        node.lease_client=S(call_async=self.lease)
        node.control_client=S(call_async=self.hold)
        node.submit_client=S(call_async=self.submit)
        self.goal=S(is_cancel_requested=False,request=S(task_id='transaction',duration=.9,
            resource_group='bimanual_tool'),publish_feedback=Mock(),succeed=Mock(),canceled=Mock(),abort=Mock())
        for client in (node.lease_client,node.control_client,node.submit_client):
            client.service_is_ready=lambda:True
        self.update()

    def frame(self,seq,grip,*,source='headset',stamp=None,clutch=0):
        poses=[np.eye(4),np.eye(4)];poses[1][1,3]=.4
        return {side:VRFrame(source,seq,self.clock if stamp is None else stamp,side,
            tuple(poses[i].ravel()),grip,clutch_seq=clutch)
            for i,side in enumerate(('left','right'))}

    def update(self):
        self.engine.tick(self.clock)
        fb=self.engine.feedback
        event=self.engine.events[-1] if self.engine.events else S(state='IDLE',code='',command_id='')
        if self.engine.stopping:
            motion,code=self.engine.stopping
            event=S(state='STOPPING',code=code,command_id=motion.command_id if motion else '')
        elif self.engine.active:
            event=S(state='RUNNING',code='OK',command_id=self.engine.active.command_id)
        state=S(instance_id='executor',profile_hash=self.profile.digest,simulated=True,
            feedback_stamp=fb.stamp,lease_epoch=self.engine.epoch,lease_active=bool(self.engine.lease_id),
            revision=self.engine.revision,state=event.state,code=event.code,command_id=event.command_id,
            reference=S(name=list(self.engine.reference_names)),stop_failures=list(self.io.stop_failures),
            joints=S(name=list(fb.positions),position=list(fb.positions.values()),
                     velocity=[fb.joint_velocities[n] for n in fb.positions]))
        self.state_history.append((self.clock,state))
        self.node.state=next((s for t,s in reversed(self.state_history)
                              if t<=self.clock-self.state_delay),state)
        self.node.mode_state.stamp=self.clock
        if not (self.input_gap and self.commands):
            self.node.frames=(self.frozen_frames if self.frozen_frames and
                              self.clock<=self.commands[0].stamp+self.duplicate_window
                              else self.frame(self.step,0. if self.step<2 else 1.))
        if self.commands and self.transition=='regrip':
            elapsed=self.clock-self.commands[0].stamp
            released=.06<=elapsed<.22
            if elapsed>=.06:self.clutch=1
            self.node.frames=self.frame(self.step,0. if released else 1.,clutch=self.clutch)
            if elapsed>=.06:
                # Move both hands while released, then engage at that new pose.
                for side,frame in self.node.frames.items():
                    pose=np.array(frame.pose).reshape(4,4);pose[0,3]+=.2
                    self.node.frames[side]=replace(frame,pose=tuple(pose.ravel()))

    def done(self,value):
        future=asyncio.get_running_loop().create_future();future.set_result(value)
        return future

    @staticmethod
    def motion_command(**kw):
        defaults=dict(velocity=S(linear=S(x=0.,y=0.,z=0.),angular=S(x=0.,y=0.,z=0.)),
                      points=[],duration=0.)
        defaults.update(kw)
        return S(**defaults)

    def lease(self,request):
        if request.operation=='release':self.releases+=1
        reply=self.services.lease(request,S())
        self.update()
        return self.done(reply)

    def hold(self,request):
        reply=self.services.control(request,S())
        self.update()
        return self.done(reply)

    def solve(self,operation,token,feedback,target,**kwargs):
        if operation=='capture':
            value=np.eye(4)
        else:
            error=next(self.solve_errors,None)
            self.solve_attempts.append((self.clock,token,self.node.frames['left'].seq,
                                        kwargs['reference'],kwargs['dt'],error))
            if error:
                self.node.worker_result=('error',token,error)
                return True
            # Deterministic already-validated candidate. Reference and execution
            # integration below are actual production code, not solver models.
            value=(tuple(q+self.candidate_step for q in kwargs['reference']),(1.,0.,1.))
        self.node.worker_result=('result',token,value)
        return True

    def submit(self,request):
        c=request.command
        self.commands.append(c)
        if self.frozen_frames is None:self.frozen_frames=dict(self.node.frames)
        if self.send_failure:raise RuntimeError('INJECTED_TRANSPORT_FAILURE')
        reply=(S(accepted=False,code='INJECTED_REJECTION',revision=self.engine.revision)
               if self.reject else self.services.submit(request,S()))
        future=asyncio.get_running_loop().create_future()
        self.futures.append(future)
        if not self.drop:
            self.receipts.append((self.clock+self.ack_delay,future,reply))
        return future

    async def pause(self):
        self.clock=round(self.clock+self.control_step,9);self.step+=1
        self.steady+=self.steady_step
        if self.commands and self.delivery_gap is not None:
            self.clock+=self.delivery_gap
            self.steady+=self.delivery_gap
            self.delivery_gap=None
        self.update()
        for at,future,reply in list(self.receipts):
            if self.clock>=at and not future.done():future.set_result(reply)
        if self.commands and self.transition and not self.transitioned:
            first_at=self.commands[0].stamp
            if self.clock>=first_at+self.ack_delay:
                self.transitioned=True
                if self.transition=='reconnect':self.node.frames=self.frame(self.step,1.,source='new-headset')
                elif self.transition=='release':self.node.frames=self.frame(self.step,0.,clutch=1)
                elif self.transition=='restart':self.node.state.instance_id='restarted-executor'
                elif self.transition=='unordered':self.node.frames=self.frame(self.step,1.,stamp=first_at-.01)
                elif self.transition=='cancel':self.goal.is_cancel_requested=True
                elif self.transition in ('one_arm_offline','stale_and_restart'):
                    self.node.frames['right']=replace(self.node.frames['right'],stamp=first_at-.3)
                    if self.transition=='stale_and_restart':self.node.state.instance_id='new-executor'
        if (self.after_error and not self.error_injected and self.node.worker_result and
                self.node.worker_result[0]=='error'):
            self.error_injected=True
            if self.after_error=='freeze_control_clock':self.control_step=0.
            elif self.after_error=='release':self.node.frames=self.frame(self.step,0.,clutch=1)
            elif self.after_error=='reconnect':self.node.frames=self.frame(self.step,1.,source='other')
            elif self.after_error=='one_arm_offline':self.node.state.missing_feedback_groups=['right_arm']
            elif self.after_error=='nan':self.node.state.joints.position[0]=float('nan')
            elif self.after_error=='limit':self.node.state.joints.position[0]=1.1
            elif self.after_error=='stop_fault':self.node.state.stop_failures=['injected']
            elif self.after_error=='old_epoch':self.node.state.lease_epoch+=2
        await asyncio.sleep(0)

    async def run(self):
        return await self.node.execute(self.goal)


class BimanualTransactions(unittest.IsolatedAsyncioTestCase):
    async def test_restart_waits_for_new_pair_instead_of_consuming_old_stop_projection(self):
        rig=Rig();original=rig.update
        def update():
            original()
            if rig.step<2:
                rig.node.frames={side:replace(f,stamp=9.9,stop=True)
                                 for side,f in rig.node.frames.items()}
            elif rig.step<4:
                rig.node.frames={side:replace(f,grip=0.) for side,f in rig.node.frames.items()}
        rig.update=update;update()
        result=await rig.run()
        self.assertNotEqual(result.code,'VR_PAIR_INVALID')
        self.assertTrue(rig.commands)
        self.assertTrue(all(c.stamp>10. for c in rig.commands))

    async def test_new_stop_projection_still_rejects_without_motion(self):
        rig=Rig();original=rig.update
        def update():
            original()
            rig.node.frames={side:replace(f,stop=True) for side,f in rig.node.frames.items()}
        rig.update=update;update()
        result=await rig.run()
        self.assertEqual(result.code,'VR_PAIR_INVALID')
        self.assertFalse(rig.commands)

    async def test_replacement_before_final_stop_cannot_confirm_old_execution(self):
        rig=Rig()
        original_engine,original_update=rig.engine,rig.update
        replaced=False
        def update():
            nonlocal replaced
            if rig.commands and not replaced:
                replaced=True
                drivers={g:SimJointDriver(names,.5) for g,names in rig.profile.groups.items()}
                rig.io=RobotIO(rig.profile,drivers,SimBaseDriver())
                rig.engine=Executor(rig.profile,rig.io,steady=lambda:rig.steady)
                rig.engine.acquire('other',list(rig.profile.groups),2.,rig.clock)
                rig.engine.halt(rig.clock,revoke=True)
                rig.services.engine,rig.services.robot_io=rig.engine,rig.io
            original_update()
            if replaced:rig.node.state.instance_id='replacement'
        rig.update=update
        result=await rig.run()
        self.assertIn('STOP_UNCONFIRMED',result.code)
        self.assertTrue(rig.node.stop_unconfirmed)
        self.assertIsNotNone(original_engine.active)
        diagnostic=json.loads(next(e[1] for e in rig.events if e[0]=='BIMANUAL_STOP_UNCONFIRMED'))
        self.assertEqual(diagnostic['requested_instance_id'],'executor')
        self.assertEqual(diagnostic['observed_instance_id'],'replacement')

    async def test_lost_release_reply_latches_across_idle_restart_and_late_ack(self):
        rig=Rig()
        original_lease=rig.lease
        lost=[]
        def lease(request):
            completed=original_lease(request)
            if request.operation=='release':
                future=asyncio.get_running_loop().create_future()
                lost.append((future,completed.result()))
                return future
            return completed
        rig.node.lease_client.call_async=lease
        result=await rig.run()
        self.assertIn('STOP_UNCONFIRMED',result.code)
        self.assertFalse(rig.node.busy)
        self.assertTrue(rig.node.stop_unconfirmed)
        count=len(rig.commands)
        for identity in ('executor','replacement'):
            rig.update()
            rig.node.state.instance_id=identity
            self.assertEqual(rig.node.goal(rig.goal.request),'REJECT')
            with self.assertRaisesRegex(RuntimeError,'STOP_UNCONFIRMED'):
                rig.node.check(require_input=False)
        lost[0][0].set_result(lost[0][1])
        rig.update()
        self.assertEqual(rig.node.goal(rig.goal.request),'REJECT')
        self.assertEqual(len(rig.commands),count)

    async def test_grasp_rejection_logs_quantitative_evidence_and_can_reengage_after_stop(self):
        rig=Rig(solve_errors=['BIMANUAL_MEASURED_GRASP_ERROR'])
        original_solve=rig.solve
        diagnostic=dict(code='BIMANUAL_MEASURED_GRASP_ERROR',stage='measured_grasp_points',
            position_error_m=.010024,rotation_error_rad=.0,position_limit_m=.01,rotation_limit_rad=.02)
        def solve(operation,token,*args,**kw):
            submitted=original_solve(operation,token,*args,**kw)
            if rig.node.worker_result and rig.node.worker_result[0]=='error':
                rig.node.worker.diagnostic=(token,dict(scale=1.,constraint=diagnostic,timing=None))
            return submitted
        rig.node.worker.submit=solve
        result=await rig.run()
        self.assertEqual(result.code,'BIMANUAL_MEASURED_GRASP_ERROR')
        self.assertFalse(rig.node.stop_unconfirmed)
        event=json.loads(next(e[1] for e in rig.events if e[0]=='BIMANUAL_WORK_REJECTED'))
        self.assertEqual(event['constraint'],diagnostic)
        self.assertFalse(any(e[0]=='BIMANUAL_SOLVE_DEFERRED' for e in rig.events))
        self.assertFalse(rig.engine.lease_id)
        rig.step=0;rig.update()
        self.assertEqual(rig.node.goal(rig.goal.request),'ACCEPT')
        rig.goal.request.duration=.3
        result=await rig.run()
        self.assertTrue(result.success,result.code)
        self.assertEqual(rig.releases,2)

    async def test_confirmed_stop_allows_explicit_new_action_without_restart(self):
        rig=Rig()
        result=await rig.run()
        self.assertTrue(result.success,result.code)
        self.assertFalse(rig.node.stop_unconfirmed)
        rig.step=0;rig.update()
        self.assertEqual(rig.node.goal(rig.goal.request),'ACCEPT')
        rig.goal.request.duration=.3
        result=await rig.run()
        self.assertTrue(result.success,result.code)
        self.assertEqual(len(rig.references),2)
        self.assertEqual(rig.releases,2)

    async def test_acquire_timeout_settles_exact_late_grant_before_release(self):
        rig=Rig()
        original_lease=rig.lease
        def lease(request):
            completed=original_lease(request)
            if request.operation=='acquire':
                future=asyncio.get_running_loop().create_future()
                rig.receipts.append((rig.clock+.8,future,completed.result()))
                return future
            return completed
        rig.node.lease_client.call_async=lease
        result=await rig.run()
        self.assertEqual(result.code,'BIMANUAL_SERVICE_TIMEOUT')
        self.assertEqual(rig.commands,[])
        self.assertEqual(rig.releases,1)
        self.assertFalse(rig.node.stop_unconfirmed)
        self.assertFalse(rig.engine.lease_id)

    async def test_unknown_acquire_latches_even_after_lease_auto_expires_to_idle(self):
        rig=Rig()
        original_lease=rig.lease
        pending=[]
        def lease(request):
            completed=original_lease(request)
            if request.operation=='acquire':
                future=asyncio.get_running_loop().create_future()
                pending.append((future,completed.result()))
                return future
            return completed
        rig.node.lease_client.call_async=lease
        result=await rig.run()
        self.assertEqual(result.code,'BIMANUAL_SERVICE_TIMEOUT:STOP_UNCONFIRMED')
        self.assertEqual(rig.commands,[])
        while rig.engine.lease_id:await rig.pause()
        # Expiry alone cannot settle an unknown RPC.
        self.assertTrue(rig.node.stop_unconfirmed)
        self.assertEqual(rig.node.goal(rig.goal.request),'REJECT')
        self.assertTrue(any(e[0]=='BIMANUAL_LEASE_UNCONFIRMED' for e in rig.events))
        self.assertFalse(pending[0][0].cancelled())
        pending[0][0].set_result(pending[0][1])
        rig.update()
        self.assertTrue(rig.node.stop_unconfirmed)
        self.assertEqual(rig.node.goal(rig.goal.request),'REJECT')

    async def test_deferral_end_diagnostic_failure_cannot_prevent_release(self):
        rig=Rig(solve_errors=['BIMANUAL_QP_DEFERRED']*100)
        rig.goal.request.duration=.12
        original_event=rig.node.event
        def event(name,*args):
            if name=='BIMANUAL_SOLVE_DEFERRED_END':
                raise RuntimeError('INJECTED_DEFERRAL_LOG_FAILURE')
            original_event(name,*args)
        rig.node.event=event
        result=await rig.run()
        self.assertFalse(result.success,result.code)
        self.assertIn('INJECTED_DEFERRAL_LOG_FAILURE',result.code)
        self.assertEqual(rig.releases,1)
        self.assertFalse(rig.engine.lease_id)
        self.assertFalse(rig.node.busy)
        self.assertTrue(any(e[0]=='BIMANUAL_DIAGNOSTIC_FAILED' for e in rig.events))

    async def test_unconfirmed_stop_diagnostic_failure_cannot_skip_latch_or_finish(self):
        for logger_fails in (False,True):
            with self.subTest(logger_fails=logger_fails):
                rig=Rig()
                original_lease,original_event=rig.lease,rig.node.event
                logger=Mock()
                if logger_fails:logger.warning.side_effect=RuntimeError('LOGGER_UNAVAILABLE')
                rig.node.get_logger=lambda:logger
                def lease(request):
                    completed=original_lease(request)
                    return (asyncio.get_running_loop().create_future()
                            if request.operation=='release' else completed)
                def event(name,*args):
                    if name=='BIMANUAL_STOP_UNCONFIRMED':
                        raise RuntimeError('INJECTED_STOP_LOG_FAILURE')
                    original_event(name,*args)
                rig.node.event=event
                rig.node.lease_client.call_async=lease
                result=await rig.run()
                self.assertIn('STOP_UNCONFIRMED',result.code)
                self.assertTrue(rig.node.stop_unconfirmed)
                self.assertFalse(rig.node.busy)
                self.assertEqual(rig.node.goal(rig.goal.request),'REJECT')
                self.assertIn('BIMANUAL_STOP_UNCONFIRMED',rig.node.diagnostic_error)
                self.assertIn('INJECTED_STOP_LOG_FAILURE',rig.node.diagnostic_error)
                logger.warning.assert_called_once()

    async def test_runtime_diagnostic_failure_ends_control_before_another_command(self):
        rig=Rig()
        original_event=rig.node.event
        def event(name,*args):
            if name=='BIMANUAL_QP_RESULT':
                raise ValueError('INJECTED_RUNTIME_LOG_FAILURE')
            original_event(name,*args)
        rig.node.event=event
        result=await rig.run()
        self.assertEqual(result.code,'INJECTED_RUNTIME_LOG_FAILURE')
        self.assertEqual(len(rig.commands),1)
        self.assertEqual(rig.releases,1)
        self.assertFalse(rig.engine.lease_id)
        self.assertIsNone(rig.engine.active)
        self.assertFalse(rig.node.busy)

    async def test_non_runtime_service_errors_cannot_bypass_unknown_stop_latch(self):
        for operation in ('acquire','release'):
            for failure in ('dispatch','future'):
                with self.subTest(operation=operation,failure=failure):
                    rig=Rig()
                    original_lease=rig.lease
                    def lease(request):
                        completed=original_lease(request)
                        if request.operation!=operation:return completed
                        error=ValueError('INJECTED_'+operation.upper()+'_FAILURE')
                        if failure=='dispatch':raise error
                        future=asyncio.get_running_loop().create_future()
                        future.set_exception(error)
                        return future
                    rig.node.lease_client.call_async=lease
                    result=await rig.run()
                    self.assertIn('STOP_UNCONFIRMED',result.code)
                    self.assertTrue(rig.node.stop_unconfirmed)
                    self.assertFalse(rig.node.busy)
                    self.assertEqual(rig.node.goal(rig.goal.request),'REJECT')

    async def test_ended_diagnostic_failure_cannot_leave_action_busy(self):
        rig=Rig()
        original_event=rig.node.event
        logger=Mock()
        rig.node.get_logger=lambda:logger
        def event(name,*args):
            if name=='BIMANUAL_ENDED':raise RuntimeError('INJECTED_ENDED_LOG_FAILURE')
            original_event(name,*args)
        rig.node.event=event
        result=await rig.run()
        self.assertTrue(result.success,result.code)
        self.assertFalse(rig.node.busy)
        self.assertFalse(rig.engine.lease_id)
        self.assertIn('INJECTED_ENDED_LOG_FAILURE',rig.node.diagnostic_error)
        logger.warning.assert_called_once()

    async def test_release_dispatch_failure_latches_and_preserves_first_cause(self):
        rig=Rig()
        original_lease=rig.lease
        def lease(request):
            completed=original_lease(request)
            if request.operation=='release':
                raise RuntimeError('INJECTED_RELEASE_TRANSPORT_FAILURE')
            return completed
        rig.node.lease_client.call_async=lease
        result=await rig.run()
        self.assertIn('STOP_UNCONFIRMED',result.code)
        self.assertTrue(rig.node.stop_unconfirmed)
        self.assertFalse(rig.node.busy)
        diagnostic=json.loads(next(e[1] for e in rig.events if e[0]=='BIMANUAL_STOP_UNCONFIRMED'))
        self.assertEqual(diagnostic['reason'],'INJECTED_RELEASE_TRANSPORT_FAILURE')
        self.assertEqual(rig.node.goal(rig.goal.request),'REJECT')

    async def test_acquire_dispatch_error_cannot_claim_stopped(self):
        rig=Rig()
        original_lease=rig.lease
        def lease(request):
            original_lease(request)
            raise RuntimeError('INJECTED_ACQUIRE_TRANSPORT_FAILURE')
        rig.node.lease_client.call_async=lease
        result=await rig.run()
        self.assertEqual(result.code,'INJECTED_ACQUIRE_TRANSPORT_FAILURE:STOP_UNCONFIRMED')
        self.assertEqual(rig.commands,[])
        self.assertTrue(rig.node.stop_unconfirmed)
        self.assertEqual(rig.node.goal(rig.goal.request),'REJECT')

    async def test_replaced_instance_during_acquire_never_bootstraps_or_confirms(self):
        rig=Rig()
        original_lease,original_update=rig.lease,rig.update
        def lease(request):
            completed=original_lease(request)
            if request.operation=='acquire':
                future=asyncio.get_running_loop().create_future()
                rig.receipts.append((rig.clock+.02,future,completed.result()))
                return future
            return completed
        def update():
            original_update()
            rig.node.state.instance_id='replacement'
        rig.node.lease_client.call_async=lease
        rig.update=update
        result=await rig.run()
        self.assertIn('STOP_UNCONFIRMED',result.code)
        self.assertTrue(result.code.startswith('BIMANUAL_EXECUTION_REVISION_UNAVAILABLE'))
        self.assertEqual(rig.commands,[])
        self.assertTrue(rig.node.stop_unconfirmed)

    async def test_release_regrip_uses_new_baselines_without_process_restart(self):
        rig=Rig(transition='regrip')
        result=await rig.run()
        self.assertTrue(result.success,result.code)
        captures=[json.loads(e[1]) for e in rig.events if e[0]=='BIMANUAL_CAPTURED']
        self.assertEqual(len(captures),2)
        self.assertGreater(np.linalg.norm(np.array(captures[1]['controller_pose'])-
                                         np.array(captures[0]['controller_pose'])),.1)
        second=captures[1]
        attempts=[a for a in rig.solve_attempts if a[0]>=second['capture_completed_at']]
        np.testing.assert_allclose(attempts[0][3],list(second['capture_reference'].values()))
        self.assertEqual(len(rig.references),1)
        self.assertTrue(all(c.stamp<rig.commands[0].stamp+.06 or
                            c.stamp>=rig.commands[0].stamp+.22 for c in rig.commands))

    async def test_one_guarded_iteration_failure_keeps_reference_and_uses_only_new_input(self):
        rig=Rig(solve_errors=['BIMANUAL_QP_DEFERRED'])
        result=await rig.run()
        self.assertTrue(result.success,result.code)
        first,second=rig.solve_attempts[:2]
        self.assertEqual(first[3],second[3])
        self.assertGreater(second[2],first[2])
        self.assertGreater(second[4],first[4])
        self.assertFalse(any(c.command_id.endswith('_'+str(first[1])) for c in rig.commands))
        self.assertGreater(len(rig.references[0].commits),2)
        summaries=[json.loads(e[1]) for e in rig.events if e[0]=='BIMANUAL_SOLVE_DEFERRED_END']
        self.assertEqual(summaries[0]['count'],1)
        self.assertEqual(summaries[0]['outcome'],'software_acceptance')

    async def test_continuous_guarded_failures_stop_with_existing_reference_budget(self):
        rig=Rig(solve_errors=['BIMANUAL_QP_DEFERRED']*100)
        result=await rig.run()
        self.assertFalse(result.success)
        self.assertIn(result.code.split(':')[0],('BIMANUAL_INTEGRATION_INTERVAL',
                      'BIMANUAL_SOLVE_DEFERRED_TIMEOUT','BIMANUAL_SOLVE_DEFERRED_LIMIT'))
        self.assertEqual(rig.commands,[])
        self.assertEqual(rig.references[0].commits,[])
        self.assertGreater(len(rig.solve_attempts),1)
        self.assertFalse(rig.engine.lease_id)
        self.assertEqual(len([e for e in rig.events if e[0]=='BIMANUAL_SOLVE_DEFERRED']),1)
        self.assertEqual(len([e for e in rig.events if e[0]=='BIMANUAL_SOLVE_DEFERRED_END']),1)

    async def test_frozen_control_clock_cannot_extend_monotonic_deferral_budget(self):
        rig=Rig(solve_errors=['BIMANUAL_QP_DEFERRED']*100,after_error='freeze_control_clock')
        result=await rig.run()
        self.assertEqual(result.code,'BIMANUAL_SOLVE_DEFERRED_TIMEOUT:STOP_UNCONFIRMED')
        summary=json.loads(next(e[1] for e in rig.events if e[0]=='BIMANUAL_SOLVE_DEFERRED_END'))
        self.assertLessEqual(summary['duration_s'],rig.node.cfg['integration_max_sample_gap']+.02)
        self.assertEqual(rig.commands,[])

    async def test_deferred_then_unknown_submit_never_advances_or_refreshes(self):
        rig=Rig(solve_errors=['BIMANUAL_QP_DEFERRED'],drop=True)
        result=await rig.run()
        self.assertFalse(result.success)
        self.assertTrue(result.code.startswith('BIMANUAL_SOLVE_DEFERRED_TIMEOUT'),result.code)
        self.assertEqual(len(rig.commands),1)
        self.assertEqual(rig.references[0].commits,[])
        self.assertFalse(rig.engine.lease_id)

    async def test_deferred_then_safety_or_compute_failure_stops_without_more_attempts(self):
        for reason in ('SELF_COLLISION:injected','BIMANUAL_POSITION_LIMIT:joint',
                       'BIMANUAL_NONFINITE_RESULT','BIMANUAL_QP_TIMEOUT:native'):
            with self.subTest(reason=reason):
                rig=Rig(solve_errors=['BIMANUAL_QP_DEFERRED',reason])
                result=await rig.run()
                self.assertEqual(result.code,reason)
                self.assertEqual(rig.commands,[])
                self.assertEqual(len(rig.solve_attempts),2)

    async def test_fast_wakeups_cannot_exceed_derived_consecutive_count(self):
        rig=Rig(solve_errors=['BIMANUAL_QP_DEFERRED']*1000,control_step=.0001,steady_step=.000001)
        result=await rig.run()
        self.assertEqual(result.code,'BIMANUAL_SOLVE_DEFERRED_LIMIT')
        summary=json.loads(next(e[1] for e in rig.events if e[0]=='BIMANUAL_SOLVE_DEFERRED_END'))
        self.assertEqual(summary['count'],summary['count_limit'])
        self.assertLessEqual(summary['count'],50)  # Existing .25 s / .005 s timer.
        self.assertEqual(rig.commands,[])

    async def test_zero_qp_step_does_not_claim_or_request_physical_stop(self):
        rig=Rig(solve_errors=[None,'BIMANUAL_QP_DEFERRED',None],candidate_step=.02)
        result=await rig.run()
        self.assertTrue(result.success,result.code)
        event=json.loads(next(e[1] for e in rig.events if e[0]=='BIMANUAL_SOLVE_DEFERRED'))
        self.assertEqual(event['execution_state'],'RUNNING')
        self.assertFalse(event['stop_confirmed'])
        self.assertEqual(event['execution_command_id'],rig.commands[0].command_id)
        self.assertGreater(max(abs(q-event['feedback'][n]) for n,q in
                               zip(event['joint_names'],event['reference'])),1e-4)
        self.assertEqual(len({c.command_id for c in rig.commands}),len(rig.commands))

    async def test_guarded_failure_combined_with_invalid_state_or_intent_cannot_continue(self):
        for fault in ('release','reconnect','one_arm_offline','nan','limit','stop_fault','old_epoch'):
            with self.subTest(fault=fault):
                rig=Rig(solve_errors=['BIMANUAL_QP_DEFERRED'],after_error=fault)
                result=await rig.run()
                self.assertEqual(rig.commands,[],result.code)
                self.assertEqual(rig.references[0].commits,[])
                self.assertFalse(any(e[0]=='BIMANUAL_SOLVE_DEFERRED' for e in rig.events),result.code)

    async def test_deferred_samples_never_refresh_a_previously_accepted_command(self):
        rig=Rig(solve_errors=[None,None]+['BIMANUAL_QP_DEFERRED']*100)
        result=await rig.run()
        self.assertFalse(result.success)
        self.assertIn(result.code.split(':')[0],('COMMAND_TIMEOUT','BIMANUAL_INTEGRATION_INTERVAL'))
        self.assertEqual(len(rig.commands),2)
        self.assertEqual(len(rig.references[0].commits),2)
        self.assertFalse(rig.engine.lease_id)

    async def test_unsafe_or_unknown_solver_failure_is_not_deferred(self):
        for error in ('BIMANUAL_QP:joint bounds infeasible','BIMANUAL_NONFINITE_RESULT',
                      'BIMANUAL_QP_TIMEOUT:native','SELF_COLLISION:arm',
                      'BIMANUAL_QP_DEFERRED:unknown'):
            with self.subTest(error=error):
                rig=Rig(solve_errors=[error]);result=await rig.run()
                self.assertEqual(result.code,error)
                self.assertFalse(result.success)
                self.assertEqual(len(rig.solve_attempts),1)
                self.assertEqual(rig.commands,[])

    def test_original_recorded_packets_keep_source_time_and_reject_stale_replay(self):
        fixture=json.loads((Path(__file__).resolve().parents[2]/'tests/config/teleoperation/snowbot_input_gap.json').read_text())
        pairs=fixture['pairs']
        origin=pairs[0]['left']['received_at']
        def received(pair):return 10.+pair['left']['received_at']-origin
        def frames(pair):
            result=[]
            for side in ('left','right'):
                d=pair[side]['data'];stamp=d['stamp']['sec']+d['stamp']['nanosec']*1e-9
                result.append(VRFrame(d['source_id'],d['seq'],10.+stamp-origin,side,
                    tuple(d['pose']),d['grip'],d['trigger'],d['init'],d['stop'],d['valid'],
                    d['clutch_seq'],d['mapping']))
            return result
        session=BimanualSession(dict(input_max_age=.25,engage_seconds=.1,
            position_scale=1.,operator_yaw_rad=0.),1,0.)
        index=max(range(len(pairs)-1),key=lambda i:received(pairs[i+1])-received(pairs[i]))
        before,after=pairs[index:index+2]
        self.assertTrue(session.ingest(*frames(before),received(before)))
        previous=session.seq
        self.assertFalse(session.ingest(*frames(before),received(before)+.001))
        with self.assertRaisesRegex(ValueError,'VR_PAIR_INVALID'):
            session.ingest(*frames(before),received(after))
        self.assertEqual(session.seq,previous)
        self.assertTrue(session.ingest(*frames(after),received(after)))

    async def test_normal_software_acceptance_advances_once_per_unique_command(self):
        rig=Rig()
        result=await rig.run()
        commits=rig.references[0].commits
        self.assertTrue(result.success,result.code)
        self.assertGreater(len(commits),2)
        # End-of-duration settles the last receipt for accounting, without
        # advancing a reference that is no longer used for motion.
        self.assertLessEqual(len(rig.commands)-len(commits),1)
        self.assertEqual(result.accepted_commands,len(rig.commands))
        self.assertEqual(len(set(c.command_id for c in rig.commands)),len(rig.commands))
        self.assertEqual([round(q[1][0],6) for q in commits],
                         [round((i+1)*.001,6) for i in range(len(commits))])

    async def test_rejection_and_unknown_acceptance_do_not_advance_or_retry(self):
        for changes in (dict(reject=True),dict(drop=True),dict(send_failure=True)):
            with self.subTest(changes=changes):
                rig=Rig(**changes)
                result=await rig.run()
                self.assertFalse(result.success)
                self.assertEqual(rig.references[0].commits,[])
                self.assertEqual(len(rig.commands),1)
                self.assertEqual(rig.releases,1)

    async def test_old_confirmation_cannot_commit_across_reconnect_unordered_release_or_restart(self):
        for transition in ('reconnect','unordered','release','restart','cancel','one_arm_offline','stale_and_restart'):
            with self.subTest(transition=transition):
                rig=Rig(transition=transition)
                result=await rig.run()
                self.assertGreaterEqual(len(rig.commands),1)
                self.assertEqual(rig.references[0].commits,[],result.code)
                self.assertEqual(rig.releases,1)

    async def test_source_expired_confirmation_never_advances_reference(self):
        rig=Rig(ack_delay=.18,state_delay=.04)
        result=await rig.run()
        self.assertFalse(result.success)
        self.assertTrue(result.code.startswith('BIMANUAL_SUBMIT_EXPIRED'),result.code)
        self.assertEqual(rig.references[0].commits,[],result.code)

    async def test_recorded_input_delivery_gap_stops_and_does_not_replay_intent(self):
        fixture=json.loads((Path(__file__).resolve().parents[2]/'tests/config/teleoperation/snowbot_input_gap.json').read_text())
        rig=Rig(drop=True,input_gap=True,delivery_gap=fixture['gap_s'])
        rig.goal.request.duration=3.
        result=await rig.run()
        self.assertFalse(result.success)
        self.assertEqual(rig.references[0].commits,[])
        self.assertEqual(len(rig.commands),1)
        self.assertFalse(rig.engine.lease_id)
        # Receiving new input alone does not re-enter the completed coroutine.
        rig.node.frames=rig.frame(1000,1.)
        await rig.pause()
        self.assertEqual(len(rig.commands),1)

    async def test_duplicate_frames_do_not_resubmit_or_integrate_again(self):
        rig=Rig(duplicate_window=.1)
        result=await rig.run()
        self.assertTrue(result.success,result.code)
        seqs=[json.loads(e[1])['input_seq'] for e in rig.events if e[0]=='BIMANUAL_SUBMIT_TIMING']
        self.assertGreater(len(seqs),2)
        self.assertEqual(len(seqs),len(set(seqs)))
        self.assertEqual(len(rig.references[0].commits),len(seqs))

    async def test_explicit_reentry_recaptures_measured_state_and_ignores_old_ack(self):
        rig=Rig(drop=True)
        result=await rig.run()
        self.assertFalse(result.success)
        first=rig.commands[0]
        self.assertEqual(rig.references[0].commits,[])
        rig.futures[0].set_result(S(accepted=True,revision=1))
        rig.drop=False;rig.step=0
        rig.node.pause_control=ToolPause();rig.node.busy=True
        rig.node.mode_state.generation+=1
        rig.node.mode_state.changed_at=rig.clock-.01
        rig.update()
        measured=tuple(rig.engine.feedback.positions[n] for n in rig.node.names)
        rig.goal.request.duration=.4
        result=await rig.run()
        self.assertTrue(result.success,result.code)
        self.assertEqual(rig.references[1].captures[0][0],measured)
        self.assertEqual(rig.references[0].commits,[])
        self.assertTrue(all(c.command_id.rsplit('_',1)[0]!=first.command_id.rsplit('_',1)[0]
                            for c in rig.commands[1:]))


if __name__=='__main__':
    unittest.main()
