"""Physical button identity, coalesced events and guarded operator transitions."""
from dataclasses import replace
import unittest
import json
from pathlib import Path
import numpy as np
from bindu_contracts.teleoperation import VRFrame
from bindu_teleoperation.vr.receiver import decode_controller, ButtonCounters
from bindu_teleoperation.pico import PicoControl, PicoFeedback, PICO_MAPPING
from bindu_teleoperation.bimanual_session import BimanualSession


from tests.support.pico import Rig


class ScenarioRuntimeTests(unittest.IsolatedAsyncioTestCase):
    def test_execution_fault_and_stop_protocol_reach_pico_without_normal_cancel_fault(self):
        from types import SimpleNamespace as S
        node=self.production('on_motion_event');node.run_id='run';node.runtime_mode='real';node.instance_id='test'
        r=Rig();node.control=r.core;node.lease=('lease','instance');node.tool_request=None
        node.now=lambda:r.now
        def event(state,code,task='pico_test',component='execution',run='run'):
            return S(run_id=run,runtime_mode='real',task_id=task,component=component,state=state,
                     code=code,stamp=S(sec=1,nanosec=0))
        node.on_motion_event(event('STOPPING','CANCELED'))
        node.on_motion_event(event('FAILED','DEVICE_FEEDBACK_ERROR',task='another'))
        node.on_motion_event(event('FAILED','DEVICE_FEEDBACK_ERROR',run='other'))
        self.assertIsNone(r.core.first_fault)
        node.on_motion_event(event('DEVICE_COMMAND_DETAIL','SNOWBOT_PDO_STALE',task=''))
        node.on_motion_event(event('STOP_CHECK',json.dumps(dict(sequence=1932,
            observed=dict(command_sequence=1930,session_reason=7)))))
        self.assertEqual(r.core.first_fault['code'],'SNOWBOT_PDO_STALE')
        self.assertEqual(r.core.recovery_required,'device_inspection')
        self.assertEqual(node.stop_diagnostic['observed']['command_sequence'],1930)

    @staticmethod
    def production(*names):
        # Execute production methods with ROS/service/process boundaries inert.
        import ast
        from types import SimpleNamespace as S
        path=Path(__file__).resolve().parents[2]/'src/integration/bindu_runtime/bindu_runtime/pico_node.py'
        node=next(n for n in ast.parse(path.read_text()).body if isinstance(n,ast.ClassDef) and n.name=='PicoNode')
        methods=[n for n in node.body if getattr(n,'name','') in names]
        module=ast.Module(body=[ast.ClassDef(name='Actual',bases=[],keywords=[],body=methods,decorator_list=[])],type_ignores=[])
        namespace=dict(json=json,math=__import__('math'),time=S(monotonic=lambda:0.),
            seconds=lambda stamp:stamp.sec+stamp.nanosec*1e-9,
            joints_at_rest=lambda joints,names,tol:all(abs(v)<=tol for n,v in zip(joints.name,joints.velocity) if n in names),
            Trigger=S(Request=lambda:S()),Superseded=type('Superseded',(RuntimeError,),{}))
        reference=next(n for n in ast.parse(path.read_text()).body if getattr(n,'name','')=='prepare_reference')
        exec(compile(ast.Module(body=[reference],type_ignores=[]),str(path),'exec'),namespace)
        exec(compile(ast.fix_missing_locations(module),str(path),'exec'),namespace)
        return namespace['Actual']()

    def test_display_keeps_fresh_coordinated_pair_during_one_hand_delivery(self):
        from types import SimpleNamespace as S
        rig=Rig();node=self.production('input_status');node.control=rig.core
        pair=rig.core.frames
        def raw(f):
            return S(side=f.side,source_id=f.source_id,seq=f.seq,valid=f.valid,
                     stamp=S(sec=int(f.stamp),nanosec=round((f.stamp-int(f.stamp))*1e9)))
        node.frames={f.side:raw(f) for f in pair}
        state=node.input_status(rig.now)
        self.assertTrue(state['raw_input_valid']);self.assertFalse(state['input_pair_pending'])
        node.frames['left']=raw(replace(pair[0],seq=pair[0].seq+1,stamp=rig.now+.01))
        state=node.input_status(rig.now+.01)
        self.assertTrue(state['raw_input_valid']);self.assertTrue(state['input_pair_pending'])
        self.assertEqual(node.control.frames,pair)  # Display never ingests/executes a partial pair.
        rig.step(.02)
        node.frames={f.side:raw(f) for f in rig.core.frames}
        self.assertFalse(node.input_status(rig.now)['input_pair_pending'])

    def test_display_never_hides_invalid_stale_future_or_changed_source_input(self):
        from types import SimpleNamespace as S
        for fault in ('invalid','stale','future','source','reversed','missing','pair_stale','no_pair'):
            with self.subTest(fault=fault):
                rig=Rig();node=self.production('input_status');node.control=rig.core
                node.frames={f.side:S(side=f.side,source_id=f.source_id,seq=f.seq,valid=True,
                    stamp=S(sec=1,nanosec=round((f.stamp-1)*1e9))) for f in rig.core.frames}
                left=node.frames['left'];now=rig.now
                if fault=='invalid':left.valid=False
                elif fault=='stale':now+=.251
                elif fault=='future':left.stamp=S(sec=2,nanosec=0)
                elif fault=='source':left.source_id='reconnected'
                elif fault=='reversed':left.seq-=1
                elif fault=='missing':node.frames.pop('right')
                elif fault=='no_pair':node.control.frames=None
                else:
                    now+=.251
                    for f in node.frames.values():
                        f.stamp=S(sec=1,nanosec=round((now-1)*1e9));f.seq+=1
                state=node.input_status(now)
                self.assertFalse(state['raw_input_valid']);self.assertFalse(state['input_pair_pending'])

    def test_unheld_policy_is_explicit_and_defaults_to_physical_grasp_checks(self):
        import ast
        from types import SimpleNamespace as S
        path=Path(__file__).resolve().parents[2]/'src/integration/bindu_runtime/bindu_runtime/pico_node.py'
        node=next(n for n in ast.parse(path.read_text()).body if isinstance(n,ast.ClassDef) and n.name=='PicoNode')
        init=next(n for n in node.body if getattr(n,'name','')=='__init__')
        begin=next(i for i,n in enumerate(init.body) if isinstance(n,ast.Assign) and isinstance(n.targets[0],ast.Name) and n.targets[0].id=='policy')
        end=next(i for i,n in enumerate(init.body) if isinstance(n,ast.Assign) and isinstance(n.targets[0],ast.Subscript) and getattr(n.targets[0].slice,'value','')=='urdf')
        code=compile(ast.Module(body=init.body[begin:end],type_ignores=[]),str(path),'exec')
        for config,unheld in (({},False),({'measured_grasp_policy':'unheld_test'},True)):
            owner=S(tool_cfg=dict(right_axial_grasp={'configured':True},
                collision_scope='configured_tool_geometry',**config),position_contract=True)
            exec(code,{'self':owner})
            self.assertIs(owner.unheld_test,unheld)
        for policy in (None,False,'disabled','unheld'):
            with self.subTest(policy=policy),self.assertRaisesRegex(ValueError,'MEASURED_GRASP_POLICY'):
                exec(code,{'self':S(tool_cfg={'measured_grasp_policy':policy})})

    async def test_explicit_unheld_confirmation_uses_no_physical_grasp_or_calibration(self):
        from types import SimpleNamespace as S
        from unittest.mock import Mock,AsyncMock
        for unheld in (False,True):
            node=self.production('confirm_tool');node.unheld_test=unheld;node.operator_grasp=False
            node.control=S(load='unknown',virtual=True);node.operator_grasp_positions={'left':.1}
            node.feedback=lambda:PicoFeedback(0.,True,('unknown','unknown'))
            node.position_ops=S(config={'calibrated':False},reading=Mock(side_effect=AssertionError('physical grasp queried')))
            node.ready_client=S(service_is_ready=lambda:True,call_async=lambda _:S(success=True))
            node.current=Mock();node.wait=AsyncMock(side_effect=lambda value:value);node.event=Mock();node.tool_cfg={}
            intent=S(operation='select_mode')
            if not unheld:
                with self.assertRaisesRegex(RuntimeError,'UNCONFIRMED_UNCALIBRATED'):await node.confirm_tool(intent)
                continue
            await node.confirm_tool(intent)
            node.position_ops.reading.assert_not_called()
            self.assertFalse(node.control.virtual);self.assertEqual(node.operator_grasp_positions,{})
            evidence=json.loads(node.event.call_args.args[1])
            self.assertFalse(evidence['physical_grasp_confirmed']);self.assertFalse(evidence['grasp_sensor_confirmed'])
            node.control.load='held'
            with self.assertRaisesRegex(RuntimeError,'UNHELD_TEST_WITH_HELD_LOAD'):await node.confirm_tool(intent)
            node.control.load='unknown';node.feedback=lambda:PicoFeedback(0.,False)
            with self.assertRaisesRegex(RuntimeError,'REQUIRES_STOPPED'):await node.confirm_tool(intent)
            node.feedback=lambda:PicoFeedback(0.,True,healthy=False)
            with self.assertRaisesRegex(RuntimeError,'FEEDBACK_UNAVAILABLE'):await node.confirm_tool(intent)

    async def test_operator_grasp_declares_held_only_after_success_and_keeps_intent_fence(self):
        from types import SimpleNamespace as S
        from unittest.mock import Mock,AsyncMock
        for success in (False,True):
            r=Rig('unknown',guided=True,dual=True);r.click(1,1)
            intent=r.core.pending;generation=r.core.generation
            node=self.production('confirm_tool','current');node.unheld_test=False;node.operator_grasp=True;node.control=r.core
            node.feedback=lambda:PicoFeedback(r.now,True,('unknown','unknown'))
            node.position_ops=S(reading=lambda _:S(measured_position=.02))
            node.ready_client=S(service_is_ready=lambda:True,call_async=lambda _:S(success=success,message='BIMANUAL_GRASP_POSE'))
            node.wait=AsyncMock(side_effect=lambda value:value);node.event=Mock();node.tool_cfg={};node.now=lambda:r.now
            if not success:
                with self.assertRaisesRegex(RuntimeError,'GRASP_POSE'):await node.confirm_tool(intent)
                self.assertEqual(r.core.load,'unknown');self.assertEqual(r.core.load_source,'unknown')
                self.assertEqual(r.core.generation,generation);continue
            await node.confirm_tool(intent)
            self.assertEqual((r.core.load,r.core.load_source),('held','operator_tool_confirmation'))
            self.assertEqual(r.core.generation,generation+1);self.assertIs(r.core.pending,intent)
            self.assertEqual(r.core.pending.token,intent.token);node.current(intent)
            self.assertEqual(r.core.context_at,r.now)
            declaration=next(c for c in node.event.call_args_list if c.args[0]=='PICO_LOAD_OPERATOR_CONFIRMED')
            self.assertFalse(json.loads(declaration.args[1])['grasp_sensor_confirmed'])
            # Reconfirming an already-held load does not consume a new generation.
            await node.confirm_tool(intent)
            self.assertEqual(r.core.generation,generation+1);node.current(intent)
            self.assertTrue(r.core.complete(intent.token,True,now=r.now))
            with self.assertRaisesRegex(ValueError,'PICO_CONTEXT_STALE_OR_INVALID'):
                r.core.context(r.core.source,generation,'empty',False,r.now)
            self.assertEqual(r.core.load,'held')
            r.step();r.click(1,1)
            self.assertIsNone(r.core.pending)
            self.assertEqual(r.core.reason,'PICO_TOOL_HELD_REQUIRES_TOOL_MODE')

    async def test_a_return_is_empty_hand_in_every_load_and_grasp_policy(self):
        from types import SimpleNamespace as S
        from unittest.mock import Mock
        for load in ('empty','held','unknown'):
            for unheld in (False,True):
                with self.subTest(load=load,unheld=unheld):
                    node=self.production('validate_prepare');node.unheld_test=unheld;node.position_contract=True
                    node.control=S(load=load);node.prepare_positions={'joint':.2};node.names=('joint',)
                    node.tool_cfg={'resource_groups':['arm']};node.cfg={'prepare_collision_recovery':{'configured':True}}
                    node.current=Mock();node.event=Mock()
                    node.profile=S(groups={'arm':['joint']},limits={'joint':(-1.,1.)},feedback_tolerances={'joint_position':.002})
                    node.reading=lambda:(S(joints=S(name=['joint'],velocity=[0.])),{'joint':0.})
                    def submit(*args,**kwargs):node.validation_result=('result',1,{});return True
                    node.validator=S(ready=True,busy=False,startup_timeout=1.,submit=Mock(side_effect=submit))
                    intent=S(operation='prepare',token=1)
                    await node.validate_prepare(intent)
                    node.validator.submit.assert_called_once_with('prepare_recovery_path',1,{'joint':0.},[.2])
                    event=json.loads(node.event.call_args.args[1])
                    self.assertFalse(event['tool_present']);self.assertFalse(event['coupling_checked'])
                    self.assertFalse(event['load_state_checked'])

    async def test_unheld_ignores_gripper_drift_while_loaded_operator_guard_remains(self):
        from types import SimpleNamespace as S
        from unittest.mock import Mock
        for unheld in (False,True):
            node=self.production('tick');node.unheld_test=unheld;node.operator_grasp=True
            node.last_tick_at=None;node.waiters=set();node.validator=S(poll=lambda:None)
            node.state=None;node.tool_result=None;node.pending_intent=None;node.record_future=None;node.running=None
            node.now=lambda:0.;node.last_status=0.;node.last_context_at=0.
            node.control=S(teleop_mode='bimanual_tool',phase='following',context_at=0.,
                lock=Mock(),update_feedback=Mock(),tick=Mock(),drain=lambda:[])
            node.feedback=lambda:PicoFeedback(0.,True)
            node.operator_grasp_positions={'left':0.}
            node.position_ops=S(config={'position_tolerance':.003},reading=Mock(return_value=S(state='IDLE',measured_position=.01)))
            node.tick()
            self.assertEqual(node.control.lock.call_count,0 if unheld else 1)
            if unheld:node.position_ops.reading.assert_not_called()

    async def test_unheld_rejects_axial_switch_before_any_gripper_command(self):
        from types import SimpleNamespace as S
        from unittest.mock import Mock,AsyncMock
        node=self.production('perform');node.unheld_test=True;node.operator_grasp=False
        node.position_contract=True;node.position_ops=S(axial_available=True,grasp_target=Mock())
        node.cancel_tool=AsyncMock();node.current=Mock();node.now=lambda:0.;node.event=Mock();node.simulation=False
        node.control=S(complete=Mock())
        await node.perform(S(operation='switch_grasp',token=1,value=True))
        node.position_ops.grasp_target.assert_not_called()
        args=node.control.complete.call_args.args
        self.assertFalse(args[1]);self.assertIn('PICO_AXIAL_GRASP_UNAVAILABLE',args[2])

    async def test_prepared_residual_inside_its_band_does_not_acquire_or_move_again(self):
        from types import SimpleNamespace as S
        from unittest.mock import Mock,AsyncMock
        for error in (.0284,.03501):
            with self.subTest(error=error):
                node=self.production('perform');node.position_contract=True;node.prepare_position_tolerance=.035
                node.control=S(complete=Mock());node.current=Mock();node.cancel_tool=AsyncMock()
                node.validate_prepare=AsyncMock(return_value={'joint':.2});node.reading=lambda:(None,{'joint':.2-error})
                node.prepare_recovery=None;node.cfg={'prepare_duration':12.};node.simulation_demo=False
                node.move=AsyncMock();node.now=lambda:0.;node.event=Mock();node.simulation=False
                await node.perform(S(operation='prepare',token=1))
                self.assertEqual(node.move.await_count,0 if error<.035 else 1)
                self.assertTrue(node.control.complete.call_args.args[1])
                if error<.035:
                    event=next(c for c in node.event.call_args_list if c.args[0]=='PICO_PREPARE_ALREADY_REACHED')
                    self.assertEqual(json.loads(event.args[1])['position_tolerance_rad'],.035)
                node.validate_prepare.assert_awaited_once()


class PrepareReturnPathTests(unittest.IsolatedAsyncioTestCase):
    async def arrival(self,operation='prepare',error=.0284,velocity=0.,fresh=True,stop_error=None):
        from types import SimpleNamespace as S
        from unittest.mock import Mock,AsyncMock,patch
        from bindu_runtime import pico_node as module
        from bindu_contracts.profile import Profile
        from bindu_teleoperation.pico import PicoIntent
        from tests.support.paths import ROOT
        profile=Profile.load(ROOT/'src/integration/bindu_runtime/config/robots/snowbot/snowbot_real_position.json')
        clock=[0.];joint='arm_l_joint_1';target=.2
        lease=S(ok=True,lease_id='test',epoch=1,profile_hash=profile.digest)
        state=S(instance_id='executor',profile_hash=profile.digest,
            lease_active=True,lease_epoch=1,revision=0,command_id='',state='HOLDING',
            joints=S(name=[joint],velocity=[velocity]),reference=S(name=[joint],position=[target]))
        def reading():
            state.feedback_stamp=module.stamp(clock[0] if fresh else 0.)
            return state,{joint:target-error}
        async def pause():clock[0]+=1.
        async def wait(value):return value
        node=S(current=Mock(),feedback=lambda:S(stopped=True),prepare_position_tolerance=.035,
            leases=S(call_async=lambda r:lease),wait=wait,instance_id='test',groups=('left_arm',),
            state=state,reading=reading,names=(joint,),position_contract=True,profile=profile,
            event=Mock(),simulation=False,now=lambda:clock[0],runtime_mode='real',pause=pause,
            submit=S(call_async=lambda r:S(accepted=True,revision=0)),
            release_lease=AsyncMock(side_effect=stop_error))
        with patch.object(module,'time',S(monotonic=lambda:clock[0])):
            try:await module.PicoNode.move(node,PicoIntent(1,operation),{joint:target},2.)
            finally:node.release_lease.assert_awaited_once()
        return node

    async def test_prepare_arrival_tolerance_is_separate_from_other_moves_and_stop(self):
        node=await self.arrival()
        arrived=[json.loads(c.args[1]) for c in node.event.call_args_list if c.args[0]=='PICO_ACTUATOR_ARRIVED']
        self.assertEqual(len(arrived),1);self.assertEqual(arrived[0]['position_tolerance_rad'],.035)
        for options in ({'error':.03501},{'operation':'load'},{'velocity':.00501},{'fresh':False}):
            with self.subTest(options=options),self.assertRaisesRegex(RuntimeError,'PICO_ACTUATOR_FEEDBACK_TIMEOUT'):
                await self.arrival(**options)
        await self.arrival(operation='load',error=.0019)
        with self.assertRaisesRegex(RuntimeError,'STOP_UNCONFIRMED'):
            await self.arrival(stop_error=RuntimeError('STOP_UNCONFIRMED'))

    def node(self, load='unknown', mode='bimanual_tool', *, error=None, changed=False):
        from types import SimpleNamespace as S
        from unittest.mock import Mock
        node=S(control=S(load=load,teleop_mode=mode),position_contract=True,unheld_test=False,
            prepare_positions={'joint':.2},names=('joint',),cfg={},
            tool_cfg={'resource_groups':['arm']},profile=S(groups={'arm':('joint',)},
                limits={'joint':(-1.,1.)},feedback_tolerances={'joint_position':.002}),
            current=Mock(),event=Mock(),validation_result=None)
        state=S(joints=S(name=['joint'],velocity=[0.]))
        readings=[(state,{'joint':0.}),(state,{'joint':.01 if changed else 0.})]
        node.reading=Mock(side_effect=readings)
        def submit(operation, token, feedback, target, **kwargs):
            node.validation_result=('error',token,error) if error else ('result',token,{})
            return True
        node.validator=S(ready=True,busy=False,startup_timeout=1.,submit=Mock(side_effect=submit))
        return node

    async def test_return_does_not_preserve_captured_grasp_in_either_mode_or_any_load_state(self):
        from bindu_runtime.pico_node import PicoNode
        from bindu_teleoperation.pico import PicoIntent
        for load in ('empty','held','unknown'):
            for mode in ('independent_arms','bimanual_tool'):
                for unheld in (False,True):
                    with self.subTest(load=load,mode=mode,unheld=unheld):
                        node=self.node(load,mode);node.unheld_test=unheld
                        self.assertEqual(await PicoNode.validate_prepare(node,PicoIntent(1,'prepare')),{'joint':.2})
                        node.validator.submit.assert_called_once_with('prepare_path',1,{'joint':0.},[.2])
                        self.assertEqual((node.control.load,node.control.teleop_mode),(load,mode))
                        event=json.loads(node.event.call_args.args[1])
                        self.assertFalse(event['load_state_checked']);self.assertFalse(event['coupling_checked'])
                        self.assertFalse(event['tool_present'])

    async def test_return_still_propagates_robot_collision_limits_and_changed_feedback(self):
        from bindu_runtime.pico_node import PicoNode
        from bindu_teleoperation.pico import PicoIntent
        for error in ('SELF_COLLISION: arm / body t=0.5','PREPARE_JOINT_LIMIT'):
            with self.subTest(error=error),self.assertRaisesRegex(RuntimeError,error):
                await PicoNode.validate_prepare(self.node(error=error),PicoIntent(1,'prepare'))
        with self.assertRaisesRegex(RuntimeError,'PREPARE_STATE_CHANGED'):
            await PicoNode.validate_prepare(self.node(changed=True),PicoIntent(1,'prepare'))

    async def test_boundary_reference_is_explicit_and_does_not_replace_measured_validation_start(self):
        from bindu_runtime.pico_node import PicoNode
        from bindu_teleoperation.pico import PicoIntent
        from types import SimpleNamespace as S
        node=self.node('empty');measured={'joint':-1.00007}
        state=S(joints=S(name=['joint'],velocity=[0.]))
        node.reading.side_effect=[(state,measured),(state,measured)]
        await PicoNode.validate_prepare(node,PicoIntent(1,'prepare'))
        node.validator.submit.assert_called_once_with('prepare_path',1,measured,[.2],reference={'joint':-1.})
        self.assertEqual(measured,{'joint':-1.00007})

    def test_boundary_reference_preserves_strict_curve_limits_and_rejects_large_overrun(self):
        from bindu_runtime.pico_node import prepare_reference
        from bindu_execution.interpolation import Curve,State
        from bindu_contracts.profile import Profile
        from tests.support.paths import ROOT
        profile=Profile.load(ROOT/'src/integration/bindu_runtime/config/robots/snowbot/snowbot_real_position.json')
        for name in ('arm_r_joint_3','arm_l_joint_3'):
            lo,hi=profile.limits[name]
            for actual,target in ((lo-.00007,lo+.1),(hi+.00007,hi-.1)):
                measured={name:actual};legal=prepare_reference(profile,[name],measured)
                curve=Curve.between(0.,2.,State.rest(legal.values()),State.rest([target]))
                self.assertTrue(curve.valid(profile,[name]))
                for t in np.linspace(0,2,101):self.assertTrue(lo<=curve.sample(t).q[0]<=hi)
                self.assertEqual(measured[name],actual)
            for bad in (lo-.002001,hi+.002001,float('nan')):
                with self.assertRaisesRegex(RuntimeError,'PREPARE_START_JOINT_LIMIT'):
                    prepare_reference(profile,[name],{name:bad})


class OperationCancellationTests(unittest.IsolatedAsyncioTestCase):
    async def test_gripper_handoff_refreshes_stopped_feedback_before_held_a_completion(self):
        from types import SimpleNamespace as S
        from unittest.mock import Mock,AsyncMock
        from bindu_runtime.pico_node import PicoNode
        r=Rig(guided=True,direct=True);r.triggers=[1.,0.];r.step()
        r.triggers=[0.,0.];r.step();r.stopped=False;r.buttons[1]=1;r.step()
        intent=r.core.pending
        node=S(control=r.core,cancel_tool=AsyncMock(),current=Mock(),
            position_ops=S(move_grippers=AsyncMock()),now=lambda:r.now,
            feedback=Mock(return_value=PicoFeedback(r.now,True,('closed','closed'))),
            event=Mock(),simulation=True)
        await PicoNode.perform(node,intent)
        node.feedback.assert_called_once()
        self.assertEqual(r.core.reason,'PREPARE_HOLD_AFTER_GRIPPER_STOP')
        self.assertEqual(r.core.a_since,r.now)

    async def run_superseded(self, cleanup_error=None):
        from types import SimpleNamespace as S
        from unittest.mock import Mock,AsyncMock
        from bindu_runtime.pico_node import PicoNode,Superseded
        r=Rig(guided=True,direct=True);r.triggers=[1.,0.];old=r.step()[-1]
        r.triggers=[0.,0.];replacement=r.step()[-1]
        node=S(control=r.core,cancel_tool=AsyncMock(side_effect=[None,cleanup_error]),
            current=Mock(side_effect=Superseded('PICO_OPERATION_SUPERSEDED')),
            now=lambda:r.now,event=Mock(),simulation=True)
        await PicoNode.perform(node,old)
        self.assertEqual(node.cancel_tool.await_count,2)
        self.assertEqual(r.core.pending,replacement)
        return node

    async def test_normal_replacement_is_cancellation_and_keeps_new_intent(self):
        node=await self.run_superseded()
        self.assertEqual(node.event.call_args.args[0],'PICO_OPERATION_CANCELED')
        value=json.loads(node.event.call_args.args[1])
        self.assertEqual(value['outcome'],'superseded')
        self.assertNotIn('success',value)

    async def test_failed_stop_during_replacement_remains_a_real_error(self):
        node=await self.run_superseded(RuntimeError('PICO_STOP_UNCONFIRMED'))
        self.assertEqual(node.event.call_args.args[0],'PICO_OPERATION_RESULT')
        value=json.loads(node.event.call_args.args[1])
        self.assertFalse(value['success']);self.assertIn('PICO_STOP_UNCONFIRMED',value['reason'])


class DualModeTests(unittest.TestCase):
    def test_default_normal_accepts_one_hand_and_releases_independently(self):
        r=Rig('unknown',guided=True,dual=True)
        self.assertEqual(r.core.teleop_mode,'independent_arms')
        r.grips=[1.,0.];r.step();self.assertEqual(r.core.pending.operation,'follow')
        r.complete();r.stopped=False;r.grips=[1.,1.];r.step()
        r.click(0,0);self.assertIsNone(r.core.pending)
        r.grips=[0.,1.];self.assertEqual(r.step(),[])
        self.assertEqual(r.core.phase,'following')
        r.grips=[0.,0.];r.step();self.assertEqual(r.core.pending.operation,'stop')

    def test_b_switches_only_at_released_rest_and_does_not_follow(self):
        r=Rig(guided=True,dual=True);r.click(1,1)
        self.assertEqual((r.core.pending.operation,r.core.pending.value),('select_mode','bimanual_tool'))
        self.assertEqual(r.core.teleop_mode,'independent_arms')
        r.complete();self.assertEqual(r.core.teleop_mode,'bimanual_tool')
        self.assertEqual(r.core.phase,'standby');self.assertIsNone(r.core.pending)
        r.step();r.click(1,1);r.complete()
        self.assertEqual(r.core.teleop_mode,'independent_arms')

    def test_rejected_tool_mode_keeps_normal_mode_and_specific_reason(self):
        r=Rig(guided=True,dual=True);r.click(1,1)
        r.core.complete(r.core.pending.token,False,'RuntimeError:BIMANUAL_GRASP_POSE',r.now,rejected=True)
        self.assertEqual(r.core.teleop_mode,'independent_arms')
        self.assertEqual(r.core.phase,'standby');self.assertIsNone(r.core.pending)

    def test_known_coupled_load_cannot_enter_independent_arms(self):
        r=Rig('held',guided=True,dual=True);r.grips=[1.,0.];r.step()
        self.assertIsNone(r.core.pending)
        self.assertEqual(r.core.reason,'PICO_TOOL_HELD_REQUIRES_TOOL_MODE')
        r.core.teleop_mode='bimanual_tool';r.grips=[0.,0.];r.step();r.click(1,1)
        self.assertIsNone(r.core.pending);self.assertEqual(r.core.teleop_mode,'bimanual_tool')

    def test_mode_key_while_following_only_stops_then_requires_new_press(self):
        r=Rig(guided=True,dual=True);r.grips=[1.,0.];r.step();r.complete()
        r.click(1,1);self.assertEqual(r.core.pending.operation,'stop');r.complete()
        self.assertEqual(r.core.teleop_mode,'independent_arms')
        r.grips=[0.,0.];r.step();self.assertIsNone(r.core.pending)
        r.click(1,1);self.assertEqual(r.core.pending.operation,'select_mode')



class StateTests(unittest.TestCase):
    def test_stop_failure_and_late_request_never_replace_first_fault(self):
        r=Rig();r.core.phase='following'
        r.core.lock('PICO_FEEDBACK_UNAVAILABLE',r.now)
        stop=r.core.pending
        r.core.record_fault('SNOWBOT_PDO_STALE',r.now-.01)
        r.core.complete(stop.token,False,'PICO_STOP_UNCONFIRMED:TOOL_STOP_UNCONFIRMED',r.now+.01)
        r.core.record_fault('PICO_STOP_UNCONFIRMED:REQUEST_NOT_CURRENT',r.now+.02,kind='request')
        r.core.lock('PICO_INPUT_TIMEOUT',r.now+.03)
        view=r.core.snapshot()
        self.assertEqual(view['first_fault']['code'],'SNOWBOT_PDO_STALE')
        self.assertEqual(view['reason'],'PICO_FEEDBACK_UNAVAILABLE')
        self.assertEqual(view['stop_failure']['code'],'PICO_STOP_UNCONFIRMED:TOOL_STOP_UNCONFIRMED')
        self.assertEqual(view['request_failure']['code'],'PICO_STOP_UNCONFIRMED:REQUEST_NOT_CURRENT')
        self.assertEqual(view['recovery_required'],'device_inspection')
        self.assertIsNone(r.core.pending)
        r.click(1,1)
        self.assertEqual(r.core.phase,'locked')

    def test_confirmed_stream_stop_recovers_only_on_fresh_released_b_edge(self):
        r=Rig();r.core.phase='following'
        r.core.lock('PICO_INPUT_TIMEOUT',r.now);r.complete()
        self.assertFalse(r.core.recovery_required)
        r.grips=[1.,0.];r.step();r.click(1,1)
        self.assertEqual(r.core.phase,'locked')
        r.grips=[0.,0.];r.step();r.click(1,1)
        self.assertEqual(r.core.phase,'standby');self.assertFalse(r.core.fault_active)
        self.assertEqual(r.core.first_fault['code'],'PICO_INPUT_TIMEOUT')
        self.assertIsNone(r.core.pending)
        r.core.record_fault('DEVICE_FEEDBACK_ERROR',r.now-.2)
        self.assertFalse(r.core.fault_active)  # Delayed prior-episode telemetry.
        r.core.lock('PICO_PAIR_INVALID',r.now+.01)
        self.assertEqual(r.core.first_fault['code'],'PICO_PAIR_INVALID')
        self.assertIsNone(r.core.stop_failure)

    def test_failed_stop_cannot_recover_from_rest_only(self):
        r=Rig();r.core.lock('PICO_INPUT_TIMEOUT',r.now)
        r.complete(False,'PICO_STOP_UNCONFIRMED:DEADLINE_EXPIRED')
        r.click(1,1)
        self.assertEqual(r.core.phase,'locked')

    def test_timeout_keeps_triggering_old_frame_evidence_after_fresh_input_returns(self):
        rig=Rig(guided=True,dual=True);rig.grips=[1.,1.];rig.step();rig.complete()
        core=rig.core;old_stamp=core.stamp;old_seq=core.seq
        rig.now=old_stamp+.26
        core.update_feedback(PicoFeedback(rig.now,False),rig.now)
        core.tick(rig.now)
        evidence=core.last_input_timeout
        self.assertEqual((core.phase,core.reason),('locked','PICO_INPUT_TIMEOUT'))
        self.assertEqual(evidence['phase'],'following')
        self.assertEqual(evidence['input_seq'],old_seq)
        self.assertAlmostEqual(evidence['input_age_s'],.26)
        core.tick(rig.now+.01)
        rig.step(.01)
        self.assertIs(core.last_input_timeout,evidence)
        self.assertTrue(core.snapshot(rig.now)['input_fresh'])
        self.assertEqual(core.snapshot(rig.now)['last_input_timeout'],evidence)


    def test_pair_skew_compares_each_hand_with_its_own_previous_time(self):
        rig=Rig();core=rig.core;t=rig.now+.02
        pair=tuple(replace(f,seq=rig.seq+1,stamp=t+i*.0007) for i,f in enumerate(core.frames))
        core.ingest(*pair,t+.005)
        newer=tuple(replace(f,seq=f.seq+1,stamp=t+dt) for f,dt in zip(pair,(.00056,.001)))
        core.ingest(*newer,t+.006)
        self.assertEqual(core.seq,newer[0].seq)
        self.assertEqual(core.phase,'standby')

    def test_real_time_reversal_of_either_hand_still_locks_and_stops(self):
        for side in (0,1):
            with self.subTest(side=side):
                rig=Rig();core=rig.core;previous=core.frames
                pair=tuple(replace(f,seq=f.seq+1,stamp=f.stamp+(-.0001 if i==side else .001))
                           for i,f in enumerate(previous))
                core.ingest(*pair,rig.now+.005)
                self.assertEqual((core.phase,core.reason),('locked','PICO_TIME_REVERSED'))
                self.assertEqual(core.pending.operation,'stop')
                self.assertEqual(core.frames,previous)

    def test_stop_fault_keeps_its_cause_instead_of_claiming_feedback_loss(self):
        rig=Rig();core=rig.core
        code='PICO_STOP_UNCONFIRMED:STOP_UNCONFIRMED:vendor_hold'
        core.update_feedback(PicoFeedback(rig.now,False,healthy=False,fault=code),rig.now)
        self.assertEqual((core.phase,core.reason),('locked',code))
        self.assertFalse(core.rest(rig.now))

    def test_startup_input_before_feedback_does_not_latch_timeout_or_use_stale_pair(self):
        core=PicoControl(initial_load='unknown')
        rig=Rig()
        core.ingest(*rig.core.frames,rig.now)
        later=rig.now+1.
        core.tick(later)
        self.assertEqual(core.phase,'checking')
        core.update_feedback(PicoFeedback(later,True),later)
        self.assertEqual(core.phase,'checking')
        rig.step(1.01)
        core.ingest(*rig.core.frames,rig.now)
        core.update_feedback(PicoFeedback(rig.now,True),rig.now)
        self.assertEqual(core.phase,'standby')
        core.tick(rig.now+1.)
        self.assertEqual((core.phase,core.reason),('locked','PICO_INPUT_TIMEOUT'))

    def test_initial_feedback_discovery_waits_without_latching_or_moving(self):
        rig=Rig();core=PicoControl(initial_load='empty')
        core.update_feedback(PicoFeedback(0.,False,healthy=False),rig.now)
        core.ingest(*rig.core.frames,rig.now)
        core.update_feedback(PicoFeedback(0.,False,healthy=False),rig.now)
        core.tick(rig.now)
        self.assertEqual(core.phase,'checking');self.assertEqual(core.drain(),[])
        core.update_feedback(PicoFeedback(rig.now,True,('closed','closed')),rig.now)
        self.assertEqual(core.phase,'standby')

    def test_boot_has_no_effects_and_squeeze_is_not_gripper_open(self):
        rig=Rig();self.assertEqual(rig.core.phase,'standby')
        rig.grips=[1.,1.];self.assertEqual(rig.step(),[])
        self.assertEqual(rig.core.desired,['closed','closed'])

    def test_trigger_controls_only_its_gripper_and_release_closes(self):
        rig=Rig();rig.loading()
        rig.triggers[0]=1.;effects=rig.step()
        self.assertEqual(effects[-1].value,('open','closed'))
        rig.modes=('open','closed');rig.complete()
        rig.triggers[0]=0.;effects=rig.step()
        self.assertEqual(effects[-1].value,('closed','closed'))
        rig.modes=('closed','closed');rig.complete()
        rig.triggers[1]=1.;self.assertEqual(rig.step()[-1].value,('closed','open'))

    def test_trigger_hysteresis_does_not_close_at_half_press(self):
        rig=Rig();rig.loading();rig.triggers[0]=1.;rig.step();rig.modes=('open','closed');rig.complete()
        rig.triggers[0]=.5;self.assertEqual(rig.step(),[])
        self.assertEqual(rig.core.desired,['open','closed'])
        rig.triggers[0]=0.;self.assertEqual(rig.step()[-1].value,('closed','closed'))

    def test_prepare_held_during_loading_must_be_released_before_reuse(self):
        rig=Rig();rig.loading();rig.buttons[1]=1;rig.step()
        rig.core.phase='standby'
        for _ in range(70):self.assertEqual(rig.step(),[])

    def test_pose_origin_jump_locks_active_follow(self):
        rig=Rig('held');rig.grips=[1.,1.];rig.step();rig.core.phase='following';left,right=rig.core.frames
        pose=np.eye(4);pose[0,3]=1.
        rig.core.ingest(replace(left,pose=tuple(pose.ravel()),seq=left.seq+1,stamp=rig.now+.01),
                        replace(right,seq=right.seq+1,stamp=rig.now+.01),rig.now+.01)
        self.assertEqual(rig.core.phase,'locked')
        self.assertEqual(rig.core.reason,'PICO_POSE_ORIGIN_JUMP')
        self.assertEqual(rig.core.last_pose_jump['translation_limit_m'],.35)
        self.assertAlmostEqual(rig.core.last_pose_jump['translation_m'],1.)

    def test_moderate_displacement_and_wrist_rotation_survive_a_frame_gap(self):
        rig=Rig('held');rig.grips=[1.,1.];rig.step();rig.core.phase='following';left,right=rig.core.frames
        angle=.8;pose=np.eye(4);pose[:2,:2]=[[np.cos(angle),-np.sin(angle)],[np.sin(angle),np.cos(angle)]]
        pose[0,3]=.25
        rig.core.ingest(replace(left,pose=tuple(pose.ravel()),seq=left.seq+1,stamp=rig.now+.14),
                        replace(right,seq=right.seq+1,stamp=rig.now+.14),rig.now+.14)
        self.assertEqual(rig.core.phase,'following')
        self.assertIsNone(rig.core.last_pose_jump)

    def test_release_takes_priority_over_a_large_paused_hand_displacement(self):
        rig=Rig('held');rig.grips=[1.,1.];rig.step();rig.core.phase='following';left,right=rig.core.frames
        pose=np.eye(4);pose[0,3]=1.
        rig.core.ingest(replace(left,pose=tuple(pose.ravel()),grip=0.,seq=left.seq+1,stamp=rig.now+.01),
                        replace(right,seq=right.seq+1,stamp=rig.now+.01),rig.now+.01)
        self.assertNotEqual(rig.core.reason,'PICO_POSE_ORIGIN_JUMP')
        self.assertEqual(rig.core.pending.operation,'stop')

    def test_invalid_pose_jump_limits_are_rejected(self):
        for options in ({'pose_jump_translation_m':0.},{'pose_jump_rotation_rad':float('nan')},
                        {'pose_jump_rotation_rad':4.}):
            with self.assertRaisesRegex(ValueError,'POSE_JUMP_CONFIG'):PicoControl(**options)

    def test_invalid_pose_never_reaches_actuator_requests(self):
        rig=Rig();left,right=rig.core.frames
        rig.core.ingest(replace(left,pose=(float('nan'),)*16,seq=10),replace(right,seq=10),rig.now)
        self.assertEqual(rig.core.phase,'locked')
        self.assertEqual(rig.core.drain()[-1].operation,'stop')

    def test_two_triggers_and_cross_release_remain_independent(self):
        rig=Rig();rig.loading();rig.triggers=[1.,1.]
        self.assertEqual(rig.step()[-1].value,('open','open'))
        rig.modes=('open','open');rig.complete()
        rig.triggers[0]=0.
        self.assertEqual(rig.step()[-1].value,('closed','open'))

    def test_open_outside_loading_only_stops_and_is_not_deferred(self):
        rig=Rig();rig.triggers[1]=1.
        self.assertEqual(rig.step()[-1].operation,'stop')
        self.assertEqual(rig.core.desired,['closed','closed'])
        rig.complete();rig.context()
        rig.click(0,2);rig.complete();rig.click(1,1)
        self.assertEqual(rig.core.phase,'loading')
        self.assertEqual(rig.step(),[])

    def test_support_and_b_are_required_before_loading(self):
        rig=Rig('held');rig.click(0,2);rig.complete();rig.click(1,1)
        self.assertEqual(rig.core.phase,'load_check')
        rig.context(supported=True);rig.click(1,1)
        self.assertEqual(rig.core.phase,'loading')

    def test_prepare_requires_a_continuously_held_empty_and_released_squeeze(self):
        rig=Rig();rig.buttons[1]=1;rig.step()
        effects=[]
        for _ in range(60):effects+=rig.step()
        self.assertEqual([x.operation for x in effects],['prepare'])
        self.assertTrue(rig.core.last_prepare_check['accepted'])
        self.assertTrue(all(rig.core.last_prepare_check['checks'].values()))
        rig.buttons[1]=0
        self.assertEqual(rig.step()[-1].operation,'stop')

    def test_prepare_rejected_while_held_does_not_run_when_condition_improves(self):
        rig=Rig('held');rig.buttons[1]=1;rig.step()
        for _ in range(60):self.assertEqual(rig.step(),[])
        rig.core.load='empty'
        for _ in range(60):self.assertEqual(rig.step(),[])

    def test_prepare_failure_snapshot_retains_attempt_conditions(self):
        rig=Rig('unknown');rig.grips[0]=.8;rig.stopped=False;rig.buttons[1]=1
        for _ in range(60):self.assertEqual(rig.step(),[])
        check=rig.core.last_prepare_check
        self.assertFalse(check['accepted'])
        self.assertEqual({k for k,v in check['checks'].items() if not v},
                         {'load_empty','left_squeeze_released','motion_stopped'})
        self.assertEqual(check['load_source'],'unknown')
        rig.stopped=True;rig.grips[0]=0.;rig.step();rig.context('empty')
        self.assertEqual(rig.core.snapshot()['load_source'],'operator_declaration')
        self.assertTrue(all(rig.core.prepare_checks(rig.now).values()))
        self.assertIs(rig.core.last_prepare_check,check)
        self.assertEqual(check['load_state'],'unknown')
        from bindu_teleoperation.vr.hud import hud_spec
        hud=hud_spec(dict(mode='standby',reason=rig.core.reason,last_prepare_check=check))
        self.assertIn('空手状态未确认',hud['detail'])
        self.assertIn('左侧握未松开',hud['detail'])

    def test_startup_held_a_and_triggers_are_not_actions(self):
        rig=Rig();rig.core=PicoControl(initial_load='empty');rig.buttons[1]=1;rig.triggers=[1.,1.]
        for _ in range(80):self.assertEqual(rig.step(),[])

    def test_squeeze_release_stops_without_closing_virtual_grip(self):
        rig=Rig('held');rig.core.virtual=True;rig.core.phase='following';rig.grips=[1.,1.];rig.step()
        rig.grips[0]=0.;effects=rig.step()
        self.assertEqual(effects[-1].operation,'stop');self.assertTrue(rig.core.virtual)

    def test_trigger_during_preparation_or_x_switch_only_stops(self):
        for phase in ('preparing','following','switching','engaging'):
            rig=Rig('held');rig.core.phase=phase;rig.buttons[1]=1 if phase=='preparing' else 0
            rig.grips=[0.,0.] if phase=='preparing' else [1.,1.]
            rig.triggers[1]=1.;rig.buttons[0]=1
            effects=rig.step()
            self.assertEqual(effects[-1].operation,'stop')
            self.assertEqual(rig.core.desired,['closed','closed'])

    def test_y_has_priority_over_b_and_a(self):
        rig=Rig();rig.buttons=[2,3];effects=rig.step()
        self.assertEqual(rig.core.phase,'locked')
        self.assertEqual([x.operation for x in effects],['stop'])

    def test_timeout_and_late_completion_never_rearm(self):
        rig=Rig();rig.loading();rig.triggers[0]=1.;token=rig.step()[-1].token
        rig.now+=.3;rig.core.tick(rig.now)
        self.assertEqual(rig.core.phase,'locked')
        self.assertFalse(rig.core.complete(token,True,now=rig.now))
        self.assertEqual(rig.core.phase,'locked')

    def test_input_loss_during_open_does_not_synthesize_close(self):
        rig=Rig();rig.loading();rig.triggers[0]=1.;rig.step();rig.modes=('open','closed');rig.complete()
        rig.now+=.3;rig.core.tick(rig.now)
        self.assertEqual(rig.core.drain()[-1].operation,'stop')
        self.assertEqual(rig.core.desired,['open','closed'])

    def test_reconnect_locks_and_retired_source_cannot_take_over(self):
        rig=Rig();old=rig.core.frames;rig.source='new';rig.counters=[ButtonCounters(),ButtonCounters()]
        rig.step();self.assertEqual(rig.core.phase,'locked')
        rig.core.ingest(*old,rig.now)
        self.assertEqual(rig.core.source,'new')

    def test_coalesced_squeeze_release_stops_follow_or_preparation(self):
        for phase in ('following','preparing'):
            rig=Rig('held');rig.core.phase=phase
            left,right=rig.core.frames
            rig.core.ingest(replace(left,seq=left.seq+1,stamp=rig.now+.01,clutch_seq=left.clutch_seq+1),
                            replace(right,seq=right.seq+1,stamp=rig.now+.01),rig.now+.01)
            self.assertEqual(rig.core.drain()[-1].operation,'stop')
            self.assertNotEqual(rig.core.phase,phase)

    def test_source_change_invalidates_support_and_load_declarations(self):
        rig=Rig('held');rig.context('held',True)
        rig.source='new';rig.counters=[ButtonCounters(),ButtonCounters()];rig.step()
        self.assertFalse(rig.core.supported);self.assertEqual(rig.core.load,'unknown')

    def test_loading_completion_needs_fresh_load_declaration(self):
        rig=Rig();rig.loading();rig.click(1,1)
        self.assertEqual(rig.core.phase,'loading')
        self.assertEqual(rig.core.reason,'FRESH_LOAD_DECLARATION_REQUIRED')

    def test_context_is_fenced_and_never_recovers_a_fault(self):
        rig=Rig();rig.core.lock('fault',rig.now);rig.complete()
        with self.assertRaises(ValueError):rig.core.context(rig.source,0,'held',True,rig.now)
        rig.context('held');self.assertEqual(rig.core.phase,'locked')



if __name__ == "__main__":
    unittest.main()
