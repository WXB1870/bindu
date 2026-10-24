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
    def test_pico_sets_async_transport_before_ros_init_and_honors_site_override(self):
        import ast,os
        from unittest.mock import Mock,patch
        path=Path(__file__).resolve().parents[2]/'src/integration/bindu_runtime/bindu_runtime/pico_node.py'
        main=next(n for n in ast.parse(path.read_text()).body if getattr(n,'name','')=='main')
        for configured in (None,'SYNCHRONOUS'):
            with patch.dict(os.environ,{} if configured is None else
                    {'RMW_FASTRTPS_PUBLICATION_MODE':configured},clear=True):
                seen=[];spin=Mock(side_effect=lambda *a,**kw:seen.append(os.environ['RMW_FASTRTPS_PUBLICATION_MODE']))
                ns=dict(os=os,spin=spin,PicoNode=object)
                exec(compile(ast.Module(body=[main],type_ignores=[]),str(path),'exec'),ns);ns['main']()
                self.assertEqual(seen,[configured or 'ASYNCHRONOUS'])
                spin.assert_called_once_with(object,threaded=False)

    def test_stale_feedback_keeps_original_deadline_and_stop_failure_priority(self):
        from types import SimpleNamespace as S
        node=self.production('reading');node.lease=node.tool_request=None;node.now=lambda:1.
        node.tool_cfg={'feedback_max_age':.2};node.profile=S(digest='p',limits={'joint':(-1,1)})
        node.backend='snowbot_vendor';node.simulation=False
        node.state=S(instance_id='executor',stop_failures=[],code='OK',profile_hash='p',
            device_backend='snowbot_vendor',simulated=False,feedback_stamp=S(sec=0,nanosec=900000000),
            joints=S(name=['joint'],position=[0.]))
        self.assertEqual(node.reading()[1],{'joint':0.})
        node.state.feedback_stamp.nanosec=580000000
        with self.assertRaisesRegex(RuntimeError,'^PICO_FEEDBACK_STALE$'):node.reading()
        node.state.stop_failures=['vendor_hold']
        with self.assertRaisesRegex(RuntimeError,'PICO_STOP_UNCONFIRMED'):node.reading()

    async def test_capture_not_ready_returns_to_standby_only_after_confirmed_stop(self):
        from types import SimpleNamespace as S
        from unittest.mock import Mock,AsyncMock
        for accepted,stop_ok,code in ((0,True,'SNOWBOT_HELD_REFERENCE_NOT_STATIONARY'),
                (1,True,'SNOWBOT_HELD_REFERENCE_NOT_STATIONARY'),
                (0,False,'SNOWBOT_HELD_REFERENCE_NOT_STATIONARY'),
                (0,True,'SNOWBOT_PDO_STALE')):
            with self.subTest(accepted=accepted,stop_ok=stop_ok,code=code):
                r=Rig(guided=True);r.core.phase='engaging';intent=r.core.emit('follow');r.core.drain()
                node=self.production('perform');node.control=r.core;node.position_contract=False
                node.cancel_tool=AsyncMock(side_effect=[None,None if stop_ok else RuntimeError('STOP_UNCONFIRMED')])
                node.current=Mock();node.select_tool_mode=AsyncMock();node.follow=AsyncMock(side_effect=RuntimeError(code))
                node.tool_result=S(done=lambda:True,result=lambda:S(result=S(code=code,accepted_commands=accepted)))
                node.now=lambda:r.now;node.feedback=lambda:PicoFeedback(r.now,stop_ok)
                node.event=Mock();node.simulation=False
                await node.perform(intent)
                retry=accepted==0 and stop_ok and code=='SNOWBOT_HELD_REFERENCE_NOT_STATIONARY'
                self.assertEqual(r.core.phase,'standby' if retry else 'locked')
                self.assertEqual(r.core.squeeze_released,[False,False])
                self.assertIsNone(r.core.pending)
                if retry:
                    self.assertFalse(r.core.fault_active)
                    r.grips=[1.,1.];self.assertEqual(r.step(),[])

    def test_pending_capture_refusal_is_not_locked_by_timer_before_cleanup(self):
        from types import SimpleNamespace as S
        from unittest.mock import Mock
        for code in ('SNOWBOT_HELD_REFERENCE_NOT_STATIONARY','SNOWBOT_PDO_STALE'):
            node=self.production('tick');node.last_tick_at=None;node.waiters=set();node.validator=S(poll=lambda:None)
            node.state=node.position_ops=node.base=None;node.last_context_at=node.last_status=1.
            node.now=lambda:1.;node.feedback=lambda:PicoFeedback(1.,True)
            node.control=S(phase='engaging',context_at=1.,pending=S(operation='follow'),
                lock=Mock(),update_feedback=Mock(),tick=Mock(),drain=lambda:[])
            node.tool_result=S(done=lambda:True,result=lambda:S(result=S(code=code,accepted_commands=0)))
            node.pending_intent=node.record_future=node.running=None
            node.tick()
            self.assertEqual(node.control.lock.called,code!='SNOWBOT_HELD_REFERENCE_NOT_STATIONARY')

    def test_independent_grippers_do_not_interrupt_follow_or_lock_mode_switch(self):
        r=Rig(load='held',guided=True,dual=True,direct=True)
        r.core.independent_grippers=True;r.core.teleop_mode='bimanual_tool'
        r.core.phase='following';r.grips=[1.,1.];r.step()
        r.triggers=[1.,1.]
        self.assertEqual(r.step(),[]);self.assertEqual(r.core.phase,'following')
        r.triggers=[0.,0.]
        self.assertEqual(r.step(),[]);self.assertEqual(r.core.phase,'following')
        r.core.phase='standby';r.grips=[0.,0.];r.step()
        result=r.click(1,1)
        self.assertEqual(result[-1].operation,'select_mode')
        self.assertEqual(result[-1].value,'independent_arms')
        r.complete();r.modes=('unknown','virtual');r.core.emit('confirm_tool')
        r.complete();self.assertFalse(r.core.virtual)


    async def test_b_recovery_rechecks_exact_stop_before_clearing_latch(self):
        from types import SimpleNamespace as S
        from unittest.mock import Mock,AsyncMock
        from bindu_runtime.stop_evidence import stop_feedback_matches
        node=self.production('recover')
        node.recover.__func__.__globals__['stop_feedback_matches']=stop_feedback_matches
        node.current=Mock();node.lease=node.tool_request=node.tool_goal=None
        node.profile=S(digest='profile');node.simulation=False;node.now=lambda:10.
        node.tool_cfg={'feedback_max_age':.2};node.names=('joint',)
        state=S(instance_id='executor',lease_epoch=2,lease_active=False,profile_hash='profile',
                simulated=False,state='CANCELED',stop_failures=[],code='CANCELED',
                feedback_stamp=9.99,reference=S(name=[]),joints=S(name=['joint'],velocity=[0.]))
        node.reading=Mock(return_value=(state,{'joint':0.}))
        barrier=dict(instance_id='executor',epoch=2,lease_active=False,requested=9.8,failed=True)
        node.stop_request=barrier;node.position_ops=S(recover=AsyncMock())
        node.control=S(update_feedback=Mock(),rest=lambda _:True);node.feedback=Mock()
        await node.recover(S(token=1))
        self.assertIsNone(node.stop_request);node.position_ops.recover.assert_awaited_once()
        for change in ({'lease_epoch':3},{'lease_active':True},{'stop_failures':['vendor_hold']}):
            state.lease_epoch=2;state.lease_active=False;state.stop_failures=[]
            for key,value in change.items():setattr(state,key,value)
            node.stop_request=barrier;node.position_ops.recover.reset_mock()
            with self.assertRaisesRegex(RuntimeError,'STOP_NOT_CONFIRMED'):await node.recover(S(token=2))
            self.assertIs(node.stop_request,barrier);node.position_ops.recover.assert_not_awaited()

    async def test_feedback_fault_recovery_needs_completed_stop_and_fresh_device_hold(self):
        from types import SimpleNamespace as S
        from unittest.mock import Mock,AsyncMock
        from bindu_runtime.stop_evidence import stop_feedback_matches
        node=self.production('recover','stopped','begin_stop_wait')
        node.recover.__func__.__globals__['stop_feedback_matches']=stop_feedback_matches
        node.current=Mock();node.lease=node.tool_request=node.tool_goal=None
        node.profile=S(digest='profile');node.simulation=False;node.now=lambda:10.
        node.tool_cfg={'feedback_max_age':.2};node.names=('joint',)
        state=S(instance_id='executor',lease_epoch=10,lease_active=False,profile_hash='profile',
                simulated=False,state='FAILED',stop_failures=[],code='DEVICE_FEEDBACK_ERROR',
                feedback_stamp=9.99,reference=S(name=[]),joints=S(name=['joint'],velocity=[0.]))
        node.state=state;node.reading=Mock(return_value=(state,{'joint':0.}))
        node.position_ops=None;node.control=S(update_feedback=Mock(),rest=lambda _:True)
        node.feedback=Mock();node.stop_request=None;node.stop_diagnostic={'old':True}
        node.prepare_hold_reference=AsyncMock(return_value={'session':2,'sequence':268})
        with self.assertRaisesRegex(RuntimeError,'STOP_NOT_CONFIRMED'):
            await node.recover(S(token=1))
        request=node.begin_stop_wait('executor',10,False);request['requested']=9.8
        self.assertIsNone(node.stop_diagnostic)
        await node.stopped(request)
        self.assertEqual(node.confirmed_stop,request);self.assertIsNone(node.stop_request)
        await node.recover(S(token=1))
        self.assertEqual(node.prepare_hold_reference.await_count,2)
        for change in ({'lease_epoch':11},{'lease_active':True},{'stop_failures':['vendor_hold']},
                       {'state':'STOPPING'},{'code':'EMERGENCY_STOP'},{'feedback_stamp':9.7}):
            with self.subTest(change=change):
                saved={k:getattr(state,k) for k in change}
                for k,v in change.items():setattr(state,k,v)
                with self.assertRaisesRegex(RuntimeError,'STOP_NOT_CONFIRMED'):
                    await node.recover(S(token=2))
                for k,v in saved.items():setattr(state,k,v)
        node.prepare_hold_reference.side_effect=RuntimeError('SNOWBOT_DRIVE_FAULT')
        with self.assertRaisesRegex(RuntimeError,'DRIVE_FAULT'):await node.recover(S(token=3))
        node.prepare_hold_reference.side_effect=None
        async def owner_changed(*args,**kwargs):
            changed=S(**vars(state));changed.lease_epoch+=1
            node.reading.return_value=(changed,{'joint':0.})
            return {'session':2,'sequence':268}
        node.prepare_hold_reference.side_effect=owner_changed
        with self.assertRaisesRegex(RuntimeError,'STATE_CHANGED'):await node.recover(S(token=4))

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

    def test_prepare_fault_is_owned_by_command_id_and_not_hidden_by_lease_failure(self):
        from types import SimpleNamespace as S
        for command,owned in (('test_8_random_93',True),('another_8_random_93',False)):
            r=Rig();node=self.production('on_motion_event');node.control=r.core
            node.run_id='run';node.runtime_mode='real';node.instance_id='test'
            node.lease=('lease','executor');node.tool_request=None;node.now=lambda:r.now
            event=S(run_id='run',runtime_mode='real',component='execution',task_id='pico_prepare',
                command_id=command,state='STOPPING',code='COMMAND_TIMEOUT',stamp=S(sec=1,nanosec=0))
            node.on_motion_event(event)
            if owned:
                r.core.record_fault('RuntimeError:INVALID_LEASE',r.now+.1)
                self.assertEqual(r.core.first_fault['code'],'COMMAND_TIMEOUT')
            else:self.assertIsNone(r.core.first_fault)

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

    async def test_auto_close_must_finish_before_operator_grasp_and_preflight(self):
        from types import SimpleNamespace as S
        from unittest.mock import Mock,AsyncMock
        for outcome in ('success','timeout','superseded'):
            with self.subTest(outcome=outcome):
                r=Rig('unknown',guided=True,dual=True);r.click(1,1);intent=r.core.pending
                node=self.production('confirm_tool','current');node.unheld_test=False
                node.operator_grasp=True;node.automatic_close_grippers=True;node.control=r.core
                node.now=lambda:r.now;node.feedback=lambda:PicoFeedback(r.now,True,('unknown','unknown'))
                closed=[]
                async def close(value,**kw):
                    self.assertIs(value,intent);self.assertEqual(kw,dict(allow_command=True))
                    if outcome=='timeout':raise RuntimeError('PICO_GRIPPER_OPERATION_FAILED:left:POSITION_TIMEOUT')
                    if outcome=='superseded':r.core.pending=None
                    closed.append(True)
                def reading(_):
                    self.assertTrue(closed);return S(measured_position=0.)
                node.position_ops=S(close_grippers=AsyncMock(side_effect=close),reading=reading)
                node.ready_client=S(service_is_ready=lambda:True,call_async=Mock(return_value=S(success=True)))
                node.wait=AsyncMock(side_effect=lambda value:value);node.event=Mock();node.tool_cfg={}
                if outcome=='success':
                    await node.confirm_tool(intent)
                    self.assertEqual((r.core.load,r.core.load_source),('held','operator_tool_confirmation'))
                    node.ready_client.call_async.assert_called_once()
                else:
                    with self.assertRaises(RuntimeError):await node.confirm_tool(intent)
                    self.assertEqual(r.core.load,'unknown')
                    node.ready_client.call_async.assert_not_called()

    async def test_follow_never_starts_a_new_auto_close(self):
        from types import SimpleNamespace as S
        from unittest.mock import AsyncMock,Mock
        node=self.production('confirm_tool');node.unheld_test=False;node.operator_grasp=True
        node.automatic_close_grippers=True;node.feedback=lambda:PicoFeedback(1.,True)
        node.position_ops=S(close_grippers=AsyncMock(side_effect=RuntimeError('PICO_GRIPPERS_CLOSE_BEFORE_TOOL_MODE')))
        node.ready_client=Mock()
        intent=S(operation='follow')
        with self.assertRaisesRegex(RuntimeError,'CLOSE_BEFORE_TOOL_MODE'):await node.confirm_tool(intent)
        node.position_ops.close_grippers.assert_awaited_once_with(intent,allow_command=False)
        node.ready_client.assert_not_called()

    async def test_prepare_closes_after_arm_arrival_and_failure_cannot_report_success(self):
        from types import SimpleNamespace as S
        from unittest.mock import AsyncMock,Mock
        for failed in (False,True):
            node=self.production('perform');node.automatic_close_grippers=True;node.unheld_test=False
            node.position_contract=True;node.prepare_recovery=None;node.prepare_position_tolerance=.01
            node.simulation=False;node.simulation_demo=False;node.cfg={'prepare_duration':1.}
            node.control=S(load='unknown',complete=Mock());node.now=lambda:1.;node.current=Mock()
            node.cancel_tool=AsyncMock();node.validate_prepare=AsyncMock(return_value={'joint':.3})
            node.reading=lambda:(None,{'joint':0.});order=[]
            async def move(*a,**kw):order.append('arms_arrived')
            async def close(*a,**kw):
                self.assertEqual(order,['arms_arrived']);order.append('close')
                if failed:raise RuntimeError('PICO_GRIPPER_OPERATION_FAILED:left:POSITION_TIMEOUT')
            node.move=move;node.position_ops=S(close_grippers=close);node.event=Mock()
            intent=S(token=1,operation='prepare')
            await node.perform(intent)
            self.assertEqual(order,['arms_arrived','close'])
            self.assertIs(node.control.complete.call_args.args[1],not failed)
            self.assertFalse(node.control.complete.call_args.kwargs['rejected'])
            self.assertEqual(node.control.load,'unknown')

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
            node=self.production('tick');node.unheld_test=unheld;node.operator_grasp=True;node.base=None
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


class LimitRecoveryRuntimeTests(unittest.IsolatedAsyncioTestCase):
    async def test_short_exit_is_revalidated_before_full_prepare(self):
        from types import SimpleNamespace as S
        from unittest.mock import Mock,AsyncMock
        for exit_fails in (False,True):
            node=ScenarioRuntimeTests.production('perform');node.position_contract=True
            node.prepare_limit_reference=None;node.prepare_recovery=None;node.prepare_position_tolerance=.035
            node.cfg={'prepare_duration':12.};node.simulation_demo=False;node.simulation=False
            node.control=S(complete=Mock());node.current=Mock();node.cancel_tool=AsyncMock()
            node.now=lambda:0.;node.event=Mock();node.reading=lambda:(None,{'joint':.55})
            calls=[]
            async def validate(intent):
                calls.append('validate')
                node.prepare_limit_reference={'positions':{'joint':.58}} if len(calls)==1 else None
                return {'joint':.546} if len(calls)==1 else {'joint':.43}
            async def move(intent,target,duration,**kw):
                calls.append(('move',duration,kw.get('limit_exit',False)))
                if exit_fails:raise RuntimeError('PREPARE_LIMIT_RECOVERY_NOT_INSIDE')
            node.validate_prepare=validate;node.move=move
            await node.perform(S(operation='prepare',token=1))
            expected=['validate',('move',3.,True)]
            if not exit_fails:expected+=['validate',('move',12.,False)]
            self.assertEqual(calls,expected)
            self.assertIs(node.control.complete.call_args.args[1],not exit_fails)

    def held_node(self):
        from types import SimpleNamespace as S
        from unittest.mock import Mock,AsyncMock
        node=ScenarioRuntimeTests.production('prepare_hold_reference')
        node.current=Mock();node.stop_request=None;node.now=lambda:10.
        node.names=('joint',);node.profile=S(digest='profile',limits={'joint':(-1.,1.)})
        node.tool_cfg={'feedback_max_age':.1}
        state=S(instance_id='executor',lease_active=False,lease_epoch=2,revision=3,
                state='CANCELED',stop_failures=[],joints=S(name=['joint'],velocity=[0.]))
        node.reading=Mock(return_value=(state,{'joint':-1.01}))
        value=dict(instance_id='executor',profile_hash='profile',lease_id='',epoch=2,
                   revision=3,stamp=9.99,positions={'joint':-.995},boot_id=1,session=2,sequence=3)
        node.prepare_hold_client=S(call_async=Mock(return_value=S()))
        node.wait=AsyncMock(return_value=S(success=True,message=json.dumps(value)))
        return node,state,value

    async def test_hold_snapshot_is_read_only_and_bound_to_current_state_and_pause(self):
        from types import SimpleNamespace as S
        node,state,value=self.held_node();intent=S(token=1)
        self.assertEqual(await node.prepare_hold_reference(intent),value)
        for key,bad in [('instance_id','other'),('epoch',1),('revision',4),('stamp',9.),
                        ('boot_id',0),('positions',{'joint':-1.001})]:
            changed=dict(value,**{key:bad});node.wait.return_value=S(success=True,message=json.dumps(changed))
            with self.subTest(key=key),self.assertRaisesRegex(RuntimeError,'HOLD_CHANGED'):
                await node.prepare_hold_reference(intent)
        node.wait.return_value=S(success=True,message=json.dumps(value));state.stop_failures=['vendor_hold']
        with self.assertRaisesRegex(RuntimeError,'HOLD_CHANGED'):await node.prepare_hold_reference(intent)
        state.stop_failures=[];state.state='FAILED';state.code='DEVICE_FEEDBACK_ERROR'
        with self.assertRaisesRegex(RuntimeError,'HOLD_CHANGED'):await node.prepare_hold_reference(intent)
        self.assertEqual(await node.prepare_hold_reference(intent,allow_feedback_fault=True),value)
        state.code='EMERGENCY_STOP'
        with self.assertRaisesRegex(RuntimeError,'HOLD_CHANGED'):
            await node.prepare_hold_reference(intent,allow_feedback_fault=True)
        state.state='CANCELED';state.lease_active=True;value.update(lease_id='lease',epoch=3);state.lease_epoch=3
        node.wait.return_value=S(success=True,message=json.dumps(value));lease=S(lease_id='lease',epoch=3)
        self.assertEqual(await node.prepare_hold_reference(intent,lease=lease,previous=dict(value)),value)
        with self.assertRaisesRegex(RuntimeError,'HOLD_CHANGED'):
            await node.prepare_hold_reference(intent,lease=lease,previous=dict(value,sequence=1))

    async def test_a_validates_raw_measured_prefix_then_the_legal_held_reference_path(self):
        from types import SimpleNamespace as S
        from unittest.mock import Mock,AsyncMock
        node=ScenarioRuntimeTests.production('validate_prepare')
        node.current=Mock();node.event=Mock();node.position_contract=True;node.unheld_test=False
        node.prepare_positions={'joint':.2};node.names=('joint',);node.cfg={}
        node.prepare_limit_max=.017453292519943295;node.prepare_position_tolerance=.035
        node.control=S(load='held');node.tool_cfg={'resource_groups':['arm']}
        node.profile=S(groups={'arm':['joint']},limits={'joint':(-1.,1.)},feedback_tolerances={'joint_position':.002})
        measured={'joint':-1.01};state=S(joints=S(name=['joint'],velocity=[0.]))
        node.reading=Mock(return_value=(state,measured))
        node.prepare_hold_reference=AsyncMock(return_value={'positions':{'joint':-.995}})
        def submit(operation,token,feedback,target,**extra):
            node.validation_result=('result',token,{});return True
        node.validator=S(ready=True,busy=False,startup_timeout=1.,submit=Mock(side_effect=submit))
        self.assertEqual(await node.validate_prepare(S(token=1)),{'joint':-.965})
        node.validator.submit.assert_called_once_with('prepare_limit_recovery_path',1,measured,[-.965],reference={'joint':-.995})
        self.assertEqual(measured,{'joint':-1.01})
        self.assertEqual(node.prepare_limit_start,measured)
        # Recorded R1 residual is inside hard limits: use the original A path.
        measured['joint']=.18645057082176208
        node.profile.limits['joint']=(-3.1417453292519943,3.1417453292519943)
        node.prepare_hold_reference.reset_mock();node.validator.submit.reset_mock()
        await node.validate_prepare(S(token=2))
        node.prepare_hold_reference.assert_not_awaited()
        node.validator.submit.assert_called_once_with('prepare_path',2,measured,[.2])

    async def movement(self,*,outward=False,hold_error=False,renew_delay=None,renew_ok=True,revoked=False,renew_epoch=2):
        from types import SimpleNamespace as S
        from unittest.mock import Mock,AsyncMock
        from bindu_contracts.profile import Profile
        from bindu_execution.interpolation import Curve,State
        import uuid
        node=ScenarioRuntimeTests.production('move');clock=[0.]
        node.move.__func__.__globals__.update(time=S(monotonic=lambda:clock[0]),Curve=Curve,State=State,
            Lease=S(Request=lambda **kw:S(**kw)),SubmitMotion=S(Request=lambda **kw:S(**kw)),
            MotionCommand=lambda **kw:S(**kw),JointTrajectoryPoint=lambda **kw:S(**kw),
            Duration=lambda **kw:S(**kw),stamp=lambda t:t,uuid=uuid)
        lo=-.5974532925199433;actual=-.6073604822158813;held=-.596094062733137
        node.profile=Profile('fixture',{'arm':['joint']},{'joint':[lo,2.61]},.5,('joint_position',),'profile',
            feedback_tolerances={'joint_position':.002})
        state=S(instance_id='executor',lease_active=True,lease_epoch=2,revision=1,
                command_id='',state='CANCELED',joints=S(name=['joint'],velocity=[0.]))
        node.state=state;node.current=Mock();node.feedback=lambda:S(stopped=True)
        node.prepare_position_tolerance=.035;node.prepare_limit_max=.017453292519943295
        node.prepare_limit_reference={'positions':{'joint':held}};node.prepare_limit_start={'joint':actual}
        node.prepare_hold_reference=AsyncMock(return_value=node.prepare_limit_reference)
        if hold_error:node.prepare_hold_reference.side_effect=RuntimeError('PREPARE_LIMIT_RECOVERY_HOLD_CHANGED')
        node.position_contract=True;node.names=('joint',);node.groups=('arm',);node.cfg={};node.simulation=False
        node.instance_id='pico';node.runtime_mode='real'
        lease=S(ok=True,lease_id='lease',epoch=2,profile_hash='profile')
        renewal=S(done=lambda:clock[0]>=(renew_delay or 0.),
            result=lambda:S(**dict(vars(lease),epoch=renew_epoch)) if renew_ok else S(ok=False,code='INVALID_LEASE'))
        node.leases=S(call_async=Mock(side_effect=lambda request:lease if request.operation=='acquire' else renewal))
        async def wait(value,*args,**kw):
            if value is renewal:
                while not value.done():clock[0]=round(clock[0]+.04,6)
                return value.result()
            return value
        node.wait=wait;node.now=lambda:clock[0];node.release_lease=AsyncMock();node.event=Mock()
        def reading():
            q=actual if not clock[0] else actual-.003 if outward else .2
            state.feedback_stamp=S(sec=int(clock[0]),nanosec=0)
            if revoked and clock[0]>=.12:
                state.command_id=node.submit.call_async.call_args.args[0].command.command_id
                state.state='STOPPING';state.code='COMMAND_TIMEOUT';state.lease_active=False;state.lease_epoch=3
            return state,{'joint':q}
        node.reading=reading
        async def pause():
            clock[0]=(round(clock[0]+.04,6) if renew_delay is not None and clock[0]<.4 else
                      .01 if outward or not clock[0] else 41.)
        node.pause=pause;node.submit=S(call_async=Mock(return_value=S(accepted=True,revision=2)))
        try:await node.move(S(token=1,operation='prepare'),{'joint':.2},12.)
        finally:node.release_lease.assert_awaited_once()
        return node,held,lo

    async def test_slow_renewal_does_not_block_fresh_targets_and_failures_still_stop(self):
        node,_,_=await self.movement(renew_delay=.24)
        stamps=[c.args[0].command.stamp for c in node.submit.call_async.call_args_list]
        self.assertGreaterEqual(len([t for t in stamps if t<.24]),5)
        self.assertLess(max(b-a for a,b in zip(stamps,stamps[1:])),.1)
        for kwargs,reason in (({'renew_ok':False},'INVALID_LEASE'),({'revoked':True},'COMMAND_TIMEOUT'),
                              ({'renew_epoch':3},'PICO_EXECUTION_AUTHORITY_CHANGED'),
                              ({'renew_delay':10.},'PICO_LEASE_RENEW_TIMEOUT')):
            with self.subTest(kwargs=kwargs),self.assertRaisesRegex(RuntimeError,reason):
                await self.movement(**dict({'renew_delay':.24},**kwargs))

    async def test_recovery_starts_at_confirmed_reference_and_returns_inside_hard_bounds(self):
        node,held,lo=await self.movement()
        command=node.submit.call_async.call_args.args[0].command
        self.assertAlmostEqual(command.positions[0],held)
        self.assertGreaterEqual(command.positions[0],lo)
        node.prepare_hold_reference.assert_awaited_once()

    async def test_changed_hold_or_further_outward_motion_aborts_and_releases(self):
        for options,reason in [({'hold_error':True},'HOLD_CHANGED'),({'outward':True},'MOVED_OUTWARD')]:
            with self.subTest(options=options),self.assertRaisesRegex(RuntimeError,reason):
                await self.movement(**options)


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
        renewal=S(done=lambda:True,result=lambda:lease)
        node=S(current=Mock(),feedback=lambda:S(stopped=True),prepare_position_tolerance=.035,
            leases=S(call_async=lambda r:lease if r.operation=='acquire' else renewal),wait=wait,instance_id='test',groups=('left_arm',),
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
    def test_b_rechecks_latched_stop_and_restores_standby_only_after_confirmation(self):
        r=Rig();r.core.lock('PICO_STOP_UNCONFIRMED',r.now);r.core.drain();r.complete(False,'PICO_STOP_UNCONFIRMED')
        intent=r.click(1,1)[-1]
        self.assertEqual(intent.operation,'recover')
        r.core.complete(intent.token,False,'GRIPPER_RECOVERY_REQUIRES_REST',r.now)
        self.assertEqual(r.core.phase,'locked')
        intent=r.click(1,1)[-1]
        r.complete(True)
        self.assertEqual((r.core.phase,r.core.reason),('standby','RECOVERED_TO_STANDBY'))
        self.assertIsNone(r.core.stop_failure)

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
