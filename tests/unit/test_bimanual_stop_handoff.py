from bindu_contracts.resources import package_share
"""Actual bimanual stop methods, isolated from ROS and numerical dependencies."""
import ast
import asyncio
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace as S
import unittest
from unittest.mock import Mock

from tests.support.execution import ExecutionFixture
from bindu_execution.executor import Rejected
from bindu_runtime.stop_evidence import stop_feedback_matches


SOURCE = Path(__file__).resolve().parents[2]/'src/integration/bindu_runtime/bindu_runtime/bimanual_tool_node.py'
TREE = ast.parse(SOURCE.read_text())
NODE = next(n for n in TREE.body if isinstance(n, ast.ClassDef) and n.name == 'BimanualToolNode')


def actual_methods(clock):
    names = {'wait', 'begin_stop', 'execution_revision', 'stop_pending', 'stop','cleanup_event'}
    methods = [n for n in NODE.body if getattr(n, 'name', '') in names]
    namespace = {'time':clock, 'seconds':lambda stamp:stamp,
                 'stop_feedback_matches':stop_feedback_matches,
                 'Lease':S(Request=lambda **kw:S(**kw)),
                 'ControlExecution':S(Request=lambda **kw:S(**kw)),
                 'joints_at_rest':lambda joints,names,tolerance:all(
                     abs(v)<=tolerance for v in joints.velocity),
                 'json':__import__('json')}
    module = ast.Module(body=[ast.ClassDef(name='Actual', bases=[], keywords=[],
        body=methods, decorator_list=[])], type_ignores=[])
    exec(compile(ast.fix_missing_locations(module), str(SOURCE), 'exec'), namespace)
    return namespace['Actual']


class BimanualStopHandoff(unittest.IsolatedAsyncioTestCase):
    def setUp(self):
        self.clock = S(value=0.)
        self.clock.monotonic = lambda:self.clock.value
        self.node = actual_methods(self.clock)()
        self.node.profile = S(stop_timeout=.3, digest='profile',feedback_tolerances={})
        self.node.simulation = True
        self.node.cfg = {'feedback_max_age':.2}
        self.node.names = ('a',)
        self.node.now = lambda:10.+self.clock.value
        self.node.event = Mock()
        self.node.pause_control = S(confirm_stopped=Mock())
        self.lease = S(lease_id='lease', epoch=3)
        self.calls = []
        self.steps = []
        self.node.execution_instance_id='executor-instance'
        self.node.state = self.state(7, 'RUNNING')
        self.node.worker_result = ('solve', 41, 'old')
        self.node.control_client = S(call_async=self.hold)
        self.node.lease_client = S(call_async=self.release)
        self.node.pause = self.pause

    def state(self, revision, state='IDLE', **kw):
        data = dict(revision=revision, state=state, code='', stop_failures=[],
            instance_id='executor-instance',
            lease_active=True, lease_epoch=3, profile_hash='profile', simulated=True,
            feedback_stamp=self.node.now(), reference=S(name=[]),
            joints=S(name=['a'], velocity=[0.]))
        data.update(kw)
        return S(**data)

    def hold(self, request):
        self.calls.append(('hold', request.lease_id, request.epoch))
        future = asyncio.get_running_loop().create_future()
        future.set_result(S(ok=True))
        return future

    def release(self, request):
        if request.operation == 'renew':
            self.calls.append(('renew',request.lease_id,request.epoch))
            future=self.pending()
            future.set_result(S(ok=True,code='OK',lease_id=request.lease_id,
                                epoch=request.epoch,profile_hash='profile'))
            return future
        self.calls.append(('release', request.lease_id, request.epoch))
        future = asyncio.get_running_loop().create_future()
        future.set_result(S(ok=True))
        self.node.state = self.state(10, lease_active=False, lease_epoch=4)
        return future

    async def pause(self):
        self.calls.append(('await',))
        self.clock.value += .02
        if self.steps:
            self.steps.pop(0)()
        await asyncio.sleep(0)

    def pending(self):
        return asyncio.get_running_loop().create_future()

    async def test_hold_renews_real_lease_until_delayed_rest_then_revokes(self):
        peer=PicoStopEvidence();peer.setUp();self.addCleanup(peer.doCleanups)
        clock=S(monotonic=lambda:peer.clock)
        tool=actual_methods(clock)();peer.node=tool
        tool.profile=S(stop_timeout=4.,digest=peer.p.digest,feedback_tolerances={})
        tool.simulation=True;tool.cfg={'feedback_max_age':.2}
        tool.names=tuple(peer.p.limits);tool.execution_instance_id='original'
        tool.now=lambda:peer.clock;tool.event=Mock()
        tool.control_client=S(call_async=lambda request:peer.rpc('control',request))
        tool.lease_client=S(call_async=lambda request:peer.rpc('lease',request))
        calls=[]
        peer.rpc_hook=lambda method,request:calls.append((method,request.operation,peer.clock))
        # The actual executor owns the two-second lease. Only measured-stop
        # availability is delayed beyond it; the client cannot invent rest.
        def observation():
            peer.overrides.update(state='STOPPING' if peer.clock<12.4 else 'CANCELED',
                joints=S(name=list(tool.names),velocity=[.02 if peer.clock<12.4 else 0.]*len(tool.names)))
        async def pause():
            observation();await peer.pause();observation();peer.update()
        tool.pause=pause;observation();peer.update()
        lease=S(lease_id=peer.e.lease_id,epoch=peer.e.epoch)
        self.assertTrue(await tool.stop(lease))
        self.assertGreaterEqual(peer.clock,12.4)
        self.assertEqual(peer.e.epoch,lease.epoch)
        self.assertEqual(peer.e.lease_id,lease.lease_id)
        self.assertGreaterEqual(sum(op=='renew' for _,op,_ in calls),2)
        self.assertTrue(await tool.stop(lease,release=True))
        self.assertEqual(peer.e.epoch,lease.epoch+1)
        self.assertFalse(peer.e.lease_id)
        self.assertEqual(calls[-1][1],'release')

    async def test_hold_renewal_cannot_extend_original_stop_deadline(self):
        self.node.state=self.state(7,'STOPPING')
        async def fresh_moving():
            await self.pause()
            self.node.state=self.state(7,'STOPPING')
        self.node.pause=fresh_moving
        self.assertFalse(await self.node.stop(self.lease))
        self.assertLess(self.clock.value,.83)
        self.assertTrue(any(call[0]=='renew' for call in self.calls))

    async def test_hold_rejects_renewal_with_changed_identity_or_permission(self):
        for change,value in (('ok',False),('lease_id','other'),('epoch',4),('profile_hash','other')):
            with self.subTest(change=change):
                self.clock.value=0.;self.node.state=self.state(7,'STOPPING')
                async def fresh():
                    await self.pause();self.node.state=self.state(7,'STOPPING')
                self.node.pause=fresh
                def reply(request):
                    data=dict(ok=True,code='OK',lease_id='lease',epoch=3,profile_hash='profile')
                    data[change]=value;future=self.pending();future.set_result(S(**data));return future
                self.node.lease_client=S(call_async=reply)
                self.assertFalse(await self.node.stop(self.lease))
                diagnostic=__import__('json').loads(self.node.event.call_args.args[1])
                self.assertIn('BIMANUAL_STOP_RENEW_REJECTED',diagnostic['reason'])

    async def test_hold_precedes_late_acceptance_and_revision_cannot_roll_back(self):
        pending = self.pending()
        def late_reply():
            pending.set_result(S(accepted=True, revision=8))
            self.node.state = self.state(8)  # Queued pre-hold state is insufficient.
            self.node.worker_result = ('solve', 41, 'late')
        self.steps = [late_reply, lambda:setattr(self.node, 'state', self.state(9))]
        self.assertEqual(await self.node.stop_pending(self.lease, pending, 7), (True,1,9))
        self.assertEqual(self.calls[0], ('hold','lease',3))
        self.assertGreaterEqual(sum(c[0]=='await' for c in self.calls),2)

    async def test_stop_fenced_rejections_are_expected_but_other_rejections_abort(self):
        for code in ('STALE_REVISION','STOPPING','STALE_OR_UNORDERED','INVALID_LEASE'):
            with self.subTest(code=code):
                pending = self.pending()
                pending.set_result(S(accepted=False, code=code))
                self.node.state = self.state(8)
                if code == 'INVALID_LEASE':
                    with self.assertRaisesRegex(RuntimeError, code):
                        await self.node.stop_pending(self.lease,pending,7)
                else:
                    self.assertEqual(await self.node.stop_pending(self.lease,pending,7),(True,0,8))
                self.assertEqual(self.calls[-1],('hold','lease',3))

    async def test_submit_timeout_still_sends_hold_and_cannot_resume(self):
        pending = self.pending()
        with self.assertRaisesRegex(RuntimeError,'BIMANUAL_SERVICE_TIMEOUT'):
            await self.node.stop_pending(self.lease,pending,7)
        self.assertEqual(self.calls[0],('hold','lease',3))
        self.assertFalse(pending.done())
        # execute.finally revokes before attempting any late submit settlement.
        self.assertTrue(await self.node.stop(self.lease,release=True))
        self.assertEqual(self.calls[-1],('release','lease',3))
        pending.set_result(S(accepted=True,revision=8))
        self.assertFalse(self.node.state.lease_active)

    async def test_unconfirmed_stop_and_wrong_lease_do_not_recover(self):
        self.node.state = self.state(7,'STOPPING')
        self.assertEqual(await self.node.stop_pending(self.lease,None,7),(False,0,7))
        self.assertTrue(self.node.event.called)
        self.clock.value = 0.
        self.node.state = self.state(8,lease_epoch=2)
        self.assertEqual(await self.node.stop_pending(self.lease,None,7),(False,0,7))

    async def test_replacement_before_stop_cannot_rebind_original_identity(self):
        self.node.state=self.state(9,instance_id='replacement')
        self.assertFalse(await self.node.stop(self.lease))
        diagnostic=__import__('json').loads(self.node.event.call_args.args[1])
        self.assertEqual(diagnostic['requested_instance_id'],'executor-instance')
        self.assertEqual(diagnostic['observed_instance_id'],'replacement')

    async def test_restart_during_pending_submit_cannot_confirm_old_stop(self):
        pending = self.pending()
        self.node.state = self.state(7, 'RUNNING')
        def restart():
            pending.set_result(S(accepted=True, revision=8))
            self.node.state = self.state(9, instance_id='new-executor')
        self.steps = [restart]
        self.assertEqual(await self.node.stop_pending(self.lease,pending,7),(False,1,8))

    async def test_normal_hold_requires_matching_instance_epoch_profile_and_mode(self):
        for changes in ({'instance_id':'new-executor'}, {'lease_epoch':9},
                        {'profile_hash':'other'}, {'simulated':False}, {'lease_active':False}):
            with self.subTest(changes=changes):
                self.clock.value = 0.
                self.node.state = self.state(7, 'STOPPING')
                self.steps = [lambda changes=changes:setattr(self.node,'state',self.state(8,**changes))]
                self.assertFalse(await self.node.stop(self.lease))

    async def test_revision_wait_cannot_cross_execution_instance_restart(self):
        self.node.state = self.state(7, 'STOPPING')
        self.steps = [lambda:setattr(self.node,'state',self.state(9,instance_id='new-executor'))]
        with self.assertRaisesRegex(RuntimeError,'REVISION_UNAVAILABLE'):
            await self.node.execution_revision(self.lease,8)

    async def test_rejected_stop_reply_records_the_original_failure(self):
        def rejected(request):
            future=self.pending()
            future.set_result(S(ok=False,code='STOP_REQUEST_FAILED'))
            return future
        self.node.control_client=S(call_async=rejected)
        self.assertFalse(await self.node.stop(self.lease))
        diagnostic=__import__('json').loads(self.node.event.call_args.args[1])
        self.assertEqual(diagnostic['reason'],'BIMANUAL_STOP_REPLY_REJECTED:STOP_REQUEST_FAILED')

    async def test_failed_stop_feedback_records_the_original_failure(self):
        self.node.state=self.state(8,code='STOP_FEEDBACK_TIMEOUT')
        self.assertFalse(await self.node.stop(self.lease))
        diagnostic=__import__('json').loads(self.node.event.call_args.args[1])
        self.assertEqual(diagnostic['reason'],'BIMANUAL_EXECUTION_STOP_FAILED:STOP_FEEDBACK_TIMEOUT')

    async def test_release_does_not_use_idle_state_from_newer_unrelated_epoch(self):
        def unrelated_release(request):
            future = self.pending()
            future.set_result(S(ok=True))
            self.node.state = self.state(10, lease_active=False, lease_epoch=9)
            return future
        self.node.lease_client = S(call_async=unrelated_release)
        self.assertFalse(await self.node.stop(self.lease,release=True))

    async def test_auto_revoked_lease_still_waits_for_original_instance_measured_rest(self):
        for instance in ('executor-instance', 'new-executor'):
            with self.subTest(instance=instance):
                self.clock.value = 0.
                self.node.state = self.state(7, 'STOPPING', lease_active=False, lease_epoch=4)
                def already_revoked(request):
                    future = self.pending()
                    future.set_result(S(ok=False,code='INVALID_LEASE',lease_id='',epoch=4,
                                        profile_hash='profile'))
                    return future
                self.node.lease_client = S(call_async=already_revoked)
                self.steps = [lambda instance=instance:setattr(self.node,'state',self.state(
                    8,lease_active=False,lease_epoch=4,instance_id=instance))]
                self.assertEqual(await self.node.stop(self.lease,release=True),
                                 instance=='executor-instance')

    async def test_pause_clutch_and_grasp_fence_old_worker_before_wait_and_reset_after(self):
        execute = next(n for n in NODE.body if getattr(n,'name','')=='execute')
        selected = {}
        for item in ast.walk(execute):
            if isinstance(item,ast.If):
                condition = ast.unparse(item.test)
                if condition in ("self.pause_control.phase == 'pausing'",
                                 'session.grasp_generation != last_grasp_generation',
                                 "session.mode == 'idle'"):
                    selected[condition] = item.body
        self.assertEqual(len(selected),3)
        for condition,body in selected.items():
            with self.subTest(condition=condition):
                text = '\n'.join(ast.unparse(n) for n in body)
                first_wait = text.index('await self.stop_pending')
                for fence in ('worker_token += 1','self.worker_result = None',"awaiting = ''"):
                    self.assertLess(text.index(fence),first_wait)
                pending = self.pending()
                self.node.state = self.state(7,'RUNNING')
                self.node.worker_result = ('solve',41,'old')
                def late():
                    pending.set_result(S(accepted=True,revision=8))
                    self.node.state = self.state(9)
                    self.node.worker_result = ('solve',41,'late')
                self.steps = [late]
                prefix = ast.parse('async def branch(self,lease,pending,session,reference,goal):\n'
                    '    accepted=0\n    submit_revision=7\n    worker_token=41\n'
                    '    awaiting="solve"\n    pending_reference=True\n'
                    '    for _ in (0,):\n        pass\n'
                    '    return accepted,submit_revision,worker_token,awaiting,pending,pending_reference\n')
                prefix.body[0].body[-2].body = body
                namespace = {'IndependentArmsSession':type('IndependentArmsSession',(),{}),
                             'json':__import__('json'),'end_deferral':Mock()}
                exec(compile(ast.fix_missing_locations(prefix),str(SOURCE),'exec'),namespace)
                session = S(reanchor=Mock(),grasp_mode='tool',grasp_generation=1,mode='idle')
                reference = S(reset=Mock())
                result = await namespace['branch'](self.node,self.lease,pending,session,
                    reference,S(request=S(task_id='test')))
                self.assertEqual(result,(1,9,42,'',None,None))
                self.assertIsNone(self.node.worker_result)
                reference.reset.assert_called_once()

    def test_motion_uses_revision_and_finally_revokes_before_late_receipt(self):
        execute = next(n for n in NODE.body if getattr(n,'name','')=='execute')
        motion = next(n for n in ast.walk(execute) if isinstance(n,ast.Call) and
            isinstance(n.func,ast.Name) and n.func.id=='MotionCommand')
        self.assertEqual(ast.unparse(next(k.value for k in motion.keywords if
            k.arg=='expected_revision')),'submit_revision')
        final = next(n for n in execute.body if isinstance(n,ast.Try)).finalbody
        text = '\n'.join(ast.unparse(n) for n in final)
        self.assertLess(text.index('await self.stop(lease, release=True)'),
                        text.index('await self.wait(pending)'))


class StopRevisionFence(ExecutionFixture):
    def test_renewal_keeps_revision_and_execution_state_exports_matching_fields(self):
        revision = self.e.revision
        self.e.renew(self.m.lease_id,self.m.epoch,10.)
        self.assertEqual(self.e.revision,revision)
        path = SOURCE.with_name('execution_node.py')
        node = ast.parse(path.read_text())
        body = '\n'.join(ast.unparse(n) for n in ast.walk(node) if
                         isinstance(n,ast.Assign))
        for field in ('reply.revision = self.engine.revision',
                      'msg.revision = self.engine.revision',
                      'msg.lease_epoch = self.engine.epoch'):
            self.assertIn(field,body)

    def test_equal_timestamp_late_submit_is_rejected_after_hold_and_fresh_revision_resumes(self):
        before = self.e.revision
        late = replace(self.m,command_id='late',expected_revision=before)
        self.e.halt(10.)
        self.assertEqual(self.e.last_stamp,late.stamp)
        self.e.tick(10.01)  # Independent measured rest completes the hold.
        self.assertIsNone(self.e.stopping)
        with self.assertRaisesRegex(Rejected,'STALE_REVISION'):
            self.e.submit(late,10.01)
        self.assertIsNone(self.e.active)
        self.assertTrue(self.e.lease_id)
        current = replace(late,command_id='fresh',expected_revision=self.e.revision)
        self.e.submit(current,10.01)
        self.assertEqual(self.e.active.command_id,'fresh')
        self.e.halt(10.02,revoke=True)
        with self.assertRaisesRegex(Rejected,'INVALID_LEASE'):
            self.e.submit(replace(current,command_id='revoked'),10.02)


PICO_SOURCE=package_share('snowbot_teleop')/'snowbot_teleop/node.py'


class PicoStopEvidence(unittest.IsolatedAsyncioTestCase,ExecutionFixture):
    async def test_failed_current_request_keeps_original_failure_after_repeated_waits(self):
        request=self.node.begin_stop_wait('original',self.lease.epoch,True)
        with self.assertRaisesRegex(RuntimeError,'EXECUTION_STOP_FAILED'):
            self.node.stop_unconfirmed(request,'EXECUTION_STOP_FAILED')
        for expired in (False,True):
            if expired:self.clock=request['deadline']+1.
            with self.assertRaisesRegex(RuntimeError,'EXECUTION_STOP_FAILED'):
                await self.node.stopped(request)
        self.assertEqual(request['failure_reason'],'EXECUTION_STOP_FAILED')
        with self.assertRaisesRegex(RuntimeError,'EXECUTION_STOP_FAILED'):
            self.node.begin_stop_wait('replacement',self.lease.epoch,True)
        failures=[c for c in self.node.event.call_args_list if c.args[0]=='PICO_STOP_UNCONFIRMED']
        self.assertEqual(len(failures),1)

    async def test_stale_request_remains_distinct_and_cannot_fail_new_request(self):
        old=self.node.begin_stop_wait('original',self.lease.epoch,True)
        current=self.node.stop_request=dict(old)
        for expired in (False,True):
            if expired:self.clock=old['deadline']+1.
            with self.assertRaisesRegex(RuntimeError,'REQUEST_NOT_CURRENT'):
                await self.node.stopped(old)
        self.assertIs(self.node.stop_request,current);self.assertFalse(current['failed'])
        self.node.control.record_fault.assert_called_with('PICO_STOP_UNCONFIRMED:REQUEST_NOT_CURRENT',
                                                         self.clock,kind='request')

    """Actual Pico stop/feedback methods and execution service handlers, no ROS."""
    def setUp(self):
        ExecutionFixture.setUp(self)
        self.clock=10.;self.pending=[];self.overrides={};self.rpc_delay=.02
        self.rpc_hook=None
        self.e.steady=lambda:self.clock
        tree=ast.parse(PICO_SOURCE.read_text())
        node=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='PicoNode')
        selected={'wait','reading','feedback','begin_stop_wait','stop_unconfirmed','stopped',
                  'cancel_tool','release_lease','tool_feedback','follow','move'}
        common=SOURCE.with_name('common.py')
        rest=next(n for n in ast.parse(common.read_text()).body
                  if isinstance(n,ast.FunctionDef) and n.name=='joints_at_rest')
        namespace=dict(time=S(monotonic=lambda:self.clock),json=__import__('json'),
            math=__import__('math'),seconds=lambda stamp:stamp,
            PicoFeedback=__import__('snowbot_teleop.control',fromlist=['PicoFeedback']).PicoFeedback,
            TeleopSession=S(Goal=lambda **kw:S(**kw)),Superseded=RuntimeError,
            stop_feedback_matches=stop_feedback_matches,
            Lease=S(Request=lambda **kw:S(**kw)),ControlExecution=S(Request=lambda **kw:S(**kw)))
        body=[rest,ast.ClassDef(name='ActualPico',bases=[],keywords=[],
             body=[n for n in node.body if getattr(n,'name','') in selected],decorator_list=[])]
        exec(compile(ast.fix_missing_locations(ast.Module(body=body,type_ignores=[])),str(PICO_SOURCE),'exec'),namespace)
        self.node=n=namespace['ActualPico']()
        n.base=None
        n.profile,n.simulation,n.backend=self.p,True,'memory'
        n.tool_cfg={'feedback_max_age':.2};n.names=tuple(self.p.limits)
        n.groups=tuple(self.p.groups);n.prepare_position_tolerance=.035;n.position_contract=True
        n.submit=S(call_async=Mock(side_effect=AssertionError('no motion submission expected')))
        n.position_ops=S(modes=lambda **kw:('closed','closed'),stopped=lambda:True)
        n.control=S(phase='following',pending=S(token=1),teleop_mode='independent_arms',record_fault=Mock());n.handoff_complete=True
        n.current=Mock();n.instance_id='pico-memory'
        n.now=lambda:self.clock;n.pause=self.pause;n.event=Mock()
        n.stop_request=None;n.stop_deadline=None  # Old-source comparison only.
        n.tool_request=None;n.tool_goal=n.tool_result=None;n.tool_phase='inactive'
        self.lease=S(lease_id=self.e.lease_id,epoch=self.e.epoch)
        n.lease=(self.lease,'original') if hasattr(n,'begin_stop_wait') else self.lease
        # The actual lease and control handlers operate the actual memory executor.
        execution=SOURCE.with_name('execution_node.py')
        definition=next(n for n in ast.parse(execution.read_text()).body
                        if isinstance(n,ast.ClassDef) and n.name=='ExecutionNode')
        methods=[n for n in definition.body if getattr(n,'name','') in ('lease','control')]
        service_ns={'Rejected':Rejected}
        exec(compile(ast.fix_missing_locations(ast.Module(body=[ast.ClassDef(name='Services',
            bases=[],keywords=[],body=methods,decorator_list=[])],type_ignores=[])),str(execution),'exec'),service_ns)
        self.services=service_ns['Services']()
        self.services.engine,self.services.robot_io,self.services.profile=self.e,self.io,self.p
        self.services.now=lambda:self.clock;self.services.closing=False
        n.stop_client=S(call_async=lambda req:self.rpc('control',req))
        n.leases=S(call_async=lambda req:self.rpc('lease',req))
        self.update()

    def update(self):
        self.e.tick(self.clock)
        fb=self.e.feedback
        event=self.e.events[-1] if self.e.events else S(state='IDLE',code='',command_id='')
        state='STOPPING' if self.e.stopping else 'RUNNING' if self.e.active else event.state
        data=dict(instance_id='original',profile_hash=self.p.digest,simulated=True,device_backend='memory',
            feedback_stamp=fb.stamp,lease_epoch=self.e.epoch,lease_active=bool(self.e.lease_id),
            state=state,code=event.code,stop_failures=list(self.io.stop_failures),
            reference=S(name=list(self.e.reference_names)),
            joints=S(name=list(fb.positions),position=list(fb.positions.values()),
                     velocity=[fb.joint_velocities[n] for n in fb.positions]))
        data.update(self.overrides);self.node.state=S(**data)

    def future(self,value,delay=0.):
        f=asyncio.get_running_loop().create_future()
        if delay:self.pending.append((self.clock+delay,f,value))
        else:f.set_result(value)
        return f

    def rpc(self,method,request):
        reply=getattr(self.services,method)(request,S())
        if self.rpc_hook:self.rpc_hook(method,request)
        self.update()
        return self.future(reply,self.rpc_delay)

    async def pause(self):
        self.clock=round(self.clock+.02,9)
        self.update()
        for at,f,value in self.pending:
            if self.clock>=at and not f.done():f.set_result(value() if callable(value) else value)
        await asyncio.sleep(0)

    def tool(self):
        self.node.lease=None
        result=asyncio.get_running_loop().create_future()
        def cancel():
            response=self.rpc('lease',S(operation='release',lease_id=self.lease.lease_id,epoch=self.lease.epoch))
            result.set_result(S(result=S(code='CANCELED')))
            return response
        goal=S(accepted=True,cancel_goal_async=cancel,get_result_async=lambda:result)
        self.node.tool_goal,self.node.tool_result=goal,result
        self.node.tool_request=dict(instance_id='original',epoch=self.lease.epoch,token=1,
            future=self.future(goal),canceling=False)
        return goal,result

    async def rejected_acquisition_code(self,reply,*,instance_id='original',epoch=None,late=False,replacement=False):
        # Execute the complete production Action method, stopping at its actual
        # explicit acquire rejection. No ROS Action server or hardware endpoint.
        method=next(n for n in NODE.body if getattr(n,'name','')=='execute')
        namespace=dict(json=__import__('json'),time=S(monotonic=lambda:self.clock),
            seconds=lambda stamp:stamp,ToolPoseSession=lambda *a,**kw:S(identity='session'),
            AcceptedReference=lambda *a,**kw:S(),
            TeleopAction=S(Result=lambda **kw:S(code='',success=False,**kw)),
            Lease=S(Request=lambda **kw:S(**kw)))
        exec(compile(ast.fix_missing_locations(ast.Module(body=[ast.ClassDef(
            name='ActualAction',bases=[],keywords=[],body=[method],decorator_list=[])],
            type_ignores=[])),str(SOURCE),'exec'),namespace)
        n=namespace['ActualAction']()
        n.mode_state=S(mode='bimanual_tool',generation=1,changed_at=self.clock)
        n.grasp_continuation=None;n.input_source='tool_pose';n.cfg={}
        n.profile=self.p;n.simulation=True;n.groups=['arm'];n.stop_unconfirmed=False
        n.state=S(instance_id=instance_id,lease_epoch=self.e.epoch if epoch is None else epoch)
        n.lease_client=S(call_async=lambda _:reply);n.cleanup_event=Mock()
        waits=0
        async def wait(value):
            nonlocal waits
            waits+=1
            if replacement:n.state.instance_id='replacement'
            if late and waits==1:raise RuntimeError('BIMANUAL_SERVICE_TIMEOUT')
            return value
        n.wait=wait
        goal=S(request=S(task_id='denied'),succeed=Mock(),abort=Mock(),canceled=Mock())
        result=await n.execute(goal)
        goal.abort.assert_called_once()
        return result.code

    async def denied_tool(self,*,code=None,delay=0.):
        await self.node.release_lease();await self.pause()
        epoch=self.e.epoch
        reply=S(ok=False,code='COMMANDS_DISABLED',lease_id='',epoch=epoch,profile_hash=self.p.digest)
        if code is None:code=await self.rejected_acquisition_code(reply)
        result=self.future(S(result=S(code=code)),delay)
        goal=S(accepted=True,get_result_async=lambda:result,
               cancel_goal_async=lambda:self.future(S()))
        self.node.tool_goal,self.node.tool_result=goal,result
        self.node.tool_request=dict(instance_id='original',epoch=epoch+1,token=1,
            future=self.future(goal),canceling=False)
        return epoch

    async def test_accepted_action_explicit_no_grant_confirms_original_baseline_rest(self):
        epoch=await self.denied_tool()
        self.assertTrue(self.node.tool_result.result().result.code.startswith('BIMANUAL_LEASE_NOT_ACQUIRED:'))
        await self.node.cancel_tool()
        self.assertEqual(self.e.epoch,epoch)
        self.assertIsNone(self.node.stop_request);self.assertIsNone(self.node.tool_request)
        self.assertTrue(self.node.feedback().stopped)

    async def test_no_grant_marker_only_emitted_for_explicit_empty_matching_reply(self):
        for change in ('lease_id','epoch','profile_hash'):
            reply=S(ok=False,code='DENIED',lease_id='',epoch=self.e.epoch,profile_hash=self.p.digest)
            setattr(reply,change,{'lease_id':'other','epoch':self.e.epoch+1,'profile_hash':'other'}[change])
            code=await self.rejected_acquisition_code(reply)
            self.assertEqual(code,'DENIED')

    async def test_late_explicit_denial_emits_proof_but_replacement_cannot(self):
        reply=S(ok=False,code='DENIED',lease_id='',epoch=self.e.epoch,profile_hash=self.p.digest)
        code=await self.rejected_acquisition_code(reply,late=True)
        self.assertTrue(code.startswith('BIMANUAL_LEASE_NOT_ACQUIRED:'))
        code=await self.rejected_acquisition_code(reply,replacement=True)
        self.assertEqual(code,'DENIED')

    async def test_no_grant_proof_mismatch_and_plain_rejection_do_not_clear_stop(self):
        for change in ('plain','malformed','instance_id','epoch','profile_hash'):
            with self.subTest(change=change):
                self.setUp()
                epoch=await self.denied_tool()
                proof=dict(instance_id='original',epoch=epoch,profile_hash=self.p.digest,reason='DENIED')
                if change=='plain':code='DENIED'
                elif change=='malformed':code='BIMANUAL_LEASE_NOT_ACQUIRED:bad-json'
                else:
                    proof[change]=epoch+1 if change=='epoch' else 'other'
                    code='BIMANUAL_LEASE_NOT_ACQUIRED:'+__import__('json').dumps(proof)
                self.node.tool_result=self.future(S(result=S(code=code)))
                with self.assertRaisesRegex(RuntimeError,'PICO_STOP_UNCONFIRMED'):
                    await self.node.cancel_tool()
                self.assertTrue(self.node.stop_request['failed'])

    async def test_valid_no_grant_proof_cannot_replace_fresh_same_instance_rest(self):
        for changes in (dict(instance_id='replacement'),dict(feedback_stamp=0.),
                        dict(lease_epoch=99),dict(state='STOPPING'),dict(stop_failures=['hold'])):
            with self.subTest(changes=changes):
                self.setUp();await self.denied_tool()
                self.overrides=changes;self.update()
                with self.assertRaisesRegex(RuntimeError,'PICO_STOP_UNCONFIRMED'):
                    await self.node.cancel_tool()
                self.assertIsNotNone(self.node.tool_request)

    async def test_late_no_grant_result_cannot_extend_original_stop_deadline(self):
        await self.denied_tool(delay=7.)
        started=self.clock
        with self.assertRaisesRegex(RuntimeError,'PICO_STOP_UNCONFIRMED'):
            await self.node.cancel_tool()
        self.assertEqual(self.node.stop_request['deadline'],started+6.)
        self.assertTrue(self.node.stop_request['failed'])
        while self.clock<started+7.1:await self.pause()
        self.assertFalse(self.node.feedback().healthy)

    async def test_pico_hold_keeps_lease_until_delayed_feedback_rest(self):
        calls=[]
        self.rpc_hook=lambda method,request:calls.append((method,request.operation))
        original_pause=self.node.pause
        async def delayed_rest():
            await original_pause()
            joints=self.node.state.joints
            speed=.02 if self.clock<12.5 else .007
            self.node.state.joints=S(name=joints.name,position=joints.position,
                                     velocity=[speed]*len(joints.name))
        self.node.pause=delayed_rest
        await delayed_rest()
        await self.node.release_lease()
        self.assertGreater(self.clock,12.5)
        self.assertGreaterEqual(calls.count(('lease','renew')),3)
        self.assertIsNone(self.node.lease)
        self.assertIsNone(self.node.stop_request)
        self.assertTrue(self.node.handoff_complete)
        self.assertFalse(self.e.lease_id)
        self.assertNotIn('LEASE_EXPIRED',[event.code for event in self.e.events])

    async def test_pico_hold_renewal_keeps_original_stop_deadline(self):
        original_pause=self.node.pause
        async def never_rest():
            await original_pause()
            joints=self.node.state.joints
            self.node.state.joints=S(name=joints.name,position=joints.position,
                                     velocity=[.02]*len(joints.name))
        self.node.pause=never_rest
        await never_rest()
        started=self.clock
        with self.assertRaisesRegex(RuntimeError,'PICO_STOP_UNCONFIRMED:DEADLINE_EXPIRED'):
            await self.node.release_lease()
        self.assertLessEqual(self.clock-started,6.05)
        self.assertIsNotNone(self.node.lease)
        self.assertTrue(self.node.stop_request['failed'])

    async def test_normal_hold_release_and_explicit_new_lease_need_no_restart(self):
        await self.node.release_lease()
        self.assertIsNone(self.node.lease)
        self.assertIsNone(self.node.stop_request)
        self.assertFalse(self.e.lease_id)
        self.assertTrue(self.node.handoff_complete)
        reply=self.services.lease(S(operation='acquire',owner='reentry',resources=['arm','hand']),S())
        self.assertTrue(reply.ok)
        self.node.lease=(reply,'original') if hasattr(self.node,'begin_stop_wait') else reply
        await self.node.release_lease()
        self.assertFalse(self.e.lease_id)
        self.assertTrue(self.node.feedback().stopped)

    async def test_intermediate_owned_stop_cannot_open_projection_handoff(self):
        request=self.node.begin_stop_wait('original',self.lease.epoch,True)
        await self.pause()
        await self.node.stopped(request,clear=False)
        self.assertFalse(self.node.handoff_complete)
        self.assertIs(self.node.stop_request,request)
        self.assertTrue(self.e.lease_id)

    async def test_restart_or_wrong_epoch_cannot_confirm_old_lease_stop(self):
        cases=(dict(instance_id='replacement',lease_active=False,lease_epoch=0,state='IDLE'),
               dict(lease_epoch=self.lease.epoch+8,lease_active=False,state='IDLE'),
               dict(profile_hash='other',lease_active=False,state='IDLE'),
               dict(simulated=False,lease_active=False,state='IDLE'))
        for changes in cases:
            with self.subTest(changes=changes):
                self.setUp()
                self.rpc_hook=lambda method,req:self.overrides.update(changes)
                with self.assertRaisesRegex(RuntimeError,'PICO_STOP_UNCONFIRMED'):
                    await self.node.release_lease()
                self.assertIsNotNone(self.node.lease)
                self.assertFalse(self.node.feedback().stopped)

    async def test_canceled_tool_cannot_use_new_instance_idle_or_unrelated_epoch(self):
        for changes in (dict(instance_id='replacement'),dict(lease_epoch=self.lease.epoch+9)):
            with self.subTest(changes=changes):
                self.setUp();goal,_=self.tool()
                self.rpc_hook=lambda method,req:self.overrides.update(changes)
                with self.assertRaisesRegex(RuntimeError,'PICO_STOP_UNCONFIRMED'):
                    await self.node.cancel_tool()
                self.assertIs(self.node.tool_goal,goal)
                self.assertFalse(self.node.feedback().stopped)

    async def test_late_rpc_after_timeout_cannot_clear_failed_stop_or_rebind_new_idle(self):
        self.rpc_delay=3.
        with self.assertRaisesRegex(RuntimeError,'PICO_STOP_UNCONFIRMED'):
            await self.node.release_lease()
        first=self.node.stop_request
        while self.clock<13.1:await self.pause()
        self.overrides=dict(instance_id='replacement',lease_active=False,lease_epoch=0,state='IDLE')
        self.update()
        with self.assertRaisesRegex(RuntimeError,'PICO_STOP_UNCONFIRMED'):
            await self.node.cancel_tool()
        self.assertIs(self.node.stop_request,first)
        self.assertFalse(self.node.feedback().healthy)
        failures=[c for c in self.node.event.call_args_list if c.args[0]=='PICO_STOP_UNCONFIRMED']
        self.assertEqual(len(failures),1)

    async def test_old_goal_completion_does_not_clear_new_goal_or_confirm_its_stop(self):
        old,_=self.tool();new=S(accepted=True)
        def replace(method,request):
            self.node.tool_goal=new
            self.node.tool_result=self.future(S(result=S(code='CANCELED')))
            self.node.tool_request=dict(instance_id='original',epoch=99,token=2,canceling=False)
        self.rpc_hook=replace
        with self.assertRaisesRegex(RuntimeError,'PICO_STOP_UNCONFIRMED'):
            await self.node.cancel_tool()
        self.assertIs(self.node.tool_goal,new)
        self.assertIsNot(self.node.tool_goal,old)

    async def test_inner_unconfirmed_result_is_not_overridden_by_fresh_idle(self):
        self.tool()
        self.node.tool_result=self.future(S(result=S(code='CANCELED:STOP_UNCONFIRMED')))
        # Cancellation callback still has its separate exact result future.
        with self.assertRaisesRegex(RuntimeError,'PICO_STOP_UNCONFIRMED'):
            await self.node.cancel_tool()
        self.assertIsNotNone(self.node.stop_request)

    async def test_already_revoked_lease_can_confirm_only_its_same_instance_next_epoch(self):
        self.e.halt(self.clock,revoke=True);self.update()
        await self.node.release_lease()
        self.assertIsNone(self.node.lease)
        self.assertIsNone(self.node.stop_request)

    async def test_disconnected_state_times_out_and_late_reconnection_does_not_rearm(self):
        self.rpc_hook=lambda method,request:self.overrides.update(feedback_stamp=0.)
        with self.assertRaisesRegex(RuntimeError,'PICO_STOP_UNCONFIRMED'):
            await self.node.release_lease()
        deadline=self.node.stop_request['deadline']
        self.assertGreaterEqual(self.clock,deadline)
        self.overrides={};self.update()
        self.assertFalse(self.node.feedback().healthy)
        with self.assertRaisesRegex(RuntimeError,'PICO_STOP_UNCONFIRMED'):
            await self.node.cancel_tool()

    async def test_same_goal_cancel_confirms_and_old_feedback_cannot_change_new_context(self):
        self.tool();old=self.node.tool_request
        await self.node.cancel_tool()
        self.assertIsNone(self.node.tool_goal)
        self.assertIsNone(self.node.stop_request)
        self.node.tool_request=dict(token=2,canceling=False)
        self.node.tool_phase='capturing'
        self.node.tool_feedback(S(feedback=S(mode='follow')),old)
        self.assertEqual(self.node.tool_phase,'capturing')

    async def late_goal(self,delay):
        self.services.lease(S(operation='release',lease_id=self.lease.lease_id,epoch=self.lease.epoch),S())
        self.update();self.node.lease=None
        await self.pause();await self.pause()
        result=asyncio.get_running_loop().create_future()
        acquired=[];cancels=[]
        def cancel():
            cancels.append(self.clock)
            lease=acquired[0]
            reply=self.rpc('lease',S(operation='release',lease_id=lease.lease_id,epoch=lease.epoch))
            if not result.done():result.set_result(S(result=S(code='CANCELED')))
            return reply
        goal=S(accepted=True,cancel_goal_async=cancel,get_result_async=lambda:result)
        def accepted():
            acquired.append(self.services.lease(S(operation='acquire',owner='tool',resources=['arm','hand']),S()))
            self.update();return goal
        future=self.future(accepted,delay)
        self.node.tools=S(send_goal_async=lambda *a,**kw:future)
        with self.assertRaisesRegex(RuntimeError,'PICO_RPC_TIMEOUT'):
            await self.node.follow(S(token=1))
        return cancels

    async def test_late_action_acceptance_is_settled_and_canceled_before_stop_confirmation(self):
        cancels=await self.late_goal(3.)
        await self.node.cancel_tool()
        self.assertTrue(cancels)
        self.assertFalse(self.e.lease_id)
        self.assertIsNone(self.node.stop_request)
        self.assertIsNone(self.node.tool_goal)

    async def test_action_acceptance_after_stop_deadline_is_canceled_but_does_not_recover(self):
        cancels=await self.late_goal(10.)
        with self.assertRaisesRegex(RuntimeError,'PICO_STOP_UNCONFIRMED'):
            await self.node.cancel_tool()
        request=self.node.stop_request
        while self.clock<20.1:await self.pause()
        self.assertTrue(cancels)
        self.assertFalse(self.e.lease_id)
        self.assertIs(self.node.stop_request,request)
        self.assertFalse(self.node.feedback().healthy)

    async def test_unknown_grant_is_settled_before_cleanup_and_never_rebound_to_new_instance(self):
        for replacement in (False,True):
            with self.subTest(replacement=replacement):
                self.setUp()
                self.services.lease(S(operation='release',lease_id=self.lease.lease_id,epoch=self.lease.epoch),S())
                self.update()
                reply=self.services.lease(S(operation='acquire',owner='pending-grant',resources=['arm','hand']),S())
                future=self.future(reply,3.)
                self.node.lease=(future,'original')
                if replacement:self.overrides=dict(instance_id='replacement',lease_active=False,lease_epoch=0,state='IDLE')
                if replacement:
                    with self.assertRaisesRegex(RuntimeError,'PICO_STOP_UNCONFIRMED'):
                        await self.node.release_lease()
                    self.assertIsNotNone(self.node.lease)
                else:
                    await self.node.release_lease()
                    self.assertIsNone(self.node.lease)
                    self.assertFalse(self.e.lease_id)

    async def test_unknown_grant_without_reply_keeps_original_context_and_stop_budget(self):
        future=asyncio.get_running_loop().create_future()
        self.node.lease=(future,'original')
        started=self.clock
        with self.assertRaisesRegex(RuntimeError,'PICO_STOP_UNCONFIRMED'):
            await self.node.release_lease()
        self.assertLessEqual(self.clock-started,6.04)
        self.assertIs(self.node.lease[0],future)
        self.assertFalse(self.node.feedback().healthy)

    async def test_production_move_cleans_up_late_grant_before_reporting_rpc_timeout(self):
        self.services.lease(S(operation='release',lease_id=self.lease.lease_id,epoch=self.lease.epoch),S())
        self.node.lease=None;self.update()
        self.rpc_hook=lambda method,request:setattr(self,'rpc_delay',3. if request.operation=='acquire' else .02)
        await self.pause();await self.pause()
        with self.assertRaisesRegex(RuntimeError,'^PICO_RPC_TIMEOUT$'):
            await self.node.move(S(token=1,operation='prepare'),dict.fromkeys(self.p.limits,0.),1.)
        self.assertIsNone(self.node.lease)
        self.assertIsNone(self.node.stop_request)
        self.assertFalse(self.e.lease_id)
        self.node.submit.call_async.assert_not_called()

    async def test_production_move_never_submits_after_instance_changes_during_acquisition(self):
        self.services.lease(S(operation='release',lease_id=self.lease.lease_id,epoch=self.lease.epoch),S())
        self.node.lease=None;self.update()
        await self.pause();await self.pause()
        self.rpc_hook=lambda method,req:self.overrides.update(instance_id='replacement')
        with self.assertRaisesRegex(RuntimeError,'PICO_STOP_UNCONFIRMED'):
            await self.node.move(S(token=1,operation='prepare'),dict.fromkeys(self.p.limits,0.),1.)
        self.node.submit.call_async.assert_not_called()
        self.assertIsNotNone(self.node.stop_request)

    async def test_replaced_active_executor_never_appears_as_healthy_stopped_before_cancel(self):
        self.tool()
        self.overrides=dict(instance_id='replacement',lease_epoch=0,lease_active=False,state='IDLE')
        self.update()
        feedback=self.node.feedback()
        self.assertFalse(feedback.healthy)
        self.assertFalse(feedback.stopped)
        self.assertEqual(feedback.fault,'PICO_EXECUTION_AUTHORITY_CHANGED')
