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


SOURCE = Path(__file__).resolve().parents[2]/'src/integration/bindu_runtime/bindu_runtime/bimanual_tool_node.py'
TREE = ast.parse(SOURCE.read_text())
NODE = next(n for n in TREE.body if isinstance(n, ast.ClassDef) and n.name == 'BimanualToolNode')


def actual_methods(clock):
    names = {'wait', 'begin_stop', 'execution_revision', 'stop_pending', 'stop'}
    methods = [n for n in NODE.body if getattr(n, 'name', '') in names]
    namespace = {'time':clock, 'seconds':lambda stamp:stamp,
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
        self.node.profile = S(stop_timeout=.3, digest='profile')
        self.node.simulation = True
        self.node.cfg = {'feedback_max_age':.2}
        self.node.names = ('a',)
        self.node.now = lambda:10.+self.clock.value
        self.node.event = Mock()
        self.node.pause_control = S(confirm_stopped=Mock())
        self.lease = S(lease_id='lease', epoch=3)
        self.calls = []
        self.steps = []
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
                             'json':__import__('json')}
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
