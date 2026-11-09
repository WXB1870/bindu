"""Pure-memory receipt deadlines; no ROS, solver process, or device endpoints."""
from bindu_contracts.resources import package_share
import queue
import ast
from pathlib import Path
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from bindu_kinematics.bimanual_worker import BimanualWorker


class GroundLiftTests(unittest.TestCase):
    def config(self):
        import json
        path=package_share('snowbot_bringup')/'config/robots/snowbot/bimanual_snowbot_real.json'
        return json.loads(path.read_text())['ground_recovery']

    def test_replay_geometry_height_and_relative_error_use_the_actual_flange_mount(self):
        import numpy as np
        from bindu_kinematics.tool_geometry import GroundLift
        config=self.config();lift=GroundLift(config)
        poses=np.array([np.eye(4),np.eye(4)]);poses[1,0,3]=.42
        poses[:,2,3]=.1
        shifted=poses.copy();shifted[:,2,3]+=.05
        self.assertAlmostEqual(lift.height(shifted[0])-lift.height(poses[0]),.05)
        self.assertEqual(lift.relative_error(shifted,poses),(0.,0.))
        shifted[1,0,3]+=.01
        self.assertAlmostEqual(lift.relative_error(shifted,poses)[0],.01)
        self.assertEqual(config['tool_id'],'g099-curved')
        self.assertGreater(max(np.linalg.norm(p) for p in config['points_in_left_flange_m']),1.)

    def test_actual_replay_start_common_upward_path_freezes_waist_and_preserves_pose(self):
        import json
        import numpy as np
        import xml.etree.ElementTree as ET
        from scipy.spatial.transform import Rotation
        from bindu_kinematics.tool_geometry import GroundLift
        from snowbot_hardware.session import REPLAY_NAMES
        root=ET.parse(package_share('snowbot_description')/'urdf/snowbot_control.urdf')
        joints={j.find('child').get('link'):j for j in root.findall('joint')}
        names=list(REPLAY_NAMES)
        manifest=json.loads((package_share('snowbot_bringup')/'config/robots/snowbot/primitive-library/manifest.json').read_text())
        start=np.array(manifest['nodes'][manifest['home_node']])
        limits={j.get('name'):(float(j.find('limit').get('lower')),float(j.find('limit').get('upper')))
                for j in joints.values() if j.find('limit') is not None and j.get('type')=='revolute'}
        bounds=np.array([limits[n] for n in names])
        def fk(q):
            values=dict(zip(names,q));cache={}
            def frame(link):
                if link in cache:return cache[link]
                if link not in joints:return np.eye(4)
                joint=joints[link];origin=joint.find('origin');local=np.eye(4)
                if origin is not None:
                    local[:3,3]=np.fromstring(origin.get('xyz','0 0 0'),sep=' ')
                    local[:3,:3]=Rotation.from_euler('xyz',np.fromstring(origin.get('rpy','0 0 0'),sep=' ')).as_matrix()
                if joint.get('type')=='revolute':
                    axis=np.fromstring(joint.find('axis').get('xyz'),sep=' ')
                    rotate=np.eye(4);rotate[:3,:3]=Rotation.from_rotvec(axis*values.get(joint.get('name'),0.)).as_matrix()
                    local=local@rotate
                cache[link]=frame(joint.find('parent').get('link'))@local
                return cache[link]
            return np.array([frame('arm_l_ft_sensor'),frame('arm_r_ft_sensor')])
        lift=GroundLift(self.config());check=Mock()
        self.assertLessEqual(lift.height(fk(start)[0]),lift.config['near_ground_m'])
        self.assertGreaterEqual(lift.height(fk(start)[0]),-.03)
        plan=lift.plan(start,bounds,list(range(2,len(start))),fk,check)
        target=np.array(plan['positions']);np.testing.assert_array_equal(target[:2],start[:2])
        np.testing.assert_allclose(fk(target)[:,:3,3]-fk(start)[:,:3,3],[[0,0,.05],[0,0,.05]],atol=1e-4)
        self.assertTrue(np.all(target>=bounds[:,0]) and np.all(target<=bounds[:,1]))
        self.assertGreaterEqual(plan['duration_s'],5.)
        check.assert_called_once()
        with self.assertRaisesRegex(ValueError,'BODY_COLLISION'):
            lift.plan(start,bounds,list(range(2,len(start))),fk,Mock(side_effect=ValueError('BODY_COLLISION')))

    def test_unreachable_upward_step_is_rejected_before_any_path_is_returned(self):
        import numpy as np
        from bindu_kinematics.tool_geometry import GroundLift
        lift=GroundLift(self.config())
        def fk(q):
            poses=np.array([np.eye(4),np.eye(4)]);poses[0,2,3]=q[0];poses[1,2,3]=q[1]
            return poses
        with self.assertRaisesRegex(ValueError,'IK_FAILED'):
            lift.plan(np.zeros(2),np.array([[0.,.01],[0.,.01]]),[0,1],fk,Mock())

    def test_worker_ground_requests_keep_tokens_and_do_not_reset_the_grasp(self):
        import sys
        import numpy as np
        from bindu_kinematics.bimanual_worker import _run
        solver=SimpleNamespace(last_constraint=None,last_timing=None,close=Mock(),
            plan_ground_recovery=Mock(return_value={'positions':[.1]}),
            ground_recovery_status=Mock(return_value=(None,None,{'height_m':.03})),
            rigid_grasp_q=np.array([.3]),rigid_grasp_reference=np.eye(4),tool=np.eye(4),independent_abi=Mock())
        checker=Mock(return_value=None)
        def plan(*args):
            solver.empty_collision_check([])
            return {'positions':[.1]}
        solver.plan_ground_recovery.side_effect=plan
        config=dict(ground_recovery=self.config(),urdf='/model/control.urdf',
                    demo=dict(grip_offset_m=[0.,0.,0.],display_urdf='display.urdf'))
        requests,results=queue.Queue(),queue.Queue()
        for operation,token in (('ground_recovery_path',7),('ground_recovery_status',8)):
            requests.put((operation,token,{'q':.2},None,[.3],None,'rigid',None,None,False))
        requests.put(None)
        with patch.dict(sys.modules,{'bindu_kinematics.bimanual':SimpleNamespace(BimanualQP=lambda *a:solver)}),\
                patch('bindu_kinematics.tool_geometry.make_collision_checker',return_value=checker) as create,\
                patch('bindu_kinematics.bimanual_worker.gc.collect'),patch('bindu_kinematics.bimanual_worker.gc.freeze'):
            _run(config,None,requests,results)
        self.assertEqual(results.get_nowait(),('ready',0,None))
        records=[]
        while not results.empty():records.append(results.get_nowait())
        self.assertIn(('result',7,{'positions':[.1]}),records)
        self.assertIn(('result',8,{'height_m':.03}),records)
        self.assertIs(solver.empty_collision_check,checker)
        self.assertIs(create.call_args.kwargs['tool_present'],False)
        np.testing.assert_array_equal(solver.rigid_grasp_reference,np.eye(4))
        np.testing.assert_array_equal(solver.rigid_grasp_q,[.3])

    def test_measured_axial_monitor_keeps_the_existing_free_roll_semantics(self):
        import numpy as np
        from bindu_kinematics.tool_geometry import GroundLift
        source=Path(__file__).resolve().parents[2]/'src/capabilities/bindu_kinematics/bindu_kinematics/bimanual.py'
        tree=ast.parse(source.read_text());node=next(n for n in tree.body if getattr(n,'name','')=='BimanualQP')
        method=next(n for n in node.body if getattr(n,'name','')=='ground_recovery_status')
        scope={'np':np,'__package__':'bindu_kinematics'}
        exec(compile(ast.Module(body=[method],type_ignores=[]),str(source),'exec'),scope)
        def poses(handle,q,n,left,right):
            left[:]=np.eye(4);right[:]=np.eye(4);right[0,3]=.4
            return True
        owner=SimpleNamespace(cfg=dict(grasp_reference_source='measured_axial'),ground_lift=GroundLift(self.config()),
            names=('joint',),model_names=('joint',),nq=1,handle=object(),
            _check_measured_limits=Mock(),_reference_error_limits=lambda:np.array([.25]),
            independent_abi=Mock(),_q=lambda feedback:np.array([feedback['joint']]),_ptr=lambda value:value,
            lib=SimpleNamespace(bq_flange_poses=poses),_axial_grip_error=Mock(return_value=(.03,.04)))
        _,_,status=scope['ground_recovery_status'](owner,{'joint':.1},np.array([.2]))
        owner._axial_grip_error.assert_called_once()
        self.assertEqual(status['relative_position_error_m'],.03)
        self.assertEqual(status['relative_rotation_error_rad'],.04)
        owner._check_measured_limits.assert_called_once()
        with self.assertRaisesRegex(ValueError,'REFERENCE_TRACKING_ERROR'):
            scope['ground_recovery_status'](owner,{'joint':.1},np.array([.4]))


class WorkerReceiptTests(unittest.TestCase):
    def test_reference_reset_is_bound_to_the_existing_execution_instance_and_epoch(self):
        source=Path(__file__).resolve().parents[2]/'src/integration/bindu_runtime/bindu_runtime/bimanual_tool_node.py'
        node=next(n for n in ast.parse(source.read_text()).body if isinstance(n,ast.ClassDef) and n.name=='BimanualToolNode')
        method=next(n for n in node.body if getattr(n,'name','')=='reset_grasp_reference')
        scope={};exec(compile(ast.Module(body=[method],type_ignores=[]),str(source),'exec'),scope)
        owner=SimpleNamespace(state=SimpleNamespace(instance_id='execution',lease_epoch=4),busy=False,
            worker=SimpleNamespace(reset_grasp_reference=Mock()),preflight_token=-3,
            preflight_status='ready',preflight_mode='bimanual_tool',preflight_grasp='right_axial')
        for instance,epoch in (('previous',4),('execution',3),('execution',5)):
            reply=scope['reset_grasp_reference'](owner,SimpleNamespace(execution_instance_id=instance,epoch=epoch),SimpleNamespace())
            self.assertFalse(reply.ok)
        owner.worker.reset_grasp_reference.assert_not_called()
        request=SimpleNamespace(execution_instance_id='execution',epoch=4)
        owner.busy=True
        self.assertFalse(scope['reset_grasp_reference'](owner,request,SimpleNamespace()).ok)
        owner.busy=False
        self.assertTrue(scope['reset_grasp_reference'](owner,request,SimpleNamespace()).ok)
        self.assertEqual(owner.preflight_status,'')
        self.assertEqual(owner.preflight_token,-4)
        owner.state.lease_epoch=6
        self.assertFalse(scope['reset_grasp_reference'](owner,request,SimpleNamespace()).ok)
        owner.worker.reset_grasp_reference.assert_called_once()

    def test_preparation_reference_reset_waits_for_next_admitted_request(self):
        worker=self.worker()
        worker.reset_grasp_reference()
        self.assertFalse(worker.submit('capture',1,{}))
        self.assertTrue(worker.reset_grasp_pending)
        worker.busy=False
        self.assertTrue(worker.submit('preflight_capture',2,{}))
        self.assertTrue(worker.requests.get_nowait()[-1])
        worker.busy=False
        self.assertTrue(worker.submit('capture',3,{}))
        self.assertFalse(worker.requests.get_nowait()[-1])

    def test_continuous_axial_does_not_request_six_dimensional_prediction(self):
        from bindu_contracts.teleoperation import tool_grasp_mode
        source=Path(__file__).resolve().parents[2]/'src/integration/bindu_runtime/bindu_runtime/bimanual_tool_node.py'
        keyword=next(n for n in ast.walk(ast.parse(source.read_text()))
                     if isinstance(n,ast.keyword) and n.arg=='rigid_reference')
        expression=compile(ast.Expression(body=keyword.value),str(source),'eval')
        for source,expected in (('measured_rigid',True),('measured_axial',False)):
            cfg=dict(grasp_reference_source=source,right_axial_grasp={},rigid_reference_prediction=True)
            value=eval(expression,dict(self=SimpleNamespace(trajectory_reference=True,cfg=cfg),
                mode=SimpleNamespace(mode='bimanual_tool'),
                session=SimpleNamespace(grasp_mode=tool_grasp_mode(cfg))))
            self.assertIs(value,expected)

    def test_executed_sample_belongs_only_to_its_solve_and_is_cleared_on_failure(self):
        from bindu_kinematics.bimanual_worker import _run
        requests,results=queue.Queue(),queue.Queue();observed=[]
        requests.put(('solve',1,{},None,[],.01,'rigid',None,(.13,),False))
        requests.put(('solve',2,{},None,[],.01,'rigid',None,None,False))
        requests.put(('capture',3,{},None,None,None,'rigid',None,(.99,),False))
        requests.put(None)
        solver=SimpleNamespace(last_geometry_step=None,last_constraint=None,last_timing=None,close=Mock())
        def solve(*args,**kwargs):
            observed.append(solver.execution_reference)
            raise ValueError('EXPECTED_REJECTION')
        solver.solve=solve
        solver.capture=lambda *args,**kw:observed.append(solver.execution_reference)
        with patch('bindu_kinematics.bimanual.BimanualQP',return_value=solver), \
                patch('bindu_kinematics.bimanual_worker.gc.collect'), \
                patch('bindu_kinematics.bimanual_worker.gc.freeze'):
            _run({},None,requests,results)
        self.assertEqual(observed,[(.13,),None,None])
        self.assertIsNone(solver.execution_reference)

    def test_gripper_body_path_keeps_all_joints_without_capturing_a_tool(self):
        import sys
        import numpy as np
        from bindu_kinematics.bimanual_worker import _run
        requests,results=queue.Queue(),queue.Queue()
        requests.put(('gripper_body_path',1,{'arm':0.,'finger':0.},
                      {'finger':.1},None,None,'rigid',None,None,False))
        requests.put(('gripper_body_path',2,{'arm':0.,'finger':0.},
                      {'finger':2.},None,None,'rigid',None,None,False))
        requests.put(None)
        solver=SimpleNamespace(tool=np.eye(4),model_names=('arm','finger'),names=('arm',),
                               capture=Mock(),close=Mock())
        check=Mock(return_value={'passed':True});factory=Mock(return_value=check)
        config={'urdf':'/tmp/model.urdf','demo':{'grip_offset_m':[0.,0.,0.],'display_urdf':'model.urdf'}}
        profile=SimpleNamespace(limits={'arm':(-1.,1.),'finger':(-1.,1.)})
        with patch.dict(sys.modules,{
                'bindu_kinematics.bimanual':SimpleNamespace(BimanualQP=Mock(return_value=solver)),
                'bindu_kinematics.tool_geometry':SimpleNamespace(make_collision_checker=factory)}), \
                patch('bindu_kinematics.bimanual_worker.gc.collect'), \
                patch('bindu_kinematics.bimanual_worker.gc.freeze'):
            _run(config,profile,requests,results)
        solver.capture.assert_not_called()
        self.assertFalse(factory.call_args.kwargs['tool_present'])
        samples=check.call_args.args[0]
        self.assertEqual(samples[-1]['q'],{'arm':0.,'finger':.1})
        replies=list(results.queue)
        self.assertIn(('result',1,{'passed':True}),replies)
        self.assertIn(('error',2,'PREPARE_JOINT_LIMIT'),replies)

    def worker(self, *, ready=True, busy=True):
        worker = BimanualWorker.__new__(BimanualWorker)
        worker.requests, worker.results = queue.Queue(1), queue.Queue(2)
        worker.process = SimpleNamespace(is_alive=lambda:True)
        worker.ready, worker.busy = ready, busy
        worker.reset_grasp_pending = False
        worker.error = ''
        worker.failure = ''
        worker.diagnostic = None
        worker.started = 0.
        worker.submitted = 10.
        worker.startup_timeout = 2.
        worker.timeout = .23
        return worker

    def poll(self, worker, now):
        with patch('bindu_kinematics.bimanual_worker.time.monotonic', return_value=now):
            return worker.poll()

    def test_queued_late_success_and_rejection_cannot_erase_request_deadline(self):
        for kind in ('result', 'error'):
            with self.subTest(kind=kind):
                worker = self.worker()
                worker.results.put((kind, 7, 'late'))
                with self.assertRaisesRegex(RuntimeError, '^BIMANUAL_WORKER_TIMEOUT$'):
                    self.poll(worker, 10.231)
                self.assertFalse(worker.ready)
                self.assertFalse(worker.submit('solve', 8, {}))
                self.assertTrue(worker.requests.empty())

    def test_late_ready_cannot_erase_startup_deadline(self):
        worker = self.worker(ready=False, busy=False)
        worker.results.put(('ready', 0, None))
        with self.assertRaisesRegex(RuntimeError, '^BIMANUAL_WORKER_TIMEOUT$'):
            self.poll(worker, 2.001)

    def test_timeout_stays_latched_when_late_result_arrives(self):
        worker = self.worker()
        with self.assertRaisesRegex(RuntimeError, '^BIMANUAL_WORKER_TIMEOUT$'):
            self.poll(worker, 10.231)
        worker.results.put(('result', 7, 'late'))
        for now in (10.24, 11., 100.):
            with self.assertRaisesRegex(RuntimeError, '^BIMANUAL_WORKER_TIMEOUT$'):
                self.poll(worker, now)
        self.assertEqual(worker.results.qsize(), 1)

    def test_in_budget_diagnostic_and_result_are_delivered_once(self):
        worker = self.worker()
        worker.results.put(('diagnostic', 7, {'scale':.5}))
        worker.results.put(('result', 7, 'accepted-candidate'))
        self.assertEqual(self.poll(worker, 10.22), ('result', 7, 'accepted-candidate'))
        self.assertEqual(worker.diagnostic, (7, {'scale':.5}))
        self.assertFalse(worker.busy)
        # Idle time does not expire a completed request.
        self.assertIsNone(self.poll(worker, 100.))

    def test_in_budget_operation_error_is_not_a_worker_fault(self):
        worker = self.worker()
        worker.results.put(('error', 7, 'QP_INFEASIBLE'))
        self.assertEqual(self.poll(worker, 10.1), ('error', 7, 'QP_INFEASIBLE'))
        self.assertTrue(worker.ready)
        with patch('bindu_kinematics.bimanual_worker.time.monotonic', return_value=10.11):
            self.assertTrue(worker.submit('capture', 8, {}))

    def test_startup_and_request_have_separate_receipt_budgets(self):
        worker = self.worker(ready=False, busy=False)
        worker.results.put(('ready', 0, None))
        self.assertIsNone(self.poll(worker, 1.99))
        self.assertTrue(worker.ready)
        self.assertIsNone(self.poll(worker, 50.))
        with patch('bindu_kinematics.bimanual_worker.time.monotonic', return_value=50.):
            self.assertTrue(worker.submit('capture', 1, {}))
        worker.results.put(('diagnostic', 1, {'scale':1.}))
        self.assertIsNone(self.poll(worker, 50.1))
        self.assertTrue(worker.busy)
        with self.assertRaisesRegex(RuntimeError, 'WORKER_TIMEOUT'):
            self.poll(worker, 50.24)

    def test_runtime_fault_emits_once_and_continues_waking_stop_waiters(self):
        source = Path(__file__).resolve().parents[2]/'src/integration/bindu_runtime/bindu_runtime/bimanual_tool_node.py'
        node = next(n for n in ast.parse(source.read_text()).body
                    if isinstance(n, ast.ClassDef) and n.name == 'BimanualToolNode')
        method = next(n for n in node.body if getattr(n, 'name', '') == 'poll_worker')
        namespace = {}
        exec(compile(ast.Module(body=[method], type_ignores=[]), str(source), 'exec'), namespace)
        worker = self.worker()
        owner = SimpleNamespace(waiters=set(), worker_error='', worker=worker, event=Mock())
        with patch('bindu_kinematics.bimanual_worker.time.monotonic', return_value=10.24):
            namespace['poll_worker'](owner)
            worker.results.put(('result', 7, 'late'))
            for _ in range(1000):
                waiter = Mock()
                waiter.done.return_value = False
                owner.waiters = {waiter}
                namespace['poll_worker'](owner)
                waiter.set_result.assert_called_once_with(None)
        owner.event.assert_called_once_with('BIMANUAL_WORKER_FAILED', 'BIMANUAL_WORKER_TIMEOUT')
        self.assertEqual(worker.results.qsize(), 1)

    def test_worker_errors_transport_diagnostics_and_never_reuse_previous_constraint(self):
        from bindu_kinematics.bimanual_worker import _run
        requests,results=queue.Queue(),queue.Queue()
        requests.put(('solve',7,{},None,[],.01,'rigid',None,None,False))
        requests.put(('capture',8,{},None,None,None,'rigid',None,None,False))
        requests.put(None)
        solver=SimpleNamespace(last_constraint=None,last_timing={'stage':'measured'},close=Mock())
        def solve(*args,**kw):
            solver.last_constraint=dict(code='BIMANUAL_MEASURED_GRASP_ERROR',
                stage='measured_grasp_points',position_error_m=.010024,position_limit_m=.01)
            raise ValueError('BIMANUAL_MEASURED_GRASP_ERROR')
        solver.solve=solve
        solver.capture=Mock(side_effect=ValueError('UNKNOWN_CAPTURE_FAILURE'))
        with patch('bindu_kinematics.bimanual.BimanualQP',return_value=solver),                patch('bindu_kinematics.bimanual_worker.gc.collect'),                patch('bindu_kinematics.bimanual_worker.gc.freeze'):
            _run({},None,requests,results)
        self.assertEqual(results.get_nowait(),('ready',0,None))
        first,reply=results.get_nowait(),results.get_nowait()
        self.assertEqual(reply,('error',7,'BIMANUAL_MEASURED_GRASP_ERROR'))
        worker=self.worker();worker.results.put(first);worker.results.put(reply)
        self.assertEqual(self.poll(worker,10.1),reply)
        self.assertEqual(worker.diagnostic[1]['constraint']['position_error_m'],.010024)
        second,reply=results.get_nowait(),results.get_nowait()
        self.assertEqual(second[1],8)
        self.assertIsNone(second[2]['constraint'])
        self.assertIsNone(second[2]['timing'])
        self.assertEqual(reply,('error',8,'UNKNOWN_CAPTURE_FAILURE'))
        self.assertTrue(results.empty());solver.close.assert_called_once()

    def test_readiness_preserves_rigid_grasp_but_leased_independent_capture_ends_it(self):
        from bindu_kinematics.bimanual_worker import _run
        requests,results=queue.Queue(),queue.Queue()
        requests.put(('independent_capture',-1,{},None,None,None,'rigid',None,None,False))
        requests.put(('capture',1,{},None,None,None,'independent:1',None,None,False))
        requests.put(None)
        solver=SimpleNamespace(capture_independent=Mock(return_value=[]),close=Mock())
        with patch('bindu_kinematics.bimanual.BimanualQP',return_value=solver), \
                patch('bindu_kinematics.bimanual_worker.gc.collect'), \
                patch('bindu_kinematics.bimanual_worker.gc.freeze'):
            _run({},None,requests,results)
        self.assertEqual(solver.capture_independent.call_args_list,[
            unittest.mock.call({},reference=None,preserve_rigid_grasp=True),
            unittest.mock.call({},reference=None)])

    def test_tool_readiness_previews_grasp_but_actual_capture_commits_it(self):
        from bindu_kinematics.bimanual_worker import _run
        requests,results=queue.Queue(),queue.Queue()
        requests.put(('preflight_capture',-1,{},None,None,None,'rigid',None,None,False))
        requests.put(('capture',1,{},None,None,None,'rigid',None,None,False))
        requests.put(None)
        solver=SimpleNamespace(capture=Mock(return_value=[]),close=Mock())
        with patch('bindu_kinematics.bimanual.BimanualQP',return_value=solver), \
                patch('bindu_kinematics.bimanual_worker.gc.collect'), \
                patch('bindu_kinematics.bimanual_worker.gc.freeze'):
            _run({},None,requests,results)
        self.assertEqual(list(results.queue),[('ready',0,None),('result',-1,[]),('result',1,[])])
        self.assertEqual(solver.capture.call_args_list,[
            unittest.mock.call({},'rigid',reference=None,commit_grasp=False),
            unittest.mock.call({},'rigid',reference=None)])

    def test_mode_readiness_dispatches_preview_with_fresh_feedback(self):
        source=Path(__file__).resolve().parents[2]/'src/integration/bindu_runtime/bindu_runtime/bimanual_tool_node.py'
        node=next(n for n in ast.parse(source.read_text()).body
                  if isinstance(n,ast.ClassDef) and n.name=='BimanualToolNode')
        method=next(n for n in node.body if getattr(n,'name','')=='preflight')
        namespace={'seconds':lambda stamp:stamp}
        exec(compile(ast.Module(body=[method],type_ignores=[]),str(source),'exec'),namespace)
        for mode,operation,source,grasp in (
                ('bimanual_tool','preflight_capture','measured_rigid','rigid'),
                ('bimanual_tool','preflight_capture','measured_axial','right_axial'),
                ('independent_arms','independent_capture','measured_axial','rigid')):
            with self.subTest(mode=mode,source=source):
                owner=SimpleNamespace(worker=SimpleNamespace(ready=True,busy=False,submit=Mock(return_value=True)),
                    worker_error='',busy=False,simulation=False,profile=SimpleNamespace(digest='profile'),
                    state=SimpleNamespace(profile_hash='profile',simulated=False,feedback_stamp=10.,
                        joints=SimpleNamespace(name=['joint'],position=[.8])),
                    now=lambda:10.01,cfg={'feedback_max_age':.2,'grasp_reference_source':source},axial_position_ready=lambda:False,
                    preflight_mode='',preflight_grasp='',preflight_status='',preflight_token=0)
                reply=namespace['preflight'](owner,SimpleNamespace(),mode)
                self.assertFalse(reply.success)
                self.assertEqual(reply.message,'BIMANUAL_PREFLIGHT_PENDING')
                owner.worker.submit.assert_called_once_with(operation,-1,{'joint':.8},grasp_mode=grasp)

    def test_dead_process_or_fatal_cannot_advertise_ready_or_accept_work(self):
        for cause in ('exit', 'fatal'):
            with self.subTest(cause=cause):
                worker = self.worker(busy=False)
                code = 'BIMANUAL_WORKER_EXITED'
                if cause == 'exit':
                    worker.process = SimpleNamespace(is_alive=lambda:False)
                else:
                    worker.results.put(('fatal', 0, 'ValueError: MODEL_INVALID'))
                    code = 'BIMANUAL_WORKER_FAILED:ValueError: MODEL_INVALID'
                for now in (10., 10.1):
                    with self.assertRaisesRegex(RuntimeError, '^'+code+'$'):
                        self.poll(worker, now)
                    self.assertFalse(worker.ready)
                    self.assertFalse(worker.submit('capture', 8, {}))

    def test_long_jittered_sequence_has_no_replayed_or_resurrected_result(self):
        worker = self.worker(busy=False)
        # Simulated 10,000 bounded requests, with dropped polls and jitter.
        for token in range(10000):
            start = 10.+token*.3
            with patch('bindu_kinematics.bimanual_worker.time.monotonic', return_value=start):
                self.assertTrue(worker.submit('solve', token, {}))
            self.assertEqual(worker.requests.get_nowait()[1], token)
            self.assertIsNone(self.poll(worker, start+.01))
            worker.results.put(('result', token, token))
            self.assertEqual(self.poll(worker, start+.02+(token%7)*.025), ('result', token, token))
            self.assertIsNone(self.poll(worker, start+.21))
        # A lost response then arriving after the budget must stay unusable.
        with patch('bindu_kinematics.bimanual_worker.time.monotonic', return_value=4000.):
            self.assertTrue(worker.submit('solve', 10000, {}))
        with self.assertRaisesRegex(RuntimeError, 'WORKER_TIMEOUT'):
            self.poll(worker, 4000.24)
        worker.results.put(('result', 10000, 'late'))
        with self.assertRaisesRegex(RuntimeError, 'WORKER_TIMEOUT'):
            self.poll(worker, 4000.25)


class PrepareLimitReferenceTests(unittest.TestCase):
    def test_inward_recovery_is_not_blocked_by_an_in_bounds_partner_residual(self):
        from bindu_kinematics.bimanual_worker import prepare_limit_reference
        measured={'r3':-.5999302864074707,'other':-.21214474737644196}
        held={'r3':-.5868355631828308,'other':-.25267574191093445}
        target={'r3':-.431745,'other':-.1}
        self.assertEqual(prepare_limit_reference(self.fixture(),tuple(held),measured,held,target,
                         .017453292519943295,.035),held)

    def test_both_real_recipes_add_one_degree_to_the_existing_all_axis_reserve(self):
        import json,math
        from pathlib import Path
        root=package_share('snowbot_bringup')/'config'
        for name in ('robots/snowbot/bimanual_snowbot_real.json','examples/snowbot/bimanual_snowbot_shovel.json'):
            cfg=json.loads((root/name).read_text())
            self.assertAlmostEqual(cfg['posture_reference_margin_rad'],.01+math.radians(1.))
            self.assertEqual(cfg['posture_policy']['reference'],'initial_capture')

    def test_limit_exit_moves_only_violating_axes_before_full_return(self):
        from bindu_kinematics.bimanual_worker import prepare_limit_exit_target,prepare_limit_reference
        profile=self.fixture();held={'r3':-.59,'other':.91}
        measured={'r3':-.60,'other':.86};final={'r3':-.43,'other':-.98}
        exit_target=prepare_limit_exit_target(profile,tuple(held),measured,held,final,.035)
        self.assertAlmostEqual(exit_target['r3'],profile.limits['r3'][0]+.035)
        self.assertEqual(exit_target['other'],held['other'])
        self.assertEqual(prepare_limit_reference(profile,tuple(held),measured,held,exit_target,.017,.035),held)
        profile.limits['r3']=(-2.61,.5817453292519943)
        held['r3']=.579782605;measured['r3']=.586484015;final['r3']=.431745
        exit_target=prepare_limit_exit_target(profile,tuple(held),measured,held,final,.035)
        self.assertAlmostEqual(exit_target['r3'],.5467453292519943)
        self.assertEqual(exit_target['other'],.91)
        prepare_limit_reference(profile,tuple(held),measured,held,exit_target,.017,.035)

    def fixture(self):
        from types import SimpleNamespace as S
        return S(limits={'r3':(-.5974532925199433,2.611745329251994),'other':(-1.,1.)},
                 feedback_tolerances={'joint_position':.002})

    def test_recorded_excursion_uses_explicit_held_reference_without_clipping_feedback(self):
        from bindu_kinematics.bimanual_worker import prepare_limit_reference
        measured={'r3':-.6073604822158813,'other':.1}
        held={'r3':-.596094062733137,'other':.09};target={'r3':-.431745,'other':.2}
        result=prepare_limit_reference(self.fixture(),tuple(held),measured,held,target,.017453292519943295,.035)
        self.assertEqual(result,held)
        self.assertEqual(measured['r3'],-.6073604822158813)
        self.assertIsNot(result,held)

    def test_large_excursion_unconfirmed_layout_and_outward_return_are_rejected(self):
        from bindu_kinematics.bimanual_worker import prepare_limit_reference
        profile=self.fixture();names=('r3','other')
        for measured,held,target,cap in [
            ({'r3':-.63,'other':.1},{'r3':-.59,'other':.1},{'r3':-.43,'other':.1},.017),
            ({'r3':-.607,'other':.1},{'r3':-.598,'other':.1},{'r3':-.43,'other':.1},.017),
            ({'r3':-.607,'other':.1},{'r3':-.59,'other':.1},{'r3':-.595,'other':.1},.017),
            ({'r3':-.607,'other':.1},{'r3':-.55,'other':.1},{'r3':-.43,'other':.1},.017),
            ({'r3':float('nan'),'other':.1},{'r3':-.59,'other':.1},{'r3':-.43,'other':.1},.017),
            ({'r3':-.607,'other':.1},{'r3':-.59},{'r3':-.43,'other':.1},.017),
            ({'r3':-.607,'other':.1},{'r3':-.59,'other':.1},{'r3':-.43,'other':.1},0.)]:
            with self.subTest(measured=measured,held=held,target=target),self.assertRaises(ValueError):
                prepare_limit_reference(profile,names,measured,held,target,cap,.035)


if __name__ == '__main__':
    unittest.main()
