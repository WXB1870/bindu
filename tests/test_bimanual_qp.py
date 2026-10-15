"""Native integration gate; skips when the optional QP package is not installed."""
import json
import tempfile
import unittest
from pathlib import Path
import numpy as np
from ament_index_python.packages import PackageNotFoundError, get_package_prefix
from bindu_contracts.profile import Profile
from bindu_kinematics.bimanual import BimanualQP, NativeConfig, check_abi
from unittest.mock import Mock, patch
from dataclasses import replace
import queue
import gc
import weakref
import time
from types import SimpleNamespace as S
from bindu_kinematics.bimanual_worker import BimanualWorker

ROOT=Path(__file__).resolve().parents[1]


def propose(solver, feedback, target, dt=.01, reference=None):
    if reference is None:
        reference = [feedback[n] for n in solver.names]
    return solver.solve(feedback, target, reference=reference, dt=dt)


class AbiLoadingTests(unittest.TestCase):
    def test_pointer_retains_temporary_array_without_cyclic_garbage(self):
        values=np.arange(4,dtype=np.float64)
        retained=weakref.ref(values)
        pointer=BimanualQP._ptr(values)
        del values
        gc.collect()
        self.assertIsNotNone(retained())
        pointer[2]=7.
        self.assertEqual(retained()[2],7.)
        del pointer
        gc.collect()
        self.assertIsNone(retained())
        values=np.ones(4)
        BimanualQP._ptr(values)  # Warm the ctypes pointer type cache.
        gc.collect()
        enabled=gc.isenabled()
        gc.disable()
        try:
            before=gc.get_count()[0]
            for _ in range(2000):BimanualQP._ptr(values)
            first=gc.get_count()[0]-before
            for _ in range(2000):BimanualQP._ptr(values)
            second=gc.get_count()[0]-before
            # Allow fixed ctypes bookkeeping, but no garbage growth per call.
            self.assertLess(first,16)
            self.assertLess(second-first,5)
        finally:
            if enabled:gc.enable()

    def test_worker_gc_preparation_precedes_ready_and_closes_native_handle(self):
        from bindu_kinematics.bimanual_worker import _run
        events=[]
        requests=Mock();requests.get.return_value=None
        results=S(put=lambda item:events.append(item[0]))
        solver=S(close=lambda:events.append('close'))
        with patch('bindu_kinematics.bimanual.BimanualQP',
                   side_effect=lambda *args:events.append('construct') or solver), \
                patch('bindu_kinematics.bimanual_worker.gc.collect',side_effect=lambda:events.append('collect')), \
                patch('bindu_kinematics.bimanual_worker.gc.freeze',side_effect=lambda:events.append('freeze')):
            _run({},None,requests,results)
        self.assertEqual(events,['construct','collect','freeze','ready','close'])

    def test_missing_and_incompatible_abi_rejected_before_binding(self):
        with self.assertRaisesRegex(ValueError, 'ABI_MISSING'):
            check_abi(object())
        lib = Mock()
        lib.bq_abi_version.return_value = 999
        with self.assertRaisesRegex(ValueError, 'ABI_MISMATCH'):
            check_abi(lib)
        lib.bq_create.assert_not_called()

    def test_path_contexts_preload_once_but_recheck_each_measured_path(self):
        from bindu_kinematics.bimanual_worker import _run
        cfg={'urdf':'/tmp/model.urdf','demo':{'display_urdf':'display.urdf','grip_offset_m':[0.,0.,0.]}}
        profile=S(limits={'joint':(-1.,1.)})
        solver=S(tool=np.eye(4),model_names=['joint'],names=['joint'],close=Mock(),
                 capture=Mock(return_value=np.eye(4)))
        requests=Mock();requests.get.side_effect=[
            ('gripper_path',i,{'joint':start},{'joint':end},None,None,'rigid',None)
            for i,start,end in ((1,0.,.01),(2,.01,.02),(3,.02,.04))]+[None]
        results=[];paths=[];constructed=[]
        def factory(*args,**kwargs):
            constructed.append(kwargs)
            def check(samples):
                paths.append(samples)
                if any(s['q']['joint']>.03 for s in samples):raise ValueError('GRASP_PENETRATION')
                return {'samples':len(samples)}
            return check
        with patch('bindu_kinematics.bimanual.BimanualQP',return_value=solver), \
                patch('bindu_kinematics.tool_geometry.make_collision_checker',side_effect=factory), \
                patch('bindu_kinematics.bimanual_worker.gc.collect'), \
                patch('bindu_kinematics.bimanual_worker.gc.freeze'):
            _run(cfg,profile,requests,S(put=results.append),True)
        self.assertEqual(constructed,[{'conservative_convex':True,'tool_present':False},
                                     {'conservative_convex':True,'tool_present':True}])
        self.assertEqual([r[0] for r in results],['ready','result','result','error'])
        self.assertIn('GRASP_PENETRATION',results[-1][2])
        self.assertEqual([p[0]['q']['joint'] for p in paths],[0.,.01,.02])
        self.assertEqual(solver.capture.call_count,3)
        solver.close.assert_called_once()

    def test_worker_startup_and_request_deadlines_are_bounded(self):
        worker = BimanualWorker.__new__(BimanualWorker)
        worker.results = queue.Queue()
        worker.process = Mock()
        worker.process.is_alive.return_value = True
        worker.error = ''
        worker.ready = worker.busy = False
        worker.started = 1.
        worker.startup_timeout = 2.
        worker.timeout = .07
        worker.submitted = 3.
        with patch('bindu_kinematics.bimanual_worker.time.monotonic', return_value=3.01):
            with self.assertRaisesRegex(RuntimeError, 'WORKER_TIMEOUT'):
                worker.poll()
        worker.ready = worker.busy = True
        with patch('bindu_kinematics.bimanual_worker.time.monotonic', return_value=3.071):
            with self.assertRaisesRegex(RuntimeError, 'WORKER_TIMEOUT'):
                worker.poll()


class NativeBimanual(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        try: get_package_prefix('bindu_bimanual_qp')
        except PackageNotFoundError: raise unittest.SkipTest('optional native QP not installed')
        cls.config=json.loads((ROOT/'src/integration/bindu_runtime/config/bimanual_g1_sim.json').read_text())
        cls.config['urdf']=str(ROOT/'src/hardware/bindu_description/urdf/g1_provisional.urdf')
        cls.profile=Profile.load(ROOT/'src/integration/bindu_runtime/config/g1_provisional_sim.json')
        cls.pose=json.loads((ROOT/'src/integration/bindu_runtime/config/bimanual_g1_pose.json').read_text())['joint_positions_rad']

    def setUp(self):
        self.qp=BimanualQP(self.config,self.profile)
        self.addCleanup(self.qp.close)
        self.feedback=dict.fromkeys(self.qp.model_names,0.)
        self.feedback.update(self.pose)

    def test_preparation_native_and_geometry_timeout_have_separate_diagnostics(self):
        for component,stage in (('input','native'),('native','native'),('geometry','validation')):
            with self.subTest(component=component):
                self.feedback.update(self.pose)
                target=self.qp.capture(self.feedback)
                if component=='input':
                    original=self.qp._q
                    def delayed(*args):
                        time.sleep(.04);return original(*args)
                    guard=patch.object(self.qp,'_q',side_effect=delayed)
                elif component=='native':
                    name='bq_solve_tracking' if self.qp.tracking else 'bq_solve'
                    original=getattr(self.qp.lib,name)
                    def delayed(*args):
                        ok=original(*args);time.sleep(.04);return ok
                    guard=patch.object(self.qp.lib,name,side_effect=delayed)
                else:
                    guard=patch.object(self.qp,'check_path',side_effect=lambda *args:time.sleep(.04))
                with guard,self.assertRaisesRegex(ValueError,'BIMANUAL_QP_TIMEOUT:'+stage):
                    propose(self.qp,self.feedback,target)
                timing=self.qp.last_timing
                self.assertFalse(self.qp.captured)
                self.assertEqual(timing['stage'],stage)
                self.assertGreater(timing['wall_us'],self.config['solve_timeout_us'])
                self.assertGreater(timing['wall_us']-timing['thread_cpu_us'],30000.)
                if component!='input':
                    self.assertGreater(timing['native_wall_us'] if stage=='native' else timing['geometry_wall_us'],40000.)

    def test_late_result_packing_cannot_commit_feedforward_or_succeed(self):
        self.qp.close()
        self.qp=BimanualQP(dict(self.config,tool_velocity_feedforward=True),self.profile)
        self.addCleanup(self.qp.close)
        self.feedback.update(self.pose)
        target=self.qp.capture(self.feedback)
        class SlowPacking(np.ndarray):
            iterations=0
            def __iter__(self):
                self.iterations+=1
                if self.iterations==2:time.sleep(.04)
                return super().__iter__()
        allocate=np.empty
        def output(shape,*args,**kw):
            array=allocate(shape,*args,**kw)
            return array.view(SlowPacking) if shape==len(self.qp.names) else array
        with patch('bindu_kinematics.bimanual.np.empty',side_effect=output), \
                self.assertRaisesRegex(ValueError,'BIMANUAL_QP_TIMEOUT:validation'):
            self.qp.solve(self.feedback,target,reference=[self.feedback[n] for n in self.qp.names],
                          dt=.01,target_stamp=1.)
        self.assertFalse(self.qp.captured)
        self.assertIsNone(self.qp.target_sample)

    def test_late_feedforward_copy_is_checked_before_committing_sample(self):
        solver=BimanualQP(dict(self.config,tool_velocity_feedforward=True),self.profile)
        self.addCleanup(solver.close)
        target=solver.capture(self.feedback)
        class SlowCopy(np.ndarray):
            def copy(self,*args,**kw):
                time.sleep(.04)
                return super().copy(*args,**kw)
        transform=solver._transform
        with patch.object(solver,'_transform',side_effect=lambda value:transform(value).view(SlowCopy)), \
                self.assertRaisesRegex(ValueError,'BIMANUAL_QP_TIMEOUT:validation'):
            solver.solve(self.feedback,target,reference=[self.feedback[n] for n in solver.names],
                         dt=.01,target_stamp=1.)
        self.assertFalse(solver.captured)
        self.assertIsNone(solver.target_sample)

    def test_feedforward_uses_source_time_and_resets_on_recapture(self):
        solver=BimanualQP({**self.config,'tool_velocity_feedforward':True},self.profile)
        self.addCleanup(solver.close)
        target=solver.capture(self.feedback)
        reference=[self.feedback[n] for n in solver.names]
        observed=[]
        native=solver.lib.bq_solve_tracking
        def observe(*args):
            observed.append(np.ctypeslib.as_array(args[6],shape=(6,)).copy())
            return native(*args)
        solver.lib.bq_solve_tracking=observe
        solver.solve(self.feedback,target,reference=reference,dt=.01,target_stamp=1.)
        moved=target.copy();moved[0,3]+=.001
        import pinocchio as pin
        moved[:3,:3]=pin.exp3(np.array([0.,0.,.002])) @ target[:3,:3]
        solver.solve(self.feedback,moved,reference=reference,dt=.03,target_stamp=1.02)
        np.testing.assert_allclose(observed[0],np.zeros(6))
        np.testing.assert_allclose(observed[1],[.05,0,0,0,0,.1],atol=1e-10)
        solver.capture(self.feedback)
        solver.solve(self.feedback,moved,reference=reference,dt=.01,target_stamp=2.)
        np.testing.assert_allclose(observed[-1],np.zeros(6))
        with self.assertRaisesRegex(ValueError,'TARGET_TIME'):
            solver.solve(self.feedback,moved,reference=reference,dt=.01,target_stamp=float('nan'))
        self.assertFalse(solver.captured)

    def test_tracking_abi_rejects_nonfinite_velocity_and_measured_pose(self):
        solver=BimanualQP({**self.config,'tool_velocity_feedforward':True},self.profile)
        self.addCleanup(solver.close)
        target=solver.capture(self.feedback)
        q=solver._q(self.feedback)
        reference=np.array([self.feedback[n] for n in solver.names])
        out=np.full(len(solver.names),123.)
        metrics=np.empty(3)
        for velocity,measured in ((np.full(6,np.nan),target),
                                  (np.zeros(6),np.full((4,4),np.nan))):
            self.assertEqual(solver.lib.bq_solve_tracking(solver.handle,solver._ptr(q),solver.nq,
                solver._ptr(reference),len(reference),solver._ptr(target),solver._ptr(velocity),
                solver._ptr(measured),.01,solver._ptr(out),solver._ptr(metrics)),0)
            np.testing.assert_array_equal(out,np.full(len(solver.names),123.))

    def test_causal_feedforward_removes_constant_velocity_servo_lag(self):
        errors=[]
        for enabled in (False,True):
            solver=BimanualQP({**self.config,'tool_velocity_feedforward':enabled},self.profile)
            self.addCleanup(solver.close)
            feedback=dict(self.feedback);home=solver.capture(feedback)
            for step in range(101):
                target=home.copy();target[0,3]+=.02*step*.01
                q,_=solver.solve(feedback,target,reference=[feedback[n] for n in solver.names],
                                 dt=.01,target_stamp=step*.01)
                feedback.update(zip(solver.names,q))
            actual=np.empty((4,4))
            self.assertTrue(solver.lib.bq_tool_pose(solver.handle,solver._ptr(solver._q(feedback)),
                            solver.nq,solver._ptr(actual)))
            errors.append(float(np.linalg.norm(actual[:3,3]-target[:3,3])))
        self.assertLess(errors[1],.003)
        self.assertLess(errors[1],errors[0]*.35)

    def test_reference_jacobian_still_tracks_measured_tool_error(self):
        solver=BimanualQP({**self.config,'reference_kinematics':True},self.profile)
        self.addCleanup(solver.close)
        home=solver.capture(self.feedback);target=home.copy();target[0,3]+=.01
        reference,_=propose(solver,self.feedback,target)
        q=solver._q(self.feedback)
        for name,value in zip(solver.names,reference):q[solver.model_names.index(name)]=value
        target=np.empty((4,4))
        self.assertTrue(solver.lib.bq_tool_pose(solver.handle,solver._ptr(q),solver.nq,solver._ptr(target)))
        # Target equals reference FK, but the measured tool has not arrived.
        advanced,_=propose(solver,self.feedback,target,reference=reference)
        for name,value in zip(solver.names,advanced):q[solver.model_names.index(name)]=value
        actual=np.empty((4,4))
        self.assertTrue(solver.lib.bq_tool_pose(solver.handle,solver._ptr(q),solver.nq,solver._ptr(actual)))
        self.assertGreater(float((actual[:3,3]-target[:3,3]) @ (target[:3,3]-home[:3,3])),1e-12)

    def test_position_reference_respects_profile_and_relative_constraint(self):
        tool=self.qp.capture(self.feedback)
        target=tool.copy();target[0,3]+=.01
        positions,metrics=propose(self.qp,self.feedback,target)
        self.assertEqual(len(positions),len(self.qp.names))
        self.assertLess(metrics[1],1e-6)
        for name,new in zip(self.qp.names,positions):
            self.assertLessEqual(abs(new-self.feedback[name]),self.profile.speed(name)*self.config['sample_period']+1e-6)
            lo,hi=self.profile.limits[name]
            self.assertGreaterEqual(new,lo)
            self.assertLessEqual(new,hi)

    def test_bounded_tool_observation_corrects_sag_and_keeps_joint_guards(self):
        solver=BimanualQP(dict(self.config,reference_kinematics=True,
            max_tool_feedback_translation_m=.05,max_tool_feedback_rotation_rad=.1),self.profile)
        self.addCleanup(solver.close)
        target=solver.capture(self.feedback);observed=target.copy();observed[2,3]-=.01
        reference=[self.feedback[n] for n in solver.names]
        out,_=solver.solve(self.feedback,target,reference=reference,dt=.01,tool_feedback=observed)
        q=solver._q(self.feedback)
        for name,value in zip(solver.names,out):q[solver.model_names.index(name)]=value
        next_tool=np.empty((4,4))
        self.assertTrue(solver.lib.bq_tool_pose(solver.handle,solver._ptr(q),solver.nq,solver._ptr(next_tool)))
        self.assertGreater(next_tool[2,3],target[2,3]+1e-5)
        damaged=dict(self.feedback);damaged[solver.names[-2]]+=.2
        with self.assertRaisesRegex(ValueError,'MEASURED_GRASP_ERROR'):
            solver.solve(damaged,target,reference=[damaged[n] for n in solver.names],dt=.01,tool_feedback=target)

    def test_tool_observation_requires_bounds_and_rejects_invalid_or_large_correction(self):
        for correction in ('unconfigured','too_far','invalid'):
            with self.subTest(correction=correction):
                cfg=dict(self.config,reference_kinematics=True)
                if correction!='unconfigured':cfg.update(max_tool_feedback_translation_m=.05,max_tool_feedback_rotation_rad=.1)
                solver=BimanualQP(cfg,self.profile);self.addCleanup(solver.close)
                target=solver.capture(self.feedback);observed=target.copy()
                if correction=='too_far':observed[0,3]+=.051
                if correction=='invalid':observed[0,0]=float('nan')
                with self.assertRaisesRegex(ValueError,'BIMANUAL_TOOL_(FEEDBACK_DISAGREEMENT|TRANSFORM)'):
                    solver.solve(self.feedback,target,reference=[self.feedback[n] for n in solver.names],dt=.01,tool_feedback=observed)
                self.assertFalse(solver.captured)

    def test_actual_posture_guard_is_independent_of_reference_jacobians(self):
        name=self.qp.names[0];value=self.feedback[name]
        solver=BimanualQP(dict(self.config,reference_kinematics=True,posture_policy={
            'reference':'initial_capture','position_bounds':{name:[value-.05,value+.05]}}),self.profile)
        self.addCleanup(solver.close)
        target=solver.capture(self.feedback);damaged=dict(self.feedback);damaged[name]+=.051
        with self.assertRaisesRegex(ValueError,'BIMANUAL_MEASURED_POSTURE_ERROR'):
            solver.solve(damaged,target,reference=[self.feedback[n] for n in solver.names],dt=.01)
        self.assertFalse(solver.captured)

    def test_integration_uses_accepted_reference_and_variable_interval(self):
        tool = self.qp.capture(self.feedback)
        target = tool.copy(); target[0,3] += .015
        initial = np.array([self.feedback[n] for n in self.qp.names])
        first, _ = propose(self.qp, self.feedback, target, dt=.01, reference=initial)
        second, _ = propose(self.qp, self.feedback, target, dt=.02, reference=first)
        self.assertGreater(np.linalg.norm(np.array(first)-initial), 1e-5)
        # Feedback deliberately lags: the previous increment must not vanish.
        np.testing.assert_allclose(np.array(second)-first, 2*(np.array(first)-initial), atol=1e-6)
        results = []
        for hz in (50, 60, 100):
            self.qp.capture(self.feedback)
            accepted = initial.copy()
            for _ in range(hz//5):
                accepted, _ = propose(self.qp, self.feedback, target, dt=1/hz, reference=accepted)
            results.append(accepted)
        np.testing.assert_allclose(results[0], results[1], atol=1e-6)
        np.testing.assert_allclose(results[0], results[2], atol=1e-6)

    def test_reference_kinematics_keeps_measured_grasp_guard(self):
        solver=BimanualQP(dict(self.config,reference_kinematics=True),self.profile)
        self.addCleanup(solver.close)
        tool=solver.capture(self.feedback)
        target=tool.copy();target[0,3]+=.01
        out,_=propose(solver,self.feedback,target)
        self.assertGreater(np.linalg.norm(np.array(out)-[self.feedback[n] for n in solver.names]),1e-6)
        damaged=dict(self.feedback);damaged[solver.names[-2]]+=.2
        with self.assertRaisesRegex(ValueError,'MEASURED_GRASP_ERROR'):
            propose(solver,damaged,target,reference=[damaged[n] for n in solver.names])
        self.assertFalse(solver.captured)

    def test_reference_governor_bounds_lead_without_relaxing_hard_guard(self):
        solver=BimanualQP(dict(self.config,reference_kinematics=True),self.profile)
        self.addCleanup(solver.close)
        target=solver.capture(self.feedback);target[0,3]+=.2
        initial=np.array([self.feedback[n] for n in solver.names]);ref=initial.copy()
        scales=[]
        for _ in range(200):
            out,metrics=propose(solver,self.feedback,target,reference=ref,dt=.02)
            ref=np.asarray(out);scales.append(metrics[2])
            self.assertLessEqual(float(np.max(np.abs(ref-initial))),.8*solver.max_reference_error+1e-10)
        self.assertLess(min(scales),.99)
        self.assertGreater(float(np.max(np.abs(ref-initial))),.01)
        wrong=ref.copy();wrong[0]=initial[0]+solver.max_reference_error+.01
        with self.assertRaisesRegex(ValueError,'REFERENCE_TRACKING_ERROR'):
            propose(solver,self.feedback,target,reference=wrong)
        self.assertFalse(solver.captured)

    def test_large_tracking_error_and_gap_require_recapture(self):
        initial = np.array([self.feedback[n] for n in self.qp.names])
        tool = self.qp.capture(self.feedback)
        wrong = initial.copy(); wrong[0] += .2
        with self.assertRaisesRegex(ValueError, 'REFERENCE_TRACKING_ERROR'):
            propose(self.qp, self.feedback, tool, reference=wrong)
        with self.assertRaisesRegex(ValueError, 'NOT_CAPTURED'):
            propose(self.qp, self.feedback, tool)
        self.qp.capture(self.feedback)
        with self.assertRaisesRegex(ValueError, 'INTEGRATION_INTERVAL'):
            propose(self.qp, self.feedback, tool, dt=.101)
        self.qp.capture(self.feedback)
        propose(self.qp, self.feedback, tool, dt=.02)

    def test_all_thirteen_config_fields_reach_native_validation(self):
        self.assertEqual(len(NativeConfig._fields_[2:]), 13)
        for name, _ in NativeConfig._fields_[2:]:
            with self.subTest(name=name):
                with self.assertRaises(ValueError):
                    solver = BimanualQP(dict(self.config, **{name:float('nan')}), self.profile)
                    self.addCleanup(solver.close)
                    solver.capture(self.feedback)
                if name not in ('max_linear_speed', 'max_angular_speed', 'max_relative_error'):
                    self.assertFalse(solver.captured)
        self.qp.capture(self.feedback)

    def test_shared_targets_are_reachable_from_json_and_can_be_cleared(self):
        name = self.qp.shared_names[0]
        cfg = dict(self.config, shared_target_weight=.3,
                   shared_targets={n:self.feedback[n]+(.03 if n==name else 0.)
                                   for n in self.qp.shared_names})
        solver = BimanualQP(cfg, self.profile)
        self.addCleanup(solver.close)
        tool = solver.capture(self.feedback)
        moved, _ = propose(solver, self.feedback, tool)
        self.assertGreater(moved[0]-self.feedback[name], 1e-5)
        solver.set_shared_targets(None)
        held, _ = propose(solver, self.feedback, tool)
        self.assertLess(abs(held[0]-self.feedback[name]), 1e-6)
        with self.assertRaisesRegex(ValueError, 'SHARED_TARGET_LAYOUT'):
            solver.set_shared_targets({'unknown': 0.})

    def test_posture_policy_survives_recapture_and_native_failure_invalidates(self):
        cfg=dict(self.config,posture_policy={'reference':'initial_capture','position_bounds':{}})
        solver=BimanualQP(cfg,self.profile)
        self.addCleanup(solver.close)
        feedback=dict(self.feedback)
        tool=solver.capture(feedback)
        preferred=solver.posture_reference.copy()
        target=tool.copy();target[0,3]+=.02
        for _ in range(20):
            positions,_=propose(solver,feedback,target)
            feedback.update(zip(solver.names,positions))
        self.assertGreater(np.linalg.norm(np.array([feedback[n] for n in solver.names])-preferred),1e-4)
        solver.capture(feedback)
        np.testing.assert_array_equal(solver.posture_reference,preferred)
        p=solver._ptr
        invalid=solver.posture_upper.copy();invalid[0]=float('nan')
        self.assertEqual(solver.lib.bq_posture_policy(solver.handle,p(preferred),p(solver.posture_lower),
                                                     p(invalid),len(preferred)),0)
        with self.assertRaisesRegex(ValueError,'not captured'):
            propose(solver,feedback,target)
        self.assertFalse(solver.captured)

    def test_posture_policy_rejects_unknown_nonfinite_or_widened_bounds(self):
        name=self.qp.names[0]
        for bounds in ({'unknown':[-1.,1.]},{name:[float('nan'),1.]},
                       {name:[self.profile.limits[name][0]-1.,self.profile.limits[name][1]]}):
            cfg=dict(self.config,posture_policy={'reference':'initial_capture','position_bounds':bounds})
            with self.assertRaisesRegex(ValueError,'POSTURE'):
                BimanualQP(cfg,self.profile)

    def test_profile_position_limits_are_used_inside_qp(self):
        tool = self.qp.capture(self.feedback)
        target = tool.copy(); target[0,3] += .015
        trial, _ = propose(self.qp, self.feedback, target)
        delta = np.array(trial)-[self.feedback[n] for n in self.qp.names]
        index = int(np.argmax(np.abs(delta)))
        name = self.qp.names[index]
        limits = dict(self.profile.limits)
        lo, hi = limits[name]
        q = self.feedback[name]
        limits[name] = (lo, q+1e-4) if delta[index] > 0 else (q-1e-4, hi)
        solver = BimanualQP(dict(self.config, limit_margin=1e-5), replace(self.profile, limits=limits))
        self.addCleanup(solver.close)
        solver.capture(self.feedback)
        out, _ = propose(solver, self.feedback, target, dt=.1)
        self.assertGreaterEqual(out[index], limits[name][0])
        self.assertLessEqual(out[index], limits[name][1])

    def test_native_rejects_null_buffers_wrong_lengths_and_nonfinite_pose(self):
        tool = self.qp.capture(self.feedback)
        q = self.qp._q(self.feedback)
        p = self.qp._ptr
        ref = np.array([self.feedback[n] for n in self.qp.names])
        out = np.full(len(ref), 123.)
        metrics = np.empty(3)
        self.assertEqual(self.qp.lib.bq_limits(self.qp.handle, None, p(out), p(out), len(ref)), 0)
        self.assertEqual(self.qp.lib.bq_solve(self.qp.handle, p(q), len(q), p(ref), len(ref)-1,
                                            p(tool), .01, p(out), p(metrics)), 0)
        bad = tool.copy(); bad[3,0] = float('nan')
        self.assertEqual(self.qp.lib.bq_solve(self.qp.handle, p(q), len(q), p(ref), len(ref),
                                            p(bad), .01, p(out), p(metrics)), 0)
        np.testing.assert_array_equal(out, np.full(len(ref), 123.))
        self.qp.close()
        with self.assertRaisesRegex(ValueError, 'CLOSED'):
            self.qp.capture(self.feedback)

    def test_invalid_grasp_geometry_and_target_fail_closed(self):
        with self.assertRaisesRegex(ValueError,'BIMANUAL_GRASP_GEOMETRY'):
            self.qp.capture(dict.fromkeys(self.qp.model_names,0.))
        tool=self.qp.capture(self.feedback)
        bad=tool.copy();bad[0,0]=-1.
        with self.assertRaisesRegex(ValueError,'invalid SE3'):
            propose(self.qp,self.feedback,bad)

    def test_other_robot_without_shared_waist_group(self):
        left=['left_joint_'+str(k) for k in range(1,7)]
        right=['right_joint_'+str(k) for k in range(1,7)]
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'profile.json'
            path.write_text(json.dumps({'name':'two_arm_fixture',
                'groups':{'left':left,'right':right},
                'limits':{n:[-2.5,2.5] for n in left+right},
                'max_speed':.5,'capabilities':['joint_position']}))
            profile=Profile.load(path)
            cfg=dict(self.config,urdf=str(ROOT/'src/capabilities/bindu_bimanual_qp/test/two_arms.urdf'),
                resource_groups=['left','right'],left_frame='left_ee',right_frame='right_ee',
                left_to_tool=[1,0,0,.12,0,1,0,-.35,0,0,1,0,0,0,0,1],
                flange_distance_range=[.5,1.5])
            solver=BimanualQP(cfg,profile)
            try:
                self.assertEqual(solver.names,tuple(left+right))
                tool=solver.capture(dict.fromkeys(solver.model_names,0.))
                self.assertEqual(tool.shape,(4,4))
            finally:
                solver.close()

    def test_snowbot_cad_waist_and_seven_axis_arm_layout(self):
        config=json.loads((ROOT/'src/integration/bindu_runtime/config/bimanual_snowbot_sim.json').read_text())
        config['urdf']=str(ROOT/'src/hardware/bindu_description/urdf/snowbot_control.urdf')
        profile=Profile.load(ROOT/'src/integration/bindu_runtime/config/snowbot_sim.json')
        solver=BimanualQP(config,profile)
        try:
            self.assertEqual(solver.names[:2],('body_joint_1','body_joint_2'))
            self.assertEqual(len(solver.names),16)
            self.assertEqual(len(solver.model_names),20)
            self.assertNotIn('arm_l_joint_8',solver.names)
            feedback=dict.fromkeys(solver.model_names,0.)
            tool=solver.capture(feedback)
            self.assertAlmostEqual(tool[1,3],0.,places=6)
            waist_changes=[]
            for axis in (0,1):
                target=tool.copy();target[axis,3]+=.01
                positions,metrics=propose(solver,feedback,target)
                waist_changes.append(abs(positions[1-axis]))
                self.assertLess(metrics[1],1e-6)
                for name,value in zip(solver.names,positions):
                    lo,hi=profile.limits[name]
                    self.assertLessEqual(lo,value)
                    self.assertLessEqual(value,hi)
                    self.assertLessEqual(abs(value),profile.speed(name)*config['sample_period']+1e-6)
            self.assertTrue(all(change>1e-5 for change in waist_changes))
        finally:
            solver.close()

    def test_shovel_grasp_checks_orientation_and_invalidates_failed_capture(self):
        config=json.loads((ROOT/'src/integration/bindu_runtime/config/bimanual_snowbot_shovel.json').read_text())
        config['urdf']=str(ROOT/'src/hardware/bindu_description/urdf/snowbot_control.urdf')
        profile=Profile.load(ROOT/'src/integration/bindu_runtime/config/snowbot_sim.json')
        solver=BimanualQP(config,profile)
        self.addCleanup(solver.close)
        ready=dict(config['demo']['initial_positions_rad'])
        tool=solver.capture(ready)
        bad=dict(ready)
        bad['arm_r_joint_7']+=.1
        # The hands are still within the distance gate, but cannot hold this
        # rigid shaft in its configured orientation.
        with self.assertRaisesRegex(ValueError,'BIMANUAL_GRASP_POSE'):
            solver.capture(bad)
        with self.assertRaisesRegex(ValueError,'BIMANUAL_NOT_CAPTURED'):
            propose(solver,ready,tool)
        solver.capture(ready)
        _,metrics=propose(solver,ready,tool)
        self.assertLess(metrics[1],1e-6)

    def test_shovel_grasp_rejects_inconsistent_second_attachment(self):
        config=json.loads((ROOT/'src/integration/bindu_runtime/config/bimanual_snowbot_shovel.json').read_text())
        config['urdf']=str(ROOT/'src/hardware/bindu_description/urdf/snowbot_control.urdf')
        profile=Profile.load(ROOT/'src/integration/bindu_runtime/config/snowbot_sim.json')
        attachment=np.array(config['right_to_tool']).reshape(4,4)
        attachment[0,3]+=.02
        config['right_to_tool']=attachment.ravel().tolist()
        solver=BimanualQP(config,profile)
        self.addCleanup(solver.close)
        with self.assertRaisesRegex(ValueError,'BIMANUAL_GRASP_POSE'):
            solver.capture(config['demo']['initial_positions_rad'])


class AxialShovelTests(unittest.TestCase):
    def setUp(self):
        self.cfg=json.loads((ROOT/'src/integration/bindu_runtime/config/bimanual_snowbot_shovel.json').read_text())
        self.cfg['urdf']=str(ROOT/'src/hardware/bindu_description/urdf/snowbot_control.urdf')
        self.profile=Profile.load(ROOT/'src/integration/bindu_runtime/config/snowbot_sim.json')
        self.qp=BimanualQP(self.cfg,self.profile)
        self.addCleanup(self.qp.close)
        self.ready=self.cfg['demo']['initial_positions_rad']

    def test_axial_capture_and_relock_preserve_tool_with_grip_checks(self):
        before=self.qp.capture(self.ready)
        axial=self.qp.capture(self.ready,'right_axial')
        np.testing.assert_allclose(axial,before,atol=1e-12)
        q,_=propose(self.qp,self.ready,axial)
        np.testing.assert_allclose(q,[self.ready[n] for n in self.qp.names],atol=1e-7)
        rigid=self.qp.capture(self.ready,'rigid')
        np.testing.assert_allclose(rigid,before,atol=1e-12)
        with self.assertRaisesRegex(ValueError,'GRASP_MODE'):
            self.qp.capture(self.ready,'unknown')
        with self.assertRaisesRegex(ValueError,'NOT_CAPTURED'):
            propose(self.qp,self.ready,rigid)

    def test_guard_rejects_tool_into_body_and_failed_capture_invalidates(self):
        # Isolate the geometry guard: the configured posture envelope now
        # rejects this large bend before it can reach collision checking.
        cfg=dict(self.cfg);cfg.pop('posture_policy',None);cfg.pop('posture_reference_margin_rad',None)
        solver=BimanualQP(cfg,self.profile)
        self.addCleanup(solver.close)
        solver.capture(self.ready)
        bad=dict(self.ready)
        bad['body_joint_2']+=.7
        with self.assertRaisesRegex(ValueError,'CLEARANCE|COLLISION|GROUND'):
            solver.capture(bad,'right_axial')
        self.assertFalse(solver.captured)

    def test_posture_envelope_rejects_capture_and_invalidates_reference(self):
        tool=self.qp.capture(self.ready)
        bad=dict(self.ready);bad['body_joint_2']+=.7
        with self.assertRaisesRegex(ValueError,'POSTURE_POLICY'):
            self.qp.capture(bad,'right_axial')
        self.assertFalse(self.qp.captured)
        with self.assertRaisesRegex(ValueError,'NOT_CAPTURED'):
            propose(self.qp,self.ready,tool)

    def test_axial_attachment_matches_shovel_geometry(self):
        np.testing.assert_allclose(self.cfg['right_axial_grasp']['axis_in_tool'],
                                   self.cfg['demo']['shaft_axis_in_tool'])
        np.testing.assert_allclose(self.cfg['right_axial_grasp']['grip_in_right_flange_m'],
                                   self.cfg['demo'].get('right_grip_offset_m', self.cfg['demo']['grip_offset_m']))

    def test_native_failed_axial_capture_cannot_reuse_rigid_reference(self):
        import ctypes as c
        tool=self.qp.capture(self.ready)
        q=self.qp._q(self.ready)
        invalid=self.qp.tool_grip.copy();invalid[0,0]=float('nan')
        ok=self.qp.lib.bq_capture_axial(self.qp.handle,self.qp._ptr(q),self.qp.nq,
            self.qp._ptr(self.qp.tool),self.qp._ptr(self.qp.right_grip),self.qp._ptr(invalid),
            c.byref(self.qp.native_config))
        self.assertEqual(ok,0)
        with self.assertRaisesRegex(ValueError,'not captured'):
            propose(self.qp,self.ready,tool)
        self.assertFalse(self.qp.captured)
