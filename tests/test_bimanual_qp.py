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
from bindu_kinematics.bimanual_worker import BimanualWorker

ROOT=Path(__file__).resolve().parents[1]


def propose(solver, feedback, target, dt=.01, reference=None):
    if reference is None:
        reference = [feedback[n] for n in solver.names]
    return solver.solve(feedback, target, reference=reference, dt=dt)


class AbiLoadingTests(unittest.TestCase):
    def test_missing_and_incompatible_abi_rejected_before_binding(self):
        with self.assertRaisesRegex(ValueError, 'ABI_MISSING'):
            check_abi(object())
        lib = Mock()
        lib.bq_abi_version.return_value = 999
        with self.assertRaisesRegex(ValueError, 'ABI_MISMATCH'):
            check_abi(lib)
        lib.bq_create.assert_not_called()

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
        self.qp.capture(self.ready)
        bad=dict(self.ready)
        bad['body_joint_2']+=.7
        with self.assertRaisesRegex(ValueError,'CLEARANCE|COLLISION|GROUND'):
            self.qp.capture(bad,'right_axial')
        self.assertFalse(self.qp.captured)

    def test_axial_attachment_matches_shovel_geometry(self):
        np.testing.assert_allclose(self.cfg['right_axial_grasp']['axis_in_tool'],
                                   self.cfg['demo']['shaft_axis_in_tool'])
        np.testing.assert_allclose(self.cfg['right_axial_grasp']['grip_in_right_flange_m'],
                                   self.cfg['demo']['grip_offset_m'])

    def test_native_failed_axial_capture_cannot_reuse_rigid_reference(self):
        import ctypes as c
        tool=self.qp.capture(self.ready)
        q=self.qp._q(self.ready)
        invalid=self.qp.tool_grip.copy();invalid[0,0]=float('nan')
        ok=self.qp.lib.bq_capture_axial(self.qp.handle,self.qp._ptr(q),self.qp.nq,
            self.qp._ptr(self.qp.tool),self.qp._ptr(self.qp.right_grip),self.qp._ptr(invalid),
            c.byref(self.qp.native_config))
        self.assertEqual(ok,0)
        with self.assertRaisesRegex(ValueError,'solver not captured'):
            propose(self.qp,self.ready,tool)
        self.assertFalse(self.qp.captured)
