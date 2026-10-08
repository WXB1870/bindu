"""Native integration gate; skips when the optional QP package is not installed."""
import json
import tempfile
import unittest
from pathlib import Path
import numpy as np
from ament_index_python.packages import PackageNotFoundError, get_package_prefix
from bindu_contracts.profile import Profile
from bindu_kinematics.bimanual import BimanualQP

ROOT=Path(__file__).resolve().parents[1]


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
        positions,metrics=self.qp.solve(self.feedback,target)
        self.assertEqual(len(positions),len(self.qp.names))
        self.assertLess(metrics[1],1e-6)
        for name,new in zip(self.qp.names,positions):
            self.assertLessEqual(abs(new-self.feedback[name]),self.profile.speed(name)*self.config['sample_period']+1e-6)
            lo,hi=self.profile.limits[name]
            self.assertGreaterEqual(new,lo)
            self.assertLessEqual(new,hi)

    def test_invalid_grasp_geometry_and_target_fail_closed(self):
        with self.assertRaisesRegex(ValueError,'BIMANUAL_GRASP_GEOMETRY'):
            self.qp.capture(dict.fromkeys(self.qp.model_names,0.))
        tool=self.qp.capture(self.feedback)
        bad=tool.copy();bad[0,0]=-1.
        with self.assertRaisesRegex(ValueError,'invalid SE3'):
            self.qp.solve(self.feedback,bad)

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
                positions,metrics=solver.solve(feedback,target)
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
            solver.solve(ready,tool)
        solver.capture(ready)
        _,metrics=solver.solve(ready,tool)
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
