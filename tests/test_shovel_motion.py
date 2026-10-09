"""Ground and tool/body clearance regressions against the Snowbot CAD model."""
import copy
import json
from pathlib import Path
import sys
import unittest

import numpy as np
import pinocchio as pin

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools'))
from plan_shovel_motion import collision_check, validate_recipe, target_at, grasp_mode_at
from shovel_geometry import bin_boxes, check_dump
from scipy.spatial.transform import Rotation


class ShovelGeometryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = json.loads((ROOT/'src/integration/bindu_runtime/config/bimanual_snowbot_shovel.json').read_text())
        cls.urdf = ROOT/'src/hardware/bindu_description/urdf/snowbot_display.urdf'
        cls.recipe = copy.deepcopy(cls.config['demo'])
        positions = cls.recipe['initial_positions_rad']
        model = pin.buildModelFromUrdf(str(cls.urdf))
        data = model.createData()
        q = pin.neutral(model)
        for name,value in positions.items():
            q[model.joints[model.getJointId(name)].idx_q] = value
        pin.framesForwardKinematics(model, data, q)
        flange = data.oMf[model.getFrameId(cls.config['left_frame'])].homogeneous.copy()
        tool = flange @ np.array(cls.config['left_to_tool']).reshape(4,4)
        cls.recipe['left_grip_in_tool_m'] = (np.linalg.inv(tool) @ flange @ np.r_[cls.recipe['grip_offset_m'],1.])[:3].tolist()
        cls.sample = {'t':0., 'q':positions, 'tool':tool.tolist()}
        cls.body_point = data.oMf[model.getFrameId('body_link_2')].translation.copy()

    def test_selected_ready_pose_has_real_ground_and_tool_clearance(self):
        validate_recipe(self.config)
        result = collision_check(self.urdf, [self.sample], self.recipe)
        self.assertEqual(result['violations'],0)
        self.assertGreaterEqual(result['min_tool_robot_clearance_m'],.02)
        self.assertGreater(result['aabb_bounded_queries'],0)
        self.assertEqual(len(result['support_heights_m']),4)

    def test_wrong_ground_empty_mesh_and_empty_motion_fail(self):
        with self.assertRaisesRegex(ValueError,'EMPTY_COLLISION'):
            collision_check(self.urdf, [], self.recipe)
        for field,value,reason in [('ground',dict(self.recipe['ground'],z_in_base_m=0.),'GROUND_SUPPORT_MISMATCH'),
                                    ('empty_mesh_proxies',{},'MISSING_COLLISION_GEOMETRY')]:
            recipe = dict(self.recipe,**{field:value})
            with self.subTest(field=field), self.assertRaisesRegex(ValueError,reason):
                collision_check(self.urdf,[self.sample],recipe)

    def test_tool_through_body_and_contact_outside_grip_band_fail(self):
        sample = copy.deepcopy(self.sample)
        tool = np.array(sample['tool'])
        tool[:3,3] = self.body_point-tool[:3,:3] @ np.array(self.recipe['pan_center_m'])
        sample['tool'] = tool.tolist()
        with self.assertRaisesRegex(ValueError,'CLEARANCE'):
            collision_check(self.urdf,[sample],self.recipe)
        recipe = copy.deepcopy(self.recipe)
        for contact in recipe['shaft_contacts'].values():
            contact['half_length_m'] = .01
        with self.assertRaisesRegex(ValueError,'SHAFT_CLEARANCE'):
            collision_check(self.urdf,[self.sample],recipe)

    def test_angle_drift_and_nonfinite_inputs_fail(self):
        config = copy.deepcopy(self.config)
        config['demo']['selected_design']['outward_swing_deg'] = 10.
        with self.assertRaisesRegex(ValueError,'KEYFRAME_DESIGN_MISMATCH'):
            validate_recipe(config)
        config = copy.deepcopy(self.config)
        config['demo']['pan_size_m'][0] = float('nan')
        with self.assertRaisesRegex(ValueError,'NONFINITE_SHOVEL_CONFIG'):
            validate_recipe(config)
        sample = copy.deepcopy(self.sample)
        sample['q']['body_joint_1'] = float('nan')
        with self.assertRaisesRegex(ValueError,'NONFINITE_COLLISION_SAMPLE'):
            collision_check(self.urdf,[sample],self.recipe)


    def test_receiving_bin_is_open_but_walls_are_checked(self):
        recipe = copy.deepcopy(self.recipe)
        recipe['receiving_bin'] = {'center_xy_m':[10.,10.], 'inner_size_xy_m':[.7,.6],
            'height_m':.45, 'wall_thickness_m':.02, 'bottom_z_m':recipe['ground']['z_in_base_m'],
            'min_clearance_m':.02}
        self.assertEqual(len(bin_boxes(recipe)),5)
        result = collision_check(self.urdf,[self.sample],recipe)
        self.assertGreater(result['min_bin_clearance_m'],1.)
        tool = np.array(self.sample['tool'])
        center = (tool @ np.r_[recipe['pan_center_m'],1.])[:3]
        recipe['receiving_bin']['center_xy_m'] = [float(center[0]-.36),float(center[1])]
        with self.assertRaisesRegex(ValueError,'BIN_CLEARANCE'):
            collision_check(self.urdf,[self.sample],recipe)

    def test_dump_requires_inversion_containment_and_complete_hold(self):
        recipe = copy.deepcopy(self.recipe)
        recipe['receiving_bin'] = {'center_xy_m':[0.,0.], 'inner_size_xy_m':[.8,.8],
            'height_m':.4,'wall_thickness_m':.02,'bottom_z_m':0.,'min_clearance_m':.02}
        recipe['dump'] = {'hold_seconds':[.5,1.5],'transfer_seconds':[0.,2.],
            'mouth_margin_m':.04,'min_rim_clearance_m':.05,'min_pan_tilt_deg':100.}
        tool = np.eye(4)
        tool[:3,:3] = Rotation.from_euler('x',120,degrees=True).as_matrix()
        tool[:3,3] = np.array([0.,0.,.9])-tool[:3,:3] @ np.array(recipe['pan_center_m'])
        samples = [{'t':t,'tool':tool.tolist()} for t in (0.,.5,1.,1.5,2.)]
        result = check_dump(samples,recipe)
        self.assertAlmostEqual(result['min_pan_tilt_deg'],120.)
        with self.assertRaisesRegex(ValueError,'MISSING_DUMP_SAMPLES'):
            check_dump(samples[1:],recipe)
        for field,reason in [('upright','DUMP_PAN_NOT_INVERTED'),('outside','DUMP_MISSES_BIN'),
                             ('low','DUMP_RIM_CLEARANCE'),('nan','NONFINITE_DUMP_POSE')]:
            bad = copy.deepcopy(samples)
            for sample in bad:
                tf = np.array(sample['tool'])
                if field == 'upright': tf[:3,:3]=np.eye(3)
                if field == 'outside': tf[0,3]+=1.
                if field == 'low': tf[2,3]-=.6
                if field == 'nan': tf[0,3]=float('nan')
                sample['tool']=tf.tolist()
            with self.subTest(field=field), self.assertRaisesRegex(ValueError,reason):
                check_dump(bad,recipe)

    def test_shaft_roll_preserves_both_grip_points(self):
        axis=np.array(self.recipe['shaft_axis_in_tool'])
        pivot=np.array(self.recipe['pivot_in_tool_m'])
        keys=[{'t':0.,'translation_m':[0.,0.,0.],'yaw_pitch_rad':[0.,0.],'shaft_roll_rad':0.},
              {'t':1.,'translation_m':[0.,0.,0.],'yaw_pitch_rad':[0.,0.],'shaft_roll_rad':2.}]
        for t in np.linspace(0,1,11):
            tf=target_at(keys,np.eye(4),pivot,t,axis)
            for point in (pivot,pivot-.34*axis):
                np.testing.assert_allclose((tf @ np.r_[point,1.])[:3],point,atol=1e-10)

    def test_target_final_grid_point_tolerates_only_roundoff(self):
        keys = self.recipe['keyframes']
        t = np.nextafter(keys[-1]['t'], float('inf'))
        target_at(keys,np.eye(4),np.array(self.recipe['pivot_in_tool_m']),t,np.array(self.recipe['shaft_axis_in_tool']))
        for invalid in (t+.01, float('nan')):
            with self.assertRaisesRegex(ValueError,'TIME_OUTSIDE_RECIPE'):
                target_at(keys,np.eye(4),np.zeros(3),invalid)

    def test_grasp_changes_require_configured_axial_support_and_stationary_hold(self):
        recipe = self.config['demo']
        schedule = recipe['grasp_schedule']
        validate_recipe(self.config)
        for change in schedule[1:]:
            self.assertEqual(grasp_mode_at(recipe, change['t']), change['mode'])
            self.assertNotEqual(grasp_mode_at(recipe, change['t']-.01), change['mode'])
        for field,value,reason in [('t',5.,'REQUIRES_HOLD'),('t',17.551,'REQUIRES_HOLD'),
                                   ('mode','open','INVALID_SHOVEL_GRASP_SCHEDULE'),
                                   ('mode','rigid','INVALID_SHOVEL_GRASP_SCHEDULE')]:
            bad = copy.deepcopy(self.config)
            bad['demo']['grasp_schedule'][1][field] = value
            with self.subTest(field=field,value=value), self.assertRaisesRegex(ValueError,reason):
                validate_recipe(bad)
        bad = copy.deepcopy(self.config)
        del bad['right_axial_grasp']
        with self.assertRaisesRegex(ValueError,'INVALID_SHOVEL_GRASP_SCHEDULE'):
            validate_recipe(bad)
        self.assertEqual(grasp_mode_at({}, 10.), 'rigid')

    def test_motion_cannot_supply_staged_waist_joint_targets(self):
        validate_recipe(self.config)
        for late_key in (False, True):
            config=copy.deepcopy(self.config)
            if late_key:
                config['demo']['keyframes'][-1]['shared_targets_rad']={'body_joint_2':.2}
            else:
                config['shared_targets']={'body_joint_1':0.,'body_joint_2':.2}
            with self.subTest(late_key=late_key), self.assertRaisesRegex(ValueError,'REQUIRES_TOOL_POSE_INPUT'):
                validate_recipe(config)

    def test_planning_limit_override_is_simulation_only(self):
        config=copy.deepcopy(self.config)
        config['simulation_only']=False
        with self.assertRaisesRegex(ValueError,'INVALID_SHOVEL_PLANNING_LIMITS'):
            validate_recipe(config)
        config=copy.deepcopy(self.config)
        config['demo']['planning_limits']['max_speed']=-1
        with self.assertRaisesRegex(ValueError,'INVALID_SHOVEL_PLANNING_LIMITS'):
            validate_recipe(config)


if __name__ == '__main__':
    unittest.main()


class OnlineGeometryTests(unittest.TestCase):
    setUpClass = classmethod(ShovelGeometryTests.setUpClass.__func__)
    # Check the conservative online representation against a real CAD fixture.
    def test_online_outer_guard_preserves_ready_and_rejects_body_overlap(self):
        from bindu_kinematics.tool_geometry import make_collision_checker
        check=make_collision_checker(self.urdf,self.recipe,conservative_convex=True)
        result=check([self.sample])
        self.assertEqual(result['tool_body_representation'],'convex_outer')
        self.assertGreaterEqual(result['min_tool_robot_clearance_m'],.02)
        bad=copy.deepcopy(self.sample)
        bad['tool']=np.eye(4);bad['tool'][:3,3]=self.body_point
        with self.assertRaisesRegex(ValueError,'CLEARANCE|COLLISION'):
            check([bad])
        with self.assertRaisesRegex(ValueError,'EMPTY'):
            check([])
