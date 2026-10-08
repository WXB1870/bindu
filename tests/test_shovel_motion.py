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
from plan_shovel_motion import collision_check, validate_recipe


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


if __name__ == '__main__':
    unittest.main()
