"""Position tracking selection, without ROS or devices."""
from bindu_contracts.resources import package_share
import json
import unittest

from bindu_runtime.teleop_deployment import apply_execution_tracking, load_deployment
from tests.support.paths import ROOT

SHARE = ROOT/'src/integration/bindu_runtime'


class PositionTrackingTests(unittest.TestCase):
    def setUp(self):
        self.config = json.loads((package_share("snowbot_bringup")/'config/robots/snowbot/bimanual_snowbot_real.json').read_text())
        self.config.pop('tracking_feedback',None)  # Legacy callers without an explicit selection.

    def test_legacy_position_tracking_is_unchanged_without_explicit_selection(self):
        _, args = load_deployment(package_share("snowbot_bringup")/'config/deployments/snowbot_vr_real.json', SHARE)
        self.assertEqual(args['execution_mode'], 'position')
        self.assertEqual(args['command_enabled'], 'false')
        self.assertIs(apply_execution_tracking(self.config, real=True, position_contract=True), self.config)
        self.assertEqual(self.config.get('tracking_feedback', 'measured'), 'measured')

    def test_position_target_stream_pairs_accumulation_with_nominal_error(self):
        selected=apply_execution_tracking(self.config,real=True,position_contract=True,
                                          trajectory_reference=True)
        self.assertEqual(selected['tracking_feedback'],'reference')
        self.assertEqual({k:v for k,v in selected.items() if k!='tracking_feedback'},self.config)
        self.assertNotIn('tracking_feedback',self.config)
        with self.assertRaisesRegex(ValueError,'TRAJECTORY_REQUIRES_REAL_NOMINAL'):
            apply_execution_tracking(dict(self.config,reference_kinematics=False),real=True,
                                     position_contract=True,trajectory_reference=True)

    def test_position_preserves_explicit_measured_tracking(self):
        for source in ('measured_task','measured_shared'):
            with self.subTest(source=source):
                config=dict(self.config,tracking_feedback=source)
                selected=apply_execution_tracking(config,real=True,position_contract=True,
                                                   trajectory_reference=True)
                self.assertEqual(selected['tracking_feedback'],source)
                self.assertEqual(config['tracking_feedback'],source)

    def test_real_recipe_selects_measured_waist_with_continuous_arm_reference(self):
        config=json.loads((package_share("snowbot_bringup")/'config/robots/snowbot/bimanual_snowbot_real.json').read_text())
        self.assertEqual(apply_execution_tracking(config,real=True,position_contract=True,
            trajectory_reference=True)['tracking_feedback'],'measured_shared')

    def test_real_sixteen_axis_nominal_config_runs_native_qp(self):
        from bindu_contracts.profile import Profile
        from bindu_kinematics.bimanual import BimanualQP
        from snowbot_hardware.protocol import GROUPS
        cfg = apply_execution_tracking(self.config, real=True, position_contract=True, trajectory_reference=True)
        cfg['urdf'] = str(package_share("snowbot_description")/'urdf/snowbot_control.urdf')
        profile = Profile.load(package_share("snowbot_bringup")/'config/examples/snowbot/snowbot_vendor_position.json')
        solver = BimanualQP(cfg, profile);self.addCleanup(solver.close)
        q = json.loads((package_share("snowbot_bringup")/'config/examples/snowbot/bimanual_snowbot_shovel.json').read_text())['demo']['initial_positions_rad']
        target = solver.capture(q).copy();target[0,3] += .001
        output, _ = solver.solve(q, target, reference=[q[n] for n in solver.names], dt=.01, target_stamp=10.)
        self.assertEqual(solver.tracking_feedback, 'reference')
        self.assertEqual(solver.names, (*GROUPS['waist'], *GROUPS['left_arm'], *GROUPS['right_arm']))
        self.assertEqual(len(output), 16)

    def test_removed_execution_modes_are_rejected_for_position_tracking(self):
        for mode in ('impedance', 'auto'):
            with self.subTest(mode=mode),self.assertRaisesRegex(ValueError,'SNOWBOT_EXECUTION_MODE'):
                apply_execution_tracking(self.config,mode,real=True,position_contract=True)
