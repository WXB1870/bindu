"""Optional impedance selection and nominal tracking, without ROS or devices."""
import json
from pathlib import Path
import tempfile
import unittest

from bindu_runtime.teleop_deployment import apply_execution_tracking, load_deployment
from tests.support.paths import ROOT

SHARE = ROOT/'src/integration/bindu_runtime'


class ImpedanceSelectionTests(unittest.TestCase):
    def setUp(self):
        self.config = json.loads((SHARE/'config/robots/snowbot/bimanual_snowbot_real.json').read_text())
        self.config.pop('tracking_feedback',None)  # Legacy callers without an explicit selection.

    def test_legacy_position_tracking_is_unchanged_without_explicit_selection(self):
        _, args = load_deployment(SHARE/'config/deployments/snowbot_vr_real.json', SHARE)
        self.assertEqual(args['execution_mode'], 'position')
        self.assertEqual(args['command_enabled'], 'false')
        self.assertIs(apply_execution_tracking(self.config, real=True, position_contract=True), self.config)
        self.assertEqual(self.config.get('tracking_feedback', 'measured'), 'measured')

    def test_impedance_selects_nominal_tracking_and_preserves_measured_guards(self):
        selected = apply_execution_tracking(self.config, 'impedance', real=True, position_contract=True)
        self.assertEqual(selected['tracking_feedback'], 'reference')
        self.assertEqual({k:v for k,v in selected.items() if k != 'tracking_feedback'}, self.config)
        self.assertNotIn('tracking_feedback', self.config)
        for real, contract in ((False, True), (True, False)):
            with self.assertRaisesRegex(ValueError, 'REQUIRES_REAL'):
                apply_execution_tracking(self.config, 'impedance', real=real, position_contract=contract)
        with self.assertRaisesRegex(ValueError, 'NOMINAL_KINEMATICS'):
            apply_execution_tracking({**self.config, 'reference_kinematics': False}, 'impedance', real=True, position_contract=True)

    def test_position_target_stream_pairs_accumulation_with_nominal_error(self):
        selected=apply_execution_tracking(self.config,real=True,position_contract=True,
                                          trajectory_reference=True)
        self.assertEqual(selected['tracking_feedback'],'reference')
        self.assertEqual({k:v for k,v in selected.items() if k!='tracking_feedback'},self.config)
        self.assertNotIn('tracking_feedback',self.config)
        with self.assertRaisesRegex(ValueError,'TRAJECTORY_REQUIRES_REAL_NOMINAL'):
            apply_execution_tracking(dict(self.config,reference_kinematics=False),real=True,
                                     position_contract=True,trajectory_reference=True)

    def test_position_preserves_explicit_measured_task_but_impedance_stays_nominal(self):
        for source in ('measured_task','measured_shared'):
            with self.subTest(source=source):
                config=dict(self.config,tracking_feedback=source)
                selected=apply_execution_tracking(config,real=True,position_contract=True,
                                                   trajectory_reference=True)
                self.assertEqual(selected['tracking_feedback'],source)
                self.assertEqual(config['tracking_feedback'],source)
                self.assertEqual(apply_execution_tracking(config,'impedance',real=True,
                    position_contract=True)['tracking_feedback'],'reference')

    def test_real_recipe_selects_measured_waist_with_continuous_arm_reference(self):
        config=json.loads((SHARE/'config/robots/snowbot/bimanual_snowbot_real.json').read_text())
        self.assertEqual(apply_execution_tracking(config,real=True,position_contract=True,
            trajectory_reference=True)['tracking_feedback'],'measured_shared')

    def test_real_sixteen_axis_nominal_config_runs_native_qp(self):
        from bindu_contracts.profile import Profile
        from bindu_kinematics.bimanual import BimanualQP
        from bindu_hardware.snowbot import GROUPS
        cfg = apply_execution_tracking(self.config, 'impedance', real=True, position_contract=True)
        cfg['urdf'] = str(ROOT/'src/hardware/bindu_description/urdf/snowbot_control.urdf')
        profile = Profile.load(SHARE/'config/robots/snowbot/snowbot_vendor_position.json')
        solver = BimanualQP(cfg, profile);self.addCleanup(solver.close)
        q = json.loads((SHARE/'config/robots/snowbot/bimanual_snowbot_shovel.json').read_text())['demo']['initial_positions_rad']
        target = solver.capture(q).copy();target[0,3] += .001
        output, _ = solver.solve(q, target, reference=[q[n] for n in solver.names], dt=.01, target_stamp=10.)
        self.assertEqual(solver.tracking_feedback, 'reference')
        self.assertEqual(solver.names, (*GROUPS['waist'], *GROUPS['left_arm'], *GROUPS['right_arm']))
        self.assertEqual(len(output), 16)

    def test_recipe_rejects_impedance_with_legacy_or_simulation(self):
        recipe = json.loads((SHARE/'config/deployments/snowbot_vr_real.json').read_text())
        recipe['arguments']['execution_mode'] = 'impedance'
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/'recipe.json'
            for changes, error in (({}, None), ({'position_protocol':'legacy_guard'}, 'REQUIRES_REAL_SESSION'),
                                    ({'runtime_mode':'simulation'}, 'REQUIRES_REAL_SESSION'),
                                    ({'execution_mode':'auto'}, 'EXECUTION_MODE')):
                data = {**recipe, 'arguments': {**recipe['arguments'], **changes}}
                path.write_text(json.dumps(data))
                if error:
                    with self.assertRaisesRegex(ValueError, error): load_deployment(path, SHARE)
                else:
                    _, args = load_deployment(path, SHARE)
                    self.assertEqual(args['execution_mode'], 'impedance')


class ImpedanceHealthTests(unittest.TestCase):
    def setUp(self):
        from types import SimpleNamespace as S
        from bindu_hardware.snowbot_session import SessionHealth, CONTROL_GROUPS
        from tests.unit.test_snowbot_position_session import observation
        self.groups = CONTROL_GROUPS
        self.health = SessionHealth(lambda: 10.001, lambda: 1.001, execution_mode='impedance')
        for layer in ('upper', 'middle', 'servo'):
            self.health.state(layer, S(header=observation().header, layer=layer, instance_id=12,
                startup_complete=True, feedback_ready=True,
                control_source='topic' if layer == 'upper' else 'impedance', stop_policy='measured_hold'))
        self.health.observe(observation(wrench=[0.]*12, sensor_valid_mask=3, sensor_age=[.001,.001]))
        self.health.observe(observation(10.001, pdo_sample=2, wrench=[0.]*12, sensor_valid_mask=3, sensor_age=[.001,.001]))

    def test_matching_explicit_backend_and_force_sample_are_required(self):
        self.health.require(self.groups)
        self.health.states['servo'][1].control_source = 'position'
        with self.assertRaisesRegex(RuntimeError, 'EXECUTION_MODE_CONFLICT'):
            self.health.require(self.groups)
        self.health.states['servo'][1].control_source = 'impedance'
        for field, value in (('sensor_valid_mask',1), ('wrench',[0.]*11),
                             ('wrench',[float('nan')]*12), ('valid_mask',0xc7f7f),
                             ('sensor_age',[.021,.001]), ('sensor_age',[-1.,.001]),
                             ('sensor_age',[float('nan'),.001])):
            obs = self.health.observation
            previous = getattr(obs, field);setattr(obs, field, value)
            with self.assertRaisesRegex(RuntimeError, 'SENSOR_INVALID'): self.health.require(self.groups)
            setattr(obs, field, previous)

    def test_default_position_requires_no_impedance_sensor_or_calibration(self):
        self.health.execution_mode = 'position'
        self.health.states['servo'][1].control_source = 'position'
        del self.health.observation.wrench
        del self.health.observation.sensor_valid_mask
        self.health.require(self.groups)
        self.health.states['servo'][1].control_source = 'impedance'
        with self.assertRaisesRegex(RuntimeError, 'EXECUTION_MODE_CONFLICT'):
            self.health.require(self.groups)

    def test_unready_calibration_state_cannot_become_ready_from_valid_force(self):
        self.health.states['servo'][1].feedback_ready = False
        with self.assertRaisesRegex(RuntimeError, 'LAYER_UNAVAILABLE:servo'):
            self.health.require(self.groups)
