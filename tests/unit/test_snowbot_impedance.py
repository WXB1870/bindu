"""Optional impedance selection and nominal tracking, without ROS or devices."""
from bindu_contracts.resources import package_share
import json
from pathlib import Path
import tempfile
import unittest

from bindu_runtime.teleop_deployment import apply_execution_tracking, load_deployment
from tests.support.paths import ROOT

SHARE = ROOT/'src/integration/bindu_runtime'


class ImpedanceSelectionTests(unittest.TestCase):
    def setUp(self):
        self.config = json.loads((package_share("snowbot_bringup")/'config/robots/snowbot/bimanual_snowbot_real.json').read_text())
        self.config.pop('tracking_feedback',None)  # Legacy callers without an explicit selection.
        self.config.pop('impedance_reference_limits',None)

    def test_legacy_position_tracking_is_unchanged_without_explicit_selection(self):
        _, args = load_deployment(package_share("snowbot_bringup")/'config/deployments/snowbot_vr_real.json', SHARE)
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
        config=json.loads((package_share("snowbot_bringup")/'config/robots/snowbot/bimanual_snowbot_real.json').read_text())
        self.assertEqual(apply_execution_tracking(config,real=True,position_contract=True,
            trajectory_reference=True)['tracking_feedback'],'measured_shared')

    def test_impedance_limits_are_explicit_and_do_not_change_position_recipe(self):
        config = dict(self.config, impedance_reference_limits={
            'max_reference_error': .22, 'max_target_lead_error': .1})
        selected = apply_execution_tracking(config, 'impedance', real=True,
                                            position_contract=True, trajectory_reference=True)
        self.assertEqual(selected['max_reference_error'], .22)
        self.assertEqual(selected['max_target_lead_error'], .1)
        self.assertEqual(selected['tracking_feedback'], 'reference')
        self.assertEqual(config['max_reference_error'], self.config['max_reference_error'])
        self.assertIs(apply_execution_tracking(config, real=True, position_contract=True), config)
        for limits in ({'max_reference_error':float('nan')}, {'max_target_lead_error':.3},
                       {'max_reference_error':True}, {'unknown':.1}, []):
            with self.subTest(limits=limits), self.assertRaisesRegex(ValueError,'REFERENCE_LIMITS_CONFIG'):
                apply_execution_tracking(dict(self.config,impedance_reference_limits=limits),
                                         'impedance',real=True,position_contract=True)

    def test_compliance_limit_does_not_expand_normal_target_advance(self):
        from bindu_contracts.teleoperation import reference_tracking_budget
        error,budget = reference_tracking_budget((.03,), (0.,), .22,
                                                executed=(.02,), target_limit=.1)
        self.assertAlmostEqual(error,.02)
        self.assertAlmostEqual(budget[0],.08)
        # A larger existing compliant displacement can persist but cannot grow
        # through another joint's unused budget or a relaxed hard threshold.
        error,budget = reference_tracking_budget((.18,.01), (0.,0.), .22,
                                                executed=(.17,.01), target_limit=.1)
        self.assertAlmostEqual(error,.17)
        self.assertAlmostEqual(budget[0],.18)
        self.assertAlmostEqual(budget[1],.08)
        error,_ = reference_tracking_budget((.18,), (0.,), .22,
                                            executed=(.221,), target_limit=.1)
        self.assertGreater(error,.22)
        for lead in (0.,-.1,.23,float('nan'),True):
            with self.subTest(lead=lead), self.assertRaisesRegex(ValueError,'TARGET_LEAD_LIMIT_CONFIG'):
                reference_tracking_budget((0.,),(0.,),.22,executed=(0.,),target_limit=lead)

    def test_changing_only_compliance_limit_preserves_or_tightens_original_advance(self):
        for hard,expected in ((.22,.15),(.1,.1)):
            config=dict(self.config,impedance_reference_limits={'max_reference_error':hard})
            selected=apply_execution_tracking(config,'impedance',real=True,
                                               position_contract=True,trajectory_reference=True)
            lead=selected.get('max_target_lead_error',selected['max_reference_error'])
            self.assertEqual(lead,expected)
        config=dict(self.config,max_target_lead_error=.08,
                    impedance_reference_limits={'max_reference_error':.22})
        selected=apply_execution_tracking(config,'impedance',real=True,position_contract=True,
                                           trajectory_reference=True)
        self.assertEqual(selected['max_target_lead_error'],.08)

    def test_real_impedance_only_widens_arm_bounds_and_retains_waist_and_advance(self):
        from bindu_contracts.teleoperation import joint_reference_limits, reference_tracking_budget
        raw=json.loads((package_share('snowbot_bringup')/'config/robots/snowbot/bimanual_snowbot_real.json').read_text())
        selected=apply_execution_tracking(raw,'impedance',real=True,position_contract=True)
        arms=[f'arm_{s}_joint_{j}' for s in ('l','r') for j in range(1,8)]
        self.assertEqual(selected['joint_reference_error_limits'],dict.fromkeys(arms,.25))
        self.assertEqual(selected['max_reference_error'],raw['max_reference_error'])
        self.assertEqual(selected.get('max_target_lead_error',selected['max_reference_error']),.15)
        self.assertNotIn('joint_reference_error_limits',apply_execution_tracking(raw,real=True,position_contract=True))
        names=('body_joint_1','arm_r_joint_3','body_joint_2','arm_l_joint_7')
        limits=joint_reference_limits(names,.15,{n:.25 for n in names if n in arms})
        reference=(0.,.21,0.,.02);measured=(0.,)*4
        error,budget=reference_tracking_budget(reference,measured,max(limits),executed=reference,
            target_limit=.15,joint_limits=limits)
        self.assertEqual(error,.21)
        self.assertEqual(budget,(.12,.21,.12,.12))
        for axis,value in ((0,.151),(2,.151),(1,.251),(3,.251)):
            displaced=list(reference);displaced[axis]=value
            with self.subTest(axis=axis),self.assertRaisesRegex(ValueError,'REFERENCE_TRACKING_ERROR'):
                reference_tracking_budget(reference,measured,max(limits),executed=displaced,
                    target_limit=.15,joint_limits=limits)
        for overrides in ({'unknown_joint':.25},{'body_joint_1':True},{'body_joint_1':float('nan')}):
            with self.subTest(overrides=overrides),self.assertRaisesRegex(ValueError,'REFERENCE_LIMIT_CONFIG'):
                joint_reference_limits(names,.15,overrides)

    def test_real_sixteen_axis_nominal_config_runs_native_qp(self):
        from bindu_contracts.profile import Profile
        from bindu_kinematics.bimanual import BimanualQP
        from snowbot_hardware.protocol import GROUPS
        raw=json.loads((package_share('snowbot_bringup')/'config/robots/snowbot/bimanual_snowbot_real.json').read_text())
        cfg = apply_execution_tracking(raw, 'impedance', real=True, position_contract=True)
        cfg['urdf'] = str(package_share("snowbot_description")/'urdf/snowbot_control.urdf')
        profile = Profile.load(package_share("snowbot_bringup")/'config/examples/snowbot/snowbot_vendor_position.json')
        solver = BimanualQP(cfg, profile);self.addCleanup(solver.close)
        q = json.loads((package_share("snowbot_bringup")/'config/examples/snowbot/bimanual_snowbot_shovel.json').read_text())['demo']['initial_positions_rad']
        target = solver.capture(q).copy();target[0,3] += .001
        output, _ = solver.solve(q, target, reference=[q[n] for n in solver.names], dt=.01, target_stamp=10.)
        self.assertEqual(solver.tracking_feedback, 'reference')
        self.assertEqual(solver.names, (*GROUPS['waist'], *GROUPS['left_arm'], *GROUPS['right_arm']))
        self.assertEqual(len(output), 16)

    def test_recipe_rejects_impedance_with_legacy_or_simulation(self):
        recipe = dict(schema_version=2, robot_launch='snowbot_adapter.launch.py', arguments=dict(
            runtime_mode='real', device_backend='snowbot_vendor', execution_mode='impedance', position_protocol='session_v1'))
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/'recipe.json'
            for changes, error in (({}, None), ({'position_protocol':'legacy_guard'}, 'REQUIRES_REAL_SESSION'),
                                    ({'runtime_mode':'simulation'}, 'REQUIRES_REAL_SESSION'),
                                    ({'execution_mode':'auto'}, 'EXECUTION_MODE')):
                data = {**recipe, 'arguments': {**recipe['arguments'], **changes}}
                if changes.get('position_protocol') == 'legacy_guard' or changes.get('runtime_mode') == 'simulation':
                    # Isolate mode compatibility from unrelated target-stream limits.
                    for key in ('trajectory_max_velocity','trajectory_max_acceleration','trajectory_max_jerk'):
                        data['arguments'].pop(key,None)
                path.write_text(json.dumps(data))
                if error:
                    with self.assertRaisesRegex(ValueError, error): load_deployment(path, SHARE)
                else:
                    _, args = load_deployment(path, SHARE)
                    self.assertEqual(args['execution_mode'], 'impedance')


class ImpedanceHealthTests(unittest.TestCase):
    def setUp(self):
        from types import SimpleNamespace as S
        from snowbot_hardware.session import SessionHealth, CONTROL_GROUPS
        from tests.unit.test_snowbot_position_session import observation
        self.groups = CONTROL_GROUPS
        self.health = SessionHealth(lambda: 10.001, lambda: 1.001, execution_mode='impedance')
        for layer in ('upper', 'middle', 'servo'):
            self.health.state(layer, S(header=observation().header, layer=layer, instance_id=12,
                startup_complete=True, feedback_ready=True, wire_version=3,
                control_source='topic' if layer == 'upper' else 'impedance', stop_policy='measured_hold'))
        self.health.observe(observation(wrench=[0.]*12, sensor_valid_mask=3, sensor_age=[.001,.001]))
        self.health.observe(observation(10.001, pdo_sample=2, wrench=[0.]*12, sensor_valid_mask=3, sensor_age=[.001,.001]))

    def test_matching_explicit_backend_and_force_sample_are_required(self):
        self.health.require(self.groups)
        self.health.states['servo'][1].control_source = 'position'
        with self.assertRaisesRegex(RuntimeError, 'EXECUTION_MODE_CONFLICT'):
            self.health.require(self.groups)
        self.health.states['servo'][1].control_source = 'impedance'
        for field, value in (('sensor_valid_mask',1),
                             ('wrench',[float('nan')]*12), ('valid_mask',0xc7f7f),
                             ('sensor_age',[.021,.001]), ('sensor_age',[-1.,.001]),
                             ('sensor_age',[float('nan'),.001])):
            obs = self.health.observation
            previous = getattr(obs, field);setattr(obs, field, value)
            with self.assertRaisesRegex(RuntimeError, 'SENSOR_INVALID'): self.health.require(self.groups)
            setattr(obs, field, previous)

    def test_target_stream_requires_matching_protocol_and_live_force_during_nominal_hold(self):
        self.health.reference_mode = 2
        for _, state in self.health.states.values():
            state.wire_version = 5
        self.health.states['servo'][1].reference_mode = 2
        self.health.require(self.groups)
        obs = self.health.observation
        obs.session_selected=True;obs.session=7;obs.command_sequence=3;obs.session_reason=1
        obs.hold_valid=True;obs.hold_sample=1;obs.hold_position=[.1]*16
        obs.reference_mode=2;obs.segment_id=3;obs.trajectory_phase=4;obs.hold_basis=0
        obs.reference_valid=obs.output_valid=True
        self.assertIsNotNone(self.health.confirmed_hold((7,3,obs.boot_id,1)))
        # A nominal hold ACK cannot stand in for a fresh, valid force sample.
        obs.sensor_age=[.021,.001]
        self.assertIn('SENSOR_INVALID',self.health.hold_failure((7,3,obs.boot_id,1)))

    def test_ros_delivery_delay_keeps_its_existing_budget_for_force_feedback(self):
        self.health.observation.sensor_age=[.01,.01]
        self.health.now=lambda:10.031
        self.health.steady=lambda:1.031
        self.health.require(self.groups)
        self.health.now=lambda:10.042
        self.health.steady=lambda:1.042
        with self.assertRaisesRegex(RuntimeError,'PDO_NOT_ADVANCING'):
            self.health.require(self.groups)

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
