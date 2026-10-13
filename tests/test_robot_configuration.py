"""Robot selection and position protocol tests. Never opens a hardware endpoint."""
import copy
import json
import math
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET
from unittest.mock import Mock, patch

from bindu_contracts.profile import Profile
from bindu_hardware.position_mapping import PositionMapping
from bindu_hardware.soem_feedback import SoemFeedbackGate
from bindu_runtime.device_assembly import assemble_devices
from bindu_runtime.robot_setup import RobotSetup
from bindu_runtime.teleop_deployment import load_deployment

ROOT = Path(__file__).resolve().parents[1]
RUNTIME = ROOT/'src/integration/bindu_runtime'
CONFIG = RUNTIME/'config'
DESCRIPTION = ROOT/'src/hardware/bindu_description'
PACKAGES = {'bindu_runtime': RUNTIME, 'bindu_description': DESCRIPTION}


def fixture(folder, arms=7, waist=2, positive_limit=False):
    """Independent serial chains with configurable axis count; no robot names."""
    groups = ({'trunk': [f'spine{i}' for i in range(waist)]} if waist else {})
    groups.update(port=[f'a{i}' for i in range(arms)], starboard=[f'b{i}' for i in range(arms)])
    groups['camera'] = ['camera_pan']
    xml = ['<robot name="fixture"><link name="root"/>']
    limits = {}
    for group, names in groups.items():
        parent = f'spine{waist-1}_link' if waist and group in ('port', 'starboard') else 'root'
        for index, name in enumerate(names):
            axis = ('1 0 0', '0 1 0', '0 0 1')[index % 3]
            xyz = ('0 .3 0' if group == 'port' else '0 -.3 0') if index == 0 and group in ('port', 'starboard') else '.1 0 0'
            lo = .2 if positive_limit and name == 'camera_pan' else -1.
            limits[name] = [lo, 1.]
            xml.append(f'<link name="{name}_link"/><joint name="{name}" type="revolute">'
                       f'<parent link="{parent}"/><child link="{name}_link"/>'
                       f'<origin xyz="{xyz}"/><axis xyz="{axis}"/>'
                       f'<limit lower="{lo}" upper="1" velocity="1" effort="10"/></joint>')
            parent = name+'_link'
    xml.append('</robot>')
    (folder/'robot.urdf').write_text(''.join(xml))
    profile = {'name': 'fixture', 'groups': groups, 'limits': limits, 'max_speed': .4,
               'capabilities': ['joint_position']}
    (folder/'profile.json').write_text(json.dumps(profile))
    for side, group in [('left', 'port'), ('right', 'starboard')]:
        config = json.loads((CONFIG/f'teleop_snowbot_{side}.json').read_text())
        config['resource_group'] = group
        config['kinematics'].update(urdf='robot.urdf', joint_names=groups[group],
                                   ee_link=groups[group][-1]+'_link',
                                   locked_joints={n: max(0., limits[n][0]) for n in limits if n not in groups[group]})
        (folder/f'{side}.json').write_text(json.dumps(config))
    tool = json.loads((CONFIG/'bimanual_g1_sim.json').read_text())
    tool.update(urdf='robot.urdf', resource_groups=list(groups)[:-1],
                input_frame='root',
                left_frame=groups['port'][-1]+'_link', right_frame=groups['starboard'][-1]+'_link',
                flange_distance_range=[.2, 1.])
    (folder/'tool.json').write_text(json.dumps(tool))
    selection = {'schema_version': 1, 'robot_id': 'fixture',
                 'model': {'urdf': 'robot.urdf', 'display_urdf': 'robot.urdf', 'base_frame': 'root'},
                 'profile': 'profile.json', 'teleop': {'single_arm': {'left': 'left.json', 'right': 'right.json'},
                                                     'tool': 'tool.json'}, 'position_mapping': None}
    path = folder/'robot.json'; path.write_text(json.dumps(selection))
    return path


class RobotConfigurationTests(unittest.TestCase):
    def test_existing_models_and_second_real_robot_are_distinct(self):
        snow = RobotSetup.load(CONFIG/'robot_snowbot.json', PACKAGES)
        g1 = RobotSetup.load(CONFIG/'robot_g1_fixture.json', PACKAGES)
        self.assertEqual((len(snow.active_joints), len(snow.context_joints)), (16, 4))
        self.assertEqual((len(g1.active_joints), len(g1.context_joints)), (16, 4))
        self.assertNotEqual(snow.active_joints, g1.active_joints)
        with self.assertRaisesRegex(ValueError, 'INCOMPLETE:wheel_gen2'):
            RobotSetup.load(CONFIG/'robot_wheel_gen2.json', PACKAGES)

    def test_axis_counts_are_derived_and_native_qp_accepts_each_layout(self):
        from bindu_kinematics.bimanual import BimanualQP
        for arms, waist in [(6, 0), (7, 2), (7, 3)]:
            with self.subTest(arms=arms, waist=waist), tempfile.TemporaryDirectory() as tmp:
                robot = RobotSetup.load(fixture(Path(tmp), arms, waist))
                self.assertEqual(len(robot.active_joints), 2*arms+waist)
                self.assertEqual(robot.context_joints, ('camera_pan',))
                config = json.loads(robot.tool_config.read_text()); config['urdf'] = str(robot.urdf)
                qp = BimanualQP(config, robot.profile)
                try:
                    self.assertEqual(tuple(qp.names), robot.active_joints)
                    feedback = dict.fromkeys(robot.profile.limits, 0.)
                    target = qp.capture(feedback)
                    positions, _ = qp.solve(feedback, target, reference=[0.]*len(qp.names), dt=.01)
                    self.assertEqual(len(positions), len(robot.active_joints))
                    self.assertTrue(all(math.isfinite(v) for v in positions))
                finally:
                    qp.close()

    def test_positive_only_limits_and_simulated_initial_position(self):
        for group in ('camera', 'hand'):
            with self.subTest(group=group), tempfile.TemporaryDirectory() as tmp:
                robot = RobotSetup.load(fixture(Path(tmp), positive_limit=True))
                if group == 'hand':
                    data = json.loads(robot.profile_path.read_text())
                    data['groups']['hand'] = data['groups'].pop('camera')
                    robot.profile_path.write_text(json.dumps(data))
                    robot = RobotSetup.load(robot.path)
                params = {}
                node = Mock()
                node.runtime_mode = 'simulation'
                node.declare_parameter.side_effect = lambda key, value, **kwargs: params.setdefault(key, value)
                node.get_parameter.side_effect = lambda key: Mock(value=params[key])
                io, _ = assemble_devices(node, robot.profile)
                feedback = io.read_feedback(1., .01)
                self.assertEqual(feedback.positions['camera_pan'], .2)

    def test_mismatched_limits_names_and_missing_context_are_rejected(self):
        for change, code in [('limit', 'LIMIT_EXCEEDS_MODEL'), ('name', 'PROFILE_LAYOUT'),
                             ('context', 'CONTEXT_LAYOUT'), ('frame', 'TOOL_MODEL')]:
            with self.subTest(change=change), tempfile.TemporaryDirectory() as tmp:
                path = fixture(Path(tmp))
                if change in ('limit', 'name'):
                    p = Path(tmp)/'profile.json'; data = json.loads(p.read_text())
                    if change == 'limit': data['limits']['a0'] = [-2., 1.]
                    else:
                        data['groups']['camera'] = ['wrong']; data['limits']['wrong'] = data['limits'].pop('camera_pan')
                elif change == 'context':
                    p = Path(tmp)/'left.json'; data = json.loads(p.read_text())
                    del data['kinematics']['locked_joints']['camera_pan']
                else:
                    p = Path(tmp)/'tool.json'; data = json.loads(p.read_text()); data['left_frame'] = 'missing'
                p.write_text(json.dumps(data))
                with self.assertRaisesRegex(ValueError, code): RobotSetup.load(path)

    def test_display_model_cannot_change_control_kinematics(self):
        for change in ('axis', 'origin', 'parent', 'type', 'extra_axis'):
            with self.subTest(change=change), tempfile.TemporaryDirectory() as tmp:
                folder = Path(tmp); path = fixture(folder)
                tree = ET.parse(folder/'robot.urdf')
                joint = tree.getroot().find("joint[@name='a0']")
                if change == 'axis': joint.find('axis').set('xyz', '0 0 1')
                elif change == 'origin': joint.find('origin').set('xyz', '0 .9 0')
                elif change == 'parent': joint.find('parent').set('link', 'root')
                elif change == 'type': joint.set('type', 'prismatic')
                else:
                    ET.SubElement(tree.getroot(), 'link', name='extra')
                    extra = ET.SubElement(tree.getroot(), 'joint', name='extra_axis', type='revolute')
                    ET.SubElement(extra, 'parent', link='root'); ET.SubElement(extra, 'child', link='extra')
                    ET.SubElement(extra, 'limit', lower='-1', upper='1', velocity='1', effort='10')
                tree.write(folder/'display.urdf')
                raw = json.loads(path.read_text()); raw['model']['display_urdf'] = 'display.urdf'
                path.write_text(json.dumps(raw))
                with self.assertRaisesRegex(ValueError, 'ROBOT_DISPLAY_'):
                    RobotSetup.load(path)

    def test_tool_collision_model_must_match_selected_robot(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp); path = fixture(folder)
            (folder/'other.urdf').write_bytes((folder/'robot.urdf').read_bytes())
            tool_path = folder/'tool.json'; tool = json.loads(tool_path.read_text())
            tool.update(collision_scope='configured_tool_geometry', demo={'display_urdf': 'other.urdf'})
            tool_path.write_text(json.dumps(tool))
            with self.assertRaisesRegex(ValueError, 'ROBOT_TOOL_COLLISION_MODEL'):
                RobotSetup.load(path)

    def test_configuration_digest_changes_with_limits(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = fixture(Path(tmp)); before = RobotSetup.load(path)
            p = Path(tmp)/'profile.json'; data = json.loads(p.read_text()); data['max_speed'] = .2
            p.write_text(json.dumps(data))
            self.assertNotEqual(before.digest, RobotSetup.load(path).digest)

    def test_logging_off_keeps_required_health_publisher(self):
        from launch import LaunchContext
        from bindu_runtime.teleop_assembly import assemble
        robot = RobotSetup.load(CONFIG/'robot_snowbot.json', PACKAGES)
        context = LaunchContext()
        context.launch_configurations.update(robot_config=str(robot.path), side='left',
            device_backend='kinematic', tool_input_source='tool_pose', recording_mode='off',
            description_enabled='false')
        with patch('bindu_runtime.teleop_assembly.RobotSetup') as loader, \
                patch('bindu_runtime.teleop_assembly.Node', side_effect=lambda **kw: kw):
            loader.load.return_value = robot
            actions = assemble(context)
        recorder = [a for a in actions if isinstance(a, dict) and a['executable'] == 'recorder']
        self.assertEqual(len(recorder), 1)
        self.assertEqual(recorder[0]['parameters'][0]['recording_mode'].evaluate(context), 'off')

    def test_new_deployment_has_no_startup_pose_and_legacy_still_loads(self):
        launch, args = load_deployment(CONFIG/'teleop_robot_deployment.json', RUNTIME)
        self.assertEqual(launch, 'robot_teleop.launch.py')
        self.assertEqual(Path(args['robot_config']), CONFIG/'robot_snowbot.json')
        self.assertNotIn('initialize_tool_pose', args)
        self.assertEqual(load_deployment(CONFIG/'teleop_snowbot_deployment.json', RUNTIME)[0], 'snowbot_sim.launch.py')


class PositionMappingTests(unittest.TestCase):
    def setUp(self):
        self.config = json.loads((CONFIG/'position_mapping_wheel_gen2.json').read_text())
        self.mapping = PositionMapping(self.config, self.config['joints'])
        self.feedback = {n: .01*i for i, n in enumerate(self.config['channels'])}

    def test_whole_frame_round_trip_and_torso_sign(self):
        q = self.mapping.capture(dict(reversed(list(self.feedback.items()))))
        self.assertEqual(q['torso_joint'], -self.feedback['tihu_t1'])
        self.assertEqual(self.mapping.encode(q), tuple(self.feedback.values()))
        q['arm_l_joint_2'] = .5
        result = dict(zip(self.mapping.channels, self.mapping.encode(q)))
        self.assertEqual(result['arm_l_joint_2'], .5)
        self.assertEqual(len(result), 48)
        for n in self.mapping.hold_channels: self.assertEqual(result[n], self.feedback[n])

    def test_no_default_zero_or_old_capture_after_failure(self):
        q = self.mapping.decode(self.feedback)
        with self.assertRaisesRegex(ValueError, 'NOT_CAPTURED'): self.mapping.encode(q)
        self.mapping.capture(self.feedback)
        bad = dict(self.feedback); bad['tihu_t1'] = float('nan')
        with self.assertRaisesRegex(ValueError, 'FEEDBACK'): self.mapping.capture(bad)
        with self.assertRaisesRegex(ValueError, 'NOT_CAPTURED'): self.mapping.encode(q)

    def test_degrees_offsets_permutations_and_duplicate_destinations(self):
        cfg = copy.deepcopy(self.config); cfg['channels'].reverse()
        cfg['joints']['arm_l_joint_1'].update(unit='deg', direction=-1, offset=10.)
        mapping = PositionMapping(cfg, cfg['joints']); q = mapping.capture(self.feedback)
        q['arm_l_joint_1'] = math.pi/2
        encoded = dict(zip(mapping.channels, mapping.encode(q)))
        self.assertAlmostEqual(encoded['arm_l_joint_1'], -80.)
        self.assertAlmostEqual(mapping.decode(encoded)['arm_l_joint_1'], math.pi/2)
        cfg['joints']['arm_l_joint_2']['channel'] = 'arm_l_joint_1'
        with self.assertRaisesRegex(ValueError, 'COVERAGE'): PositionMapping(cfg, cfg['joints'])

    def test_calibration_overflow_invalidates_previous_capture(self):
        config = copy.deepcopy(self.config)
        config['joints']['arm_l_joint_1']['offset'] = 1e308
        mapping = PositionMapping(config, config['joints'])
        feedback = dict(self.feedback); feedback['arm_l_joint_1'] = 1e308
        positions = mapping.capture(feedback)
        feedback['arm_l_joint_1'] = -1e308
        with self.assertRaisesRegex(ValueError, 'FEEDBACK'):
            mapping.decode(feedback)
        with self.assertRaisesRegex(ValueError, 'NOT_CAPTURED'):
            mapping.encode(positions)

    def test_incomplete_feedback_and_partial_targets_rejected(self):
        q = self.mapping.capture(self.feedback)
        incomplete = dict(self.feedback); incomplete.pop('tihu_t4')
        with self.assertRaisesRegex(ValueError, 'FEEDBACK'): self.mapping.decode(incomplete)
        with self.assertRaisesRegex(ValueError, 'NOT_CAPTURED'): self.mapping.encode(q)
        q = self.mapping.capture(self.feedback); del q['arm_l_joint_2']
        with self.assertRaisesRegex(ValueError, 'TARGET'): self.mapping.encode(q)


class SoemFeedbackGateTests(unittest.TestCase):
    def setUp(self):
        self.config = json.loads((CONFIG/'position_mapping_wheel_gen2.json').read_text())
        self.names = self.config['channels']
        self.positions = [.01*i for i in range(48)]
        self.gate = SoemFeedbackGate(self.config)
        self.healthy = 'alive=1 state=1 error=0 ready=1 hand_held_mask=0 write_enable=0'

    def test_read_only_frame_preserves_source_time_and_sign(self):
        self.assertIsNone(self.gate.take(10.))
        self.gate.observe_status(self.healthy, 10.)
        self.gate.observe_joint_state(self.names, self.positions, 10.01, 10.02)
        stamp, mapped = self.gate.take(10.03)
        self.assertEqual(stamp, 10.01)
        self.assertEqual(len(mapped), 16)
        self.assertEqual(mapped['torso_joint'], -self.positions[14])
        self.assertIsNone(self.gate.take(10.04))

    def test_fault_or_stale_status_never_relabels_old_angles(self):
        self.gate.observe_status(self.healthy, 10.)
        self.gate.observe_joint_state(self.names, self.positions, 10.01, 10.02)
        self.assertIsNone(self.gate.take(10.21))
        self.gate.observe_status(self.healthy.replace('alive=1', 'alive=0'), 10.22)
        self.assertIsNone(self.gate.take(10.22))
        self.gate.observe_status(self.healthy, 10.23)
        self.assertIsNone(self.gate.take(10.23))
        self.gate.observe_joint_state(self.names, self.positions, 10.24, 10.25)
        self.assertIsNotNone(self.gate.take(10.25))
        with self.assertRaisesRegex(ValueError, 'DUPLICATE'):
            self.gate.observe_status(self.healthy + ' alive=1', 10.26)
        self.assertIsNone(self.gate.take(10.26))

    def test_invalid_frames_and_hand_feedback_are_rejected(self):
        self.gate.observe_status(self.healthy, 10.)
        cases = [
            (self.names[:-1] + [self.names[0]], self.positions, 10.01, 10.02),
            (self.names, self.positions[:-1] + [float('nan')], 10.01, 10.02),
            (self.names, self.positions, 9.7, 10.02),
            (self.names, self.positions, 10.2, 10.02),
        ]
        for names, positions, stamp, received in cases:
            with self.subTest(stamp=stamp, names=len(set(names)), finite=math.isfinite(positions[-1])):
                with self.assertRaisesRegex(ValueError, 'SOEM_JOINT_STATE_INVALID'):
                    self.gate.observe_joint_state(names, positions, stamp, received)
                self.assertIsNone(self.gate.take(received))
        self.gate.observe_joint_state(self.names, self.positions, 10.03, 10.04)
        self.assertIsNotNone(self.gate.take(10.04))
        with self.assertRaisesRegex(ValueError, 'SOEM_JOINT_STATE_INVALID'):
            self.gate.observe_joint_state(self.names, self.positions, 10.02, 10.05)


if __name__ == '__main__':
    unittest.main()
