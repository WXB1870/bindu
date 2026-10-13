"""Environment selection and provenance gates; no hardware or ROS processes."""
import importlib.util
from collections import deque
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace as S
import unittest
from unittest.mock import Mock, patch

from bindu_contracts.runtime_mode import resolve_mode, command_mode, message_matches
from bindu_runtime.teleop_deployment import load_deployment
from bindu_runtime.common import RuntimeNode, joints_at_rest
from bindu_runtime.execution_node import ExecutionNode
from bindu_contracts.contracts import Feedback
from bindu_runtime.recorder_node import RecorderNode
from bindu_interfaces.msg import ExecutionState, MotionCommand
import test_system_contracts as contracts_fixture

ROOT = Path(__file__).resolve().parents[1]
SHARE = ROOT/'src/integration/bindu_runtime'


class RuntimeModeTests(unittest.TestCase):
    def test_selection_and_legacy_alias_conflicts_are_explicit(self):
        self.assertEqual(resolve_mode(), 'simulation')
        self.assertEqual(resolve_mode(default='real'), 'real')
        self.assertEqual(resolve_mode(simulation=False), 'real')
        self.assertEqual(resolve_mode('real', False), 'real')
        for args in (('real', True), ('simulation', False), ('REAL', None), (True, None), ('', 'false')):
            with self.subTest(args=args), self.assertRaises(ValueError): resolve_mode(*args)

    def test_real_commands_require_a_mode_and_cannot_cross_environments(self):
        self.assertEqual(command_mode('', 'simulation'), 'simulation')
        for mode, expected in (('', 'real'), ('real', 'simulation'), ('simulation', 'real')):
            with self.subTest(mode=mode), self.assertRaises(ValueError): command_mode(mode, expected)

    def test_state_mode_and_legacy_boolean_must_agree(self):
        self.assertTrue(message_matches(S(runtime_mode='real', simulated=False), 'real'))
        self.assertTrue(message_matches(S(simulated=True), 'simulation'))
        for msg in (S(runtime_mode='simulation', simulated=True), S(runtime_mode='real', simulated=True),
                    S(runtime_mode='', simulated=False), S(runtime_mode='real', simulated=0)):
            self.assertFalse(message_matches(msg, 'real'))

    def test_wrong_mode_command_cannot_advance_or_write_existing_execution(self):
        fixture = contracts_fixture.Contracts(); fixture.setUp(); self.addCleanup(fixture.doCleanups)
        fixture.e.submit(fixture.m, 10.)
        revision, active = fixture.e.revision, fixture.e.active
        with patch.object(fixture.io, 'write_joints') as write:
            with self.assertRaisesRegex(ValueError, 'RUNTIME_MODE_MISMATCH'):
                fixture.e.submit(replace(fixture.m, command_id='wrong', runtime_mode='real'), 10.01)
            write.assert_not_called()
        self.assertEqual(fixture.e.revision, revision)
        self.assertIs(fixture.e.active, active)

    def test_wrong_feedback_mode_revokes_execution_and_keeps_last_valid_snapshot(self):
        fixture = contracts_fixture.Contracts(); fixture.setUp(); self.addCleanup(fixture.doCleanups)
        fixture.e.submit(fixture.m, 10.)
        previous = fixture.e.feedback
        with patch.object(fixture.io, 'read_feedback', return_value=replace(previous, simulated=False)):
            fixture.e.tick(10.01)
        self.assertFalse(fixture.e.lease_id)
        self.assertIs(fixture.e.feedback, previous)
        self.assertEqual(fixture.e.results['cmd'].code, 'FEEDBACK_RUNTIME_MODE_MISMATCH')

    def test_configuration_selects_one_mode_for_execution_and_recorder(self):
        spec = importlib.util.spec_from_file_location('mode_launch', SHARE/'launch/snowbot_adapter.launch.py')
        module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
        from launch import LaunchContext
        for mode, executable, backend in [('simulation', 'execution', 'kinematic'),
                                          ('real', 'snowbot_adapter', 'snowbot_vendor')]:
            context = LaunchContext();context.launch_configurations.update(runtime_mode=mode,command_enabled='false')
            with patch.object(module, 'get_package_share_directory', return_value=str(SHARE)), \
                    patch.object(module, 'Node', side_effect=lambda **kw: kw):
                actions = module.assemble(context)
            self.assertEqual(actions[0]['executable'], executable)
            self.assertTrue(all(a['parameters'][0]['runtime_mode'] == mode for a in actions))
            self.assertEqual(actions[1]['parameters'][0]['device_backend'], backend)
            context.launch_configurations['device_backend'] = 'snowbot_vendor' if mode == 'simulation' else 'kinematic'
            with self.assertRaisesRegex(ValueError, 'BACKEND_CONFLICT'): module.assemble(context)

    def test_mixed_state_and_command_are_not_written_to_real_recording(self):
        node = RecorderNode.__new__(RecorderNode)
        node.runtime_mode, node.input_error = 'real', ''
        node.writer = Mock()
        node.record('feedback', ExecutionState(runtime_mode='simulation', simulated=True))
        node.record('reference', MotionCommand(runtime_mode='simulation'))
        node.writer.submit.assert_not_called()
        self.assertEqual(node.input_error, 'RECORDING_RUNTIME_MODE_MISMATCH')

    def test_bad_source_clears_cached_state_instead_of_reusing_it(self):
        node = RuntimeNode.__new__(RuntimeNode);node.runtime_mode = 'simulation'
        node.state = ExecutionState(runtime_mode='simulation', simulated=True)
        node.store_mode_message('state', ExecutionState(runtime_mode='real', simulated=False))
        self.assertIsNone(node.state)

    def test_default_runtime_recipe_is_simulation_and_readonly_hardware_permission(self):
        launch, args = load_deployment(SHARE/'config/runtime_deployment.json', SHARE)
        self.assertEqual(launch, 'snowbot_adapter.launch.py')
        self.assertEqual(args['runtime_mode'], 'simulation')
        self.assertEqual(args['command_enabled'], 'false')
        self.assertEqual(args['recording_mode'], 'compact')
        self.assertNotIn('device_backend', args)

    def test_unknown_partial_duplicate_and_nonfinite_velocities_are_not_rest(self):
        self.assertTrue(joints_at_rest(S(name=['j'],velocity=[0.]),['j']))
        for joints in (S(name=['j'],velocity=[]),S(name=['j','k'],velocity=[0.]),
                       S(name=['j','j'],velocity=[0.,0.]),S(name=['k'],velocity=[0.]),
                       S(name=['j'],velocity=[float('nan')]),S(name=['j'],velocity=[.1])):
            with self.subTest(joints=joints):self.assertFalse(joints_at_rest(joints,['j']))

    def test_published_unknown_velocity_is_empty_and_shutdown_rejects_input(self):
        node=ExecutionNode.__new__(ExecutionNode)
        node.engine=S(feedback=Feedback(10.,{'j':.3},0.,0.,(0.,0.)),tick=lambda now:None,
                      events=deque(),active=None,revision=1,lease_id='',epoch=1,groups=(),reference=None,stopping=None)
        node.robot_io=S(feedback=S(source_stamps={'arm':10.}),reference_positions={},stop_failures=())
        node.profile=S(digest='hash');node.instance_id='test'
        node.runtime_mode='simulation';node.device_backend='external_simulation'
        node.state_pub=Mock();node.last_event=None
        with patch.object(node,'now',return_value=10.):node.tick()
        state=node.state_pub.publish.call_args.args[0]
        self.assertEqual(list(state.joints.velocity),[])
        self.assertEqual(list(state.reference.name),[])
        node.closing=True
        with self.assertRaisesRegex(ValueError,'SHUTDOWN'):
            node.accept(MotionCommand(runtime_mode='simulation'))


if __name__ == '__main__':
    unittest.main()
