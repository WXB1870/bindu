"""Deployment helper checks without launching devices or acquiring control."""
from tests.support.paths import ROOT
import copy
import importlib.util
import json
from pathlib import Path
import subprocess
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

spec = importlib.util.spec_from_file_location('snowbot_teleop_tools', ROOT/'tools/deployment/snowbot_teleop.py')
helper = importlib.util.module_from_spec(spec)
spec.loader.exec_module(helper)


class TeleopToolsTests(unittest.TestCase):
    def setUp(self):
        self.recipe = json.loads(helper.RECIPE.read_text())

    def test_configure_is_readonly_and_does_not_replace_existing_file(self):
        with tempfile.TemporaryDirectory() as tmp:
            config = Path(tmp)/'site.json'
            self.assertEqual(helper.main(['configure', '--config', str(config)]), 0)
            self.assertFalse(json.loads(config.read_text())['arguments']['command_enabled'])
            before = config.read_bytes()
            self.assertEqual(helper.main(['configure', '--config', str(config), '--enable-commands']), 2)
            self.assertEqual(config.read_bytes(), before)

    def test_writable_config_is_only_generated_with_explicit_option(self):
        with tempfile.TemporaryDirectory() as tmp:
            config = Path(tmp)/'write.json'
            self.assertEqual(helper.main(['configure', '--config', str(config), '--enable-commands']), 0)
            self.assertTrue(json.loads(config.read_text())['arguments']['command_enabled'])

    def test_network_listener_requires_tls_pair_before_creating_file(self):
        with tempfile.TemporaryDirectory() as tmp:
            config = Path(tmp)/'site.json'
            self.assertEqual(helper.main(['configure', '--config', str(config), '--host', '0.0.0.0']), 2)
            self.assertFalse(config.exists())
            self.assertEqual(helper.main(['configure', '--config', str(config), '--cert-file', 'cert.pem']), 2)
            self.assertFalse(config.exists())

    def test_network_config_has_absolute_paths_and_explicit_vendor_root(self):
        with tempfile.TemporaryDirectory() as tmp:
            config = Path(tmp)/'site.json'
            self.assertEqual(helper.main(['configure', '--config', str(config), '--host', '0.0.0.0',
                                         '--cert-file', 'cert.pem', '--key-file', 'key.pem',
                                         '--vendor-namespace', '/']), 0)
            args = json.loads(config.read_text())['arguments']
            self.assertTrue(Path(args['cert_file']).is_absolute())
            self.assertEqual(args['vendor_namespace'], '/')
            self.assertFalse(args['command_enabled'])

    def test_minimal_recipe_rejects_simulation_and_extra_nodes(self):
        for key, value in (('runtime_mode', 'simulation'), ('device_backend', 'kinematic'),
                           ('command_enabled', 'false'), ('pico_enabled', True),
                           ('single_arm_enabled', True), ('recorder_enabled', True),
                           ('tool_input_source', 'tool_pose'), ('description_enabled', True)):
            with self.subTest(key=key):
                changed = copy.deepcopy(self.recipe)
                changed['arguments'][key] = value
                with self.assertRaises(ValueError):
                    helper.validate_recipe(changed)

    def test_bad_names_and_ports_fail_closed(self):
        for key, value in (('namespace', 'bad/name/'), ('vendor_namespace', 'snowbot'),
                           ('servo_receiver_node', '/'), ('arm_receiver_node', 'servo'),
                           ('port', 0), ('port', True), ('host', '')):
            with self.subTest(key=key, value=value):
                changed = copy.deepcopy(self.recipe)
                changed['arguments'][key] = value
                with self.assertRaises(ValueError):
                    helper.validate_recipe(changed)

    def test_start_checks_before_exec_and_propagates_exact_config_path(self):
        with tempfile.TemporaryDirectory(prefix='bindu site ') as tmp:
            config = Path(tmp)/'site.json'
            with patch.object(helper, 'preflight', return_value=({}, None)) as check, \
                    patch.object(helper, 'ensure_stopped') as stopped, \
                    patch.object(helper.os, 'execvp') as execute:
                self.assertEqual(helper.main(['start', '--config', str(config)]), 0)
                check.assert_called_once_with(config)
                stopped.assert_called_once_with({})
                execute.assert_called_once_with('ros2', ['ros2', 'launch', 'bindu_runtime',
                                                         'teleop_only.launch.py', 'config:='+str(config)])

    def test_failed_preflight_never_starts_launch(self):
        with patch.object(helper, 'preflight', side_effect=ValueError('ABI mismatch')), \
                patch.object(helper.os, 'execvp') as execute:
            self.assertEqual(helper.main(['start']), 2)
            execute.assert_not_called()

    def test_running_instance_blocks_start_without_killing_it(self):
        with patch.object(helper, 'preflight', return_value=({}, None)), \
                patch.object(helper, 'ensure_stopped', side_effect=ValueError('existing execution')), \
                patch.object(helper.os, 'execvp') as execute:
            self.assertEqual(helper.main(['start']), 2)
            execute.assert_not_called()

    def test_status_duration_is_bounded_before_ros_import(self):
        for seconds in ('0', '-1', '31', 'nan', 'inf'):
            with self.subTest(seconds=seconds), patch.object(helper, 'preflight') as check:
                self.assertEqual(helper.main(['status', '--seconds', seconds]), 2)
                check.assert_not_called()

    def test_observed_feedback_rejects_simulation_staleness_future_and_stop_failure(self):
        stamp = lambda value: NS(sec=int(value), nanosec=int((value-int(value))*1e9))
        execution = NS(simulated=False, runtime_mode='real', device_backend='snowbot_vendor',
                       stamp=stamp(100.), feedback_stamp=stamp(100.), profile_hash='profile', stop_failures=[])
        mode = NS(runtime_mode='real', profile_hash='profile', stamp=stamp(100.))
        self.assertEqual(helper.assess_state(execution, mode, 100.1, 'profile'), [])
        for change in (dict(simulated=True), dict(runtime_mode='simulation'),
                       dict(feedback_stamp=stamp(99.)), dict(feedback_stamp=stamp(101.)),
                       dict(profile_hash='other'), dict(stop_failures=['waist'])):
            with self.subTest(change=change):
                altered = copy.deepcopy(execution)
                altered.__dict__.update(change)
                self.assertTrue(helper.assess_state(altered, mode, 100.1, 'profile'))
        self.assertTrue(helper.assess_state(None, None, 100.1, 'profile'))

    def test_wrapper_help_and_ros_distro_mismatch_need_no_install(self):
        wrapper = str(ROOT/'tools/snowbot_teleop.sh')
        result = subprocess.run(['bash', wrapper, 'check', '--help'], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        import os
        env = dict(os.environ, ROS_DISTRO='jazzy', BINDU_ROS_DISTRO='humble')
        result = subprocess.run(['bash', wrapper, 'start'], env=env, capture_output=True, text=True)
        self.assertEqual(result.returncode, 2)
        self.assertIn('不一致', result.stderr)


if __name__ == '__main__':
    unittest.main()
