"""Portable deployment checks; never start a controller or touch a device."""
import json
import os
from pathlib import Path
import subprocess
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

from tools.deployment import runtime_check, x86_real


class X86DeploymentTests(unittest.TestCase):
    def test_supported_os_python_pairs_and_abi_rejection(self):
        for release, version, distro in [('22.04', (3, 10), 'humble'), ('24.04', (3, 12), 'jazzy')]:
            info = dict(ID='ubuntu', VERSION_ID=release)
            self.assertEqual(x86_real.target_distro(info, 'x86_64', version), distro)
            for machine, python in [('aarch64', version), ('x86_64', (3, 11))]:
                with self.assertRaises(ValueError):
                    x86_real.target_distro(info, machine, python)
        with self.assertRaises(ValueError):
            x86_real.target_distro(dict(ID='debian', VERSION_ID='12'), 'x86_64', (3, 12))

    def test_clean_environment_keeps_dds_and_drops_old_overlays(self):
        values = dict(PYTHONPATH='/old', AMENT_PREFIX_PATH='/old', CONDA_PREFIX='/old',
                      BINDU_VR_CONFIG='simulation.json', BINDU_VIEWER_DEPS='/old',
                      ROS_DISTRO='humble', ROS_DOMAIN_ID='193', RMW_IMPLEMENTATION='rmw_cyclonedds_cpp')
        with patch.dict(os.environ, values, clear=True):
            env = x86_real.clean_environment()
        for key in values:
            if key not in ('ROS_DOMAIN_ID', 'RMW_IMPLEMENTATION'):
                self.assertNotIn(key, env)
        self.assertEqual(env['ROS_DOMAIN_ID'], '193')
        self.assertEqual(env['RMW_IMPLEMENTATION'], 'rmw_cyclonedds_cpp')
        self.assertEqual(env['PATH'], x86_real.SYSTEM_PATH)

    def test_generated_environment_quotes_paths_and_removes_overlay_chain(self):
        with tempfile.TemporaryDirectory(prefix="bindu ' $ ") as tmp:
            workspace = Path(tmp)
            vendor = workspace/'vendor local_setup.bash'
            vendor.write_text('export VENDOR_LOADED=1\n')
            (workspace/'venv/bin').mkdir(parents=True)
            (workspace/'venv/bin/activate').write_text('export VENV_LOADED=1\n')
            profile = dict(ros='jazzy', python=[3, 12], vendor_setup=str(vendor))
            script = x86_real.environment_script(workspace, profile)
            # Stand-in for ROS setup; exercise the generated shell, including quoting.
            script = script.replace('source /opt/ros/jazzy/setup.bash || return', ':')
            env_file = workspace/'env.sh'
            env_file.write_text(script)
            command = ['bash', '--noprofile', '--norc', '-ec',
                       'source "$1"; /usr/bin/python3 -c '\
                       "'import os,json; print(json.dumps(dict(os.environ)))'", 'test', str(env_file)]
            env = dict(os.environ, PYTHONPATH='/old', AMENT_PREFIX_PATH='/old')
            result = json.loads(subprocess.check_output(command, env=env, text=True))
            self.assertNotIn('AMENT_PREFIX_PATH', result)
            self.assertNotIn('/old', result['PYTHONPATH'])
            self.assertEqual(result['BINDU_VR_CONFIG'], str(workspace/'site.json'))
            self.assertEqual(result['VENDOR_LOADED'], '1')
            self.assertEqual(result['VENV_LOADED'], '1')

    def test_vr_wrapper_keeps_stdlib_before_viewer_backports(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root/'argparse.py').write_text('raise RuntimeError("obsolete argparse imported")\n')
            env_file = root/'env.sh'
            import shlex
            env_file.write_text('export BINDU_VIEWER_DEPS='+shlex.quote(str(root))+'\n')
            env = dict(os.environ, BINDU_VR_ENV=str(env_file), PYTHONPATH='', PYTHONDONTWRITEBYTECODE='1')
            result = subprocess.run(['bash', str(x86_real.ROOT/'tools/run_vr.sh'), '--unknown-flag'],
                                    cwd=root, env=env, text=True, capture_output=True)
            self.assertEqual(result.returncode, 2)
            self.assertIn('unrecognized arguments', result.stderr)
            self.assertNotIn('obsolete argparse imported', result.stderr)

    def test_workspace_identity_rejects_moved_repo_and_os(self):
        with tempfile.TemporaryDirectory() as tmp:
            workspace = Path(tmp)
            data = dict(schema=1, root=str(x86_real.ROOT), workspace=str(workspace),
                        ros='jazzy', python=[3, 12], ready=True)
            with patch.object(x86_real, 'system_target', return_value=('jazzy', [3, 12])):
                for key, value in [('root', '/another/checkout'), ('ros', 'humble'), ('python', [3, 10])]:
                    (workspace/'deployment.json').write_text(json.dumps(dict(data, **{key: value})))
                    with self.assertRaises(ValueError):
                        x86_real.read_profile(workspace)
                (workspace/'deployment.json').write_text(json.dumps(data))
                self.assertTrue(x86_real.read_profile(workspace)['ready'])

    def test_incomplete_prepare_and_simulation_cannot_start(self):
        for command in ('check', 'start'):
            with patch.object(x86_real, 'read_profile', return_value=dict(ready=False)), \
                    patch.object(x86_real, 'in_environment') as execute:
                self.assertEqual(x86_real.main([command]), 2)
                execute.assert_not_called()
        with tempfile.TemporaryDirectory() as tmp:
            workspace = Path(tmp)
            (workspace/'install').mkdir()
            (workspace/'install/local_setup.bash').touch()
            (workspace/'site.json').write_text(json.dumps(dict(arguments=dict(runtime_mode='simulation'))))
            with patch.object(x86_real, 'read_profile', return_value=dict(ready=True)), \
                    patch.object(x86_real, 'require_current_build'), \
                    patch.object(x86_real, 'in_environment') as execute:
                self.assertEqual(x86_real.main(['start', '--workspace', tmp]), 2)
                execute.assert_not_called()

    def test_check_uses_site_and_passes_check_flag_without_start(self):
        with tempfile.TemporaryDirectory() as tmp:
            workspace = Path(tmp)
            (workspace/'install').mkdir()
            (workspace/'install/local_setup.bash').touch()
            (workspace/'site.json').write_text(x86_real.TEMPLATE.read_text())
            with patch.object(x86_real, 'read_profile', return_value=dict(ready=True)), \
                    patch.object(x86_real, 'require_current_build'), \
                    patch.object(x86_real, 'in_environment') as execute:
                self.assertEqual(x86_real.main(['check', '--workspace', tmp, '--domain-id', '193']), 0)
                command = execute.call_args.args[1]
                self.assertIn('--check', command)
                self.assertIn(workspace/'site.json', command)
                self.assertEqual(json.loads((workspace/'site.json').read_text())['arguments']['command_enabled'], False)
                execute.reset_mock()
                self.assertEqual(x86_real.main(['start', '--workspace', tmp, '--physics']), 2)
                execute.assert_not_called()

    def test_changed_source_and_failed_build_require_rebuild(self):
        with tempfile.TemporaryDirectory() as tmp:
            workspace = Path(tmp)
            with patch.object(x86_real, 'source_digest', return_value='current'):
                for info in (dict(ready=False, source_sha256='current'),
                             dict(ready=True, source_sha256='old')):
                    (workspace/'build.json').write_text(json.dumps(info))
                    with self.assertRaisesRegex(ValueError, '重新 build'):
                        x86_real.require_current_build(workspace)
                (workspace/'build.json').write_text(json.dumps(dict(ready=True, source_sha256='current')))
                x86_real.require_current_build(workspace)

    def test_prepare_retry_preserves_site_and_marks_failed_dependency_update(self):
        with tempfile.TemporaryDirectory() as tmp:
            workspace = Path(tmp)/'deploy'
            vendor = Path(tmp)/'local_setup.bash'
            vendor.touch()
            options = NS(workspace=workspace, vendor_setup=vendor, ros_distro=None)
            original_is_file = Path.is_file
            def is_file(path):
                return str(path) == '/opt/ros/jazzy/setup.bash' or original_is_file(path)
            with patch.object(x86_real, 'system_target', return_value=('jazzy', [3, 12])), \
                    patch.object(Path, 'is_file', is_file), \
                    patch.object(x86_real, 'in_environment'), \
                    patch.object(x86_real.subprocess, 'run') as run:
                x86_real.prepare(options)
                self.assertTrue(json.loads((workspace/'deployment.json').read_text())['ready'])
                site = workspace/'site.json'
                site.write_text('reviewed site config\n')
                run.side_effect = subprocess.CalledProcessError(1, 'pip')
                with self.assertRaises(subprocess.CalledProcessError):
                    x86_real.prepare(options)
                self.assertFalse(json.loads((workspace/'deployment.json').read_text())['ready'])
                self.assertEqual(site.read_text(), 'reviewed site config\n')
                run.side_effect = None
                x86_real.prepare(options)
                self.assertTrue(json.loads((workspace/'deployment.json').read_text())['ready'])
                self.assertEqual(site.read_text(), 'reviewed site config\n')


class RuntimeCheckTests(unittest.TestCase):
    def test_detects_stale_pico_and_gripper_but_allows_site_overrides(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            source = root/'src/integration/bindu_runtime'
            installed = root/'install/share/bindu_runtime'
            source.mkdir(parents=True)
            installed.mkdir(parents=True)
            for filename in ('pico.json', 'gripper.json'):
                (source/filename).write_text('new')
                (installed/filename).write_text('old')
                with patch.object(runtime_check, 'ROOT', root):
                    with self.assertRaisesRegex(ValueError, '请重建'):
                        runtime_check.check_installed_configs([installed/filename], installed)
                    runtime_check.check_installed_configs([source/filename], installed)

    def native_fixture(self, root):
        tool = root/'tool.json'
        tool.write_text('{}')
        robot = NS(tool_config=tool, urdf=root/'robot.urdf', profile=object(), profile_path=root/'profile.json')
        package = NS(**{name: object() for name in (
            'ExecutionState', 'VRInput', 'TeleopModeState', 'GripperState', 'SelectTeleopMode',
            'TeleopSession', 'GripperControl', 'JointPositionCommand', 'RetInformation')})
        solver = Mock()
        support = Mock()
        modules = {'ament_index_python.packages': NS(get_package_share_directory=lambda _: str(root)),
                   'rclpy.type_support': NS(check_for_type_support=support),
                   'bindu_kinematics.bimanual': NS(BimanualQP=solver)}
        return robot, package, solver, support, modules

    def test_loads_native_execution_vendor_types_and_qp_then_closes(self):
        with tempfile.TemporaryDirectory() as tmp:
            robot, package, solver, support, modules = self.native_fixture(Path(tmp))
            with patch.dict('sys.modules', modules), patch.object(runtime_check, 'check_installed_configs'), \
                    patch.object(runtime_check.importlib, 'import_module', return_value=package) as load:
                runtime_check.check_runtime(dict(runtime_mode='real', robot_config='robot.json'), robot)
            self.assertIn(('bindu_execution.interpolation',), [call.args for call in load.call_args_list])
            self.assertIn(('snowbot_ctl_msgs.msg',), [call.args for call in load.call_args_list])
            self.assertEqual(support.call_count, 9)
            solver.return_value.close.assert_called_once()

    def test_missing_execution_native_or_qp_symbol_fails_preflight(self):
        with tempfile.TemporaryDirectory() as tmp:
            robot, package, solver, _, modules = self.native_fixture(Path(tmp))
            for broken in ('execution', 'qp'):
                def load(name):
                    if broken == 'execution' and name == 'bindu_execution.interpolation':
                        raise ImportError('missing _native')
                    return package
                solver.side_effect = ValueError('missing bq_shared_motion_weight') if broken == 'qp' else None
                with patch.dict('sys.modules', modules), patch.object(runtime_check, 'check_installed_configs'), \
                        patch.object(runtime_check.importlib, 'import_module', side_effect=load):
                    with self.assertRaises((ImportError, ValueError)):
                        runtime_check.check_runtime(dict(runtime_mode='real', robot_config='robot.json'), robot)


if __name__ == '__main__':
    unittest.main()
